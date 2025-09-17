/*********************************************************************
 *
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2023, Pedestrian Prediction Package
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *********************************************************************/

#include <pedestrian_prediction/predictor.h>
// #include <angles/angles.h>  // May not be available
#include <tf2/utils.h>
#include <cmath>

namespace pedestrian_prediction
{

PedestrianPredictor::PedestrianPredictor() 
    : tf_listener_(tf_buffer_), map_frame_("map"), state_timeout_(5.0), publish_markers_(true)
{
    last_cleanup_time_ = ros::Time::now();
}

PedestrianPredictor::~PedestrianPredictor()
{
}

bool PedestrianPredictor::initialize(ros::NodeHandle& nh)
{
    nh_ = nh;
    
    // Load parameters from parameter server
    loadParameters(nh_);
    
    // Setup visualization publisher
    if (publish_markers_)
    {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("pedestrian_predictions_markers", 10);
    }
    
    ROS_INFO("Pedestrian Predictor initialized successfully");
    return true;
}

void PedestrianPredictor::loadParameters(ros::NodeHandle& nh)
{
    // Load frame parameters
    nh.param("map_frame", map_frame_, map_frame_);
    nh.param("state_timeout", state_timeout_, state_timeout_);
    nh.param("publish_markers", publish_markers_, publish_markers_);
    
    // Load NPSN algorithm parameters
    nh.param("npsn/prediction_horizon", npsn_params_.prediction_horizon, npsn_params_.prediction_horizon);
    nh.param("npsn/time_step", npsn_params_.time_step, npsn_params_.time_step);
    nh.param("npsn/social_radius", npsn_params_.social_radius, npsn_params_.social_radius);
    nh.param("npsn/comfort_zone", npsn_params_.comfort_zone, npsn_params_.comfort_zone);
    nh.param("npsn/max_velocity", npsn_params_.max_velocity, npsn_params_.max_velocity);
    nh.param("npsn/min_velocity", npsn_params_.min_velocity, npsn_params_.min_velocity);
    nh.param("npsn/goal_attraction", npsn_params_.goal_attraction, npsn_params_.goal_attraction);
    nh.param("npsn/obstacle_repulsion", npsn_params_.obstacle_repulsion, npsn_params_.obstacle_repulsion);
    nh.param("npsn/social_force", npsn_params_.social_force, npsn_params_.social_force);
    nh.param("npsn/history_length", npsn_params_.history_length, npsn_params_.history_length);
    
    ROS_INFO("Loaded NPSN parameters: horizon=%.2f, time_step=%.3f, social_radius=%.2f", 
             npsn_params_.prediction_horizon, npsn_params_.time_step, npsn_params_.social_radius);
}

void PedestrianPredictor::updatePedestrianState(int pedestrian_id, 
                                               const geometry_msgs::PoseStamped& pose, 
                                               const geometry_msgs::Twist& velocity)
{
    ros::Time current_time = ros::Time::now();
    
    // Update or create pedestrian state
    PedestrianState& state = pedestrian_states_[pedestrian_id];
    state.id = pedestrian_id;
    state.pose = pose;
    state.velocity = velocity;
    state.last_updated = current_time;
    state.confidence = 1.0; // Default confidence
    
    // Add to history (keep limited history)
    state.history.push_back(pose);
    if (state.history.size() > static_cast<size_t>(npsn_params_.history_length))
    {
        state.history.erase(state.history.begin());
    }
    
    // Cleanup stale states periodically
    if ((current_time - last_cleanup_time_).toSec() > 1.0)
    {
        cleanupStaleStates();
        last_cleanup_time_ = current_time;
    }
}

void PedestrianPredictor::cleanupStaleStates()
{
    ros::Time current_time = ros::Time::now();
    auto it = pedestrian_states_.begin();
    
    while (it != pedestrian_states_.end())
    {
        if ((current_time - it->second.last_updated).toSec() > state_timeout_)
        {
            ROS_DEBUG("Removing stale pedestrian state for ID: %d", it->first);
            it = pedestrian_states_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

bool PedestrianPredictor::predictTrajectory(int pedestrian_id, PedestrianTrajectory& trajectory)
{
    auto it = pedestrian_states_.find(pedestrian_id);
    if (it == pedestrian_states_.end())
    {
        ROS_WARN("Pedestrian ID %d not found in tracked states", pedestrian_id);
        return false;
    }
    
    const PedestrianState& state = it->second;
    
    // Initialize trajectory message
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = map_frame_;
    trajectory.pedestrian_id = pedestrian_id;
    trajectory.pedestrian_label = "pedestrian_" + std::to_string(pedestrian_id);
    trajectory.current_pose = state.pose;
    trajectory.current_velocity = state.velocity;
    trajectory.prediction_horizon = npsn_params_.prediction_horizon;
    trajectory.time_step = npsn_params_.time_step;
    trajectory.prediction_method = "NPSN";
    trajectory.is_valid = true;
    trajectory.uncertainty = 0.1; // Default uncertainty
    
    // Predict trajectory using NPSN algorithm
    int num_steps = static_cast<int>(npsn_params_.prediction_horizon / npsn_params_.time_step);
    trajectory.predicted_poses.clear();
    trajectory.prediction_times.clear();
    trajectory.confidence_scores.clear();
    
    geometry_msgs::PoseStamped current_pose = state.pose;
    geometry_msgs::Twist current_velocity = state.velocity;
    
    for (int i = 0; i < num_steps; ++i)
    {
        double prediction_time = i * npsn_params_.time_step;
        
        // Compute forces using NPSN algorithm
        Eigen::Vector2d social_force(0.0, 0.0);
        Eigen::Vector2d goal_force(0.0, 0.0);
        Eigen::Vector2d obstacle_force(0.0, 0.0);
        
        computeSocialForce(pedestrian_id, state, social_force);
        computeGoalForce(current_pose, current_velocity, goal_force);
        computeObstacleForce(current_pose, obstacle_force);
        
        // Integrate motion
        geometry_msgs::PoseStamped next_pose;
        geometry_msgs::Twist next_velocity;
        
        // Simple integration - in a real implementation, this would use the NPSN neural network
        next_pose = current_pose;
        next_velocity = current_velocity;
        
        // Update position based on velocity
        next_pose.pose.position.x += current_velocity.linear.x * npsn_params_.time_step;
        next_pose.pose.position.y += current_velocity.linear.y * npsn_params_.time_step;
        next_pose.header.stamp = ros::Time::now() + ros::Duration(prediction_time);
        
        // Validate velocity bounds
        validateVelocity(next_velocity);
        
        // Store prediction
        trajectory.predicted_poses.push_back(next_pose);
        trajectory.prediction_times.push_back(prediction_time);
        trajectory.confidence_scores.push_back(std::max(0.1, 1.0 - prediction_time / npsn_params_.prediction_horizon));
        
        // Update for next iteration
        current_pose = next_pose;
        current_velocity = next_velocity;
    }
    
    return true;
}

bool PedestrianPredictor::predictAllTrajectories(std::vector<PedestrianTrajectory>& trajectories)
{
    trajectories.clear();
    
    for (const auto& state_pair : pedestrian_states_)
    {
        PedestrianTrajectory trajectory;
        if (predictTrajectory(state_pair.first, trajectory))
        {
            trajectories.push_back(trajectory);
        }
    }
    
    // Publish visualization if enabled
    if (publish_markers_ && !trajectories.empty())
    {
        publishVisualization(trajectories);
    }
    
    return !trajectories.empty();
}

void PedestrianPredictor::computeSocialForce(int target_id, const PedestrianState& target_state, 
                                            Eigen::Vector2d& social_force)
{
    social_force.setZero();
    
    // Compute social forces from other pedestrians within social radius
    Eigen::Vector2d target_pos(target_state.pose.pose.position.x, target_state.pose.pose.position.y);
    
    for (const auto& other_state : pedestrian_states_)
    {
        if (other_state.first == target_id) continue;
        
        Eigen::Vector2d other_pos(other_state.second.pose.pose.position.x, 
                                 other_state.second.pose.pose.position.y);
        Eigen::Vector2d diff = target_pos - other_pos;
        double distance = diff.norm();
        
        if (distance < npsn_params_.social_radius && distance > 0.1)
        {
            // Repulsive force inversely proportional to distance
            double force_magnitude = npsn_params_.social_force * std::exp(-distance / npsn_params_.comfort_zone);
            social_force += force_magnitude * diff.normalized();
        }
    }
}

void PedestrianPredictor::computeGoalForce(const geometry_msgs::PoseStamped& current_pose,
                                          const geometry_msgs::Twist& velocity,
                                          Eigen::Vector2d& goal_force)
{
    // Simple goal force - maintain current direction with slight forward bias
    goal_force(0) = npsn_params_.goal_attraction * velocity.linear.x;
    goal_force(1) = npsn_params_.goal_attraction * velocity.linear.y;
}

void PedestrianPredictor::computeObstacleForce(const geometry_msgs::PoseStamped& current_pose,
                                              Eigen::Vector2d& obstacle_force)
{
    // Placeholder for obstacle force computation
    // In a real implementation, this would use costmap or obstacle detection
    obstacle_force.setZero();
}

void PedestrianPredictor::validateVelocity(geometry_msgs::Twist& velocity)
{
    // Clamp linear velocities to reasonable bounds
    double vel_magnitude = std::sqrt(velocity.linear.x * velocity.linear.x + 
                                   velocity.linear.y * velocity.linear.y);
    
    if (vel_magnitude > npsn_params_.max_velocity)
    {
        double scale = npsn_params_.max_velocity / vel_magnitude;
        velocity.linear.x *= scale;
        velocity.linear.y *= scale;
    }
    else if (vel_magnitude < npsn_params_.min_velocity && vel_magnitude > 0.01)
    {
        double scale = npsn_params_.min_velocity / vel_magnitude;
        velocity.linear.x *= scale;
        velocity.linear.y *= scale;
    }
    
    // Clamp angular velocity
    velocity.angular.z = std::max(-1.0, std::min(1.0, velocity.angular.z));
}

void PedestrianPredictor::publishVisualization(const std::vector<PedestrianTrajectory>& trajectories)
{
    if (!publish_markers_ || trajectories.empty()) return;
    
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    for (const auto& trajectory : trajectories)
    {
        // Create trajectory line markers
        visualization_msgs::Marker line_marker;
        line_marker.header = trajectory.header;
        line_marker.ns = "pedestrian_trajectories";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.lifetime = ros::Duration(1.0);
        
        line_marker.scale.x = 0.05; // Line width
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        // Add current position
        geometry_msgs::Point start_point;
        start_point.x = trajectory.current_pose.pose.position.x;
        start_point.y = trajectory.current_pose.pose.position.y;
        start_point.z = trajectory.current_pose.pose.position.z;
        line_marker.points.push_back(start_point);
        
        // Add predicted positions
        for (const auto& pose : trajectory.predicted_poses)
        {
            geometry_msgs::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            line_marker.points.push_back(point);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    marker_pub_.publish(marker_array);
}

} // namespace pedestrian_prediction