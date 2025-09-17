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

#ifndef PEDESTRIAN_PREDICTION_PREDICTOR_H
#define PEDESTRIAN_PREDICTION_PREDICTOR_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pedestrian_prediction/PedestrianTrajectory.h>

#include <vector>
#include <map>
#include <memory>
#include <Eigen/Dense>

namespace pedestrian_prediction
{

/**
 * @brief Structure to hold pedestrian state information
 */
struct PedestrianState
{
    int id;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist velocity;
    ros::Time last_updated;
    std::vector<geometry_msgs::PoseStamped> history;
    double confidence;
    
    PedestrianState() : id(-1), confidence(0.0) {}
};

/**
 * @brief NPSN (Neural Pedestrian Social Network) prediction parameters
 */
struct NPSNParams
{
    double prediction_horizon;    // Prediction time horizon in seconds
    double time_step;            // Time step for prediction in seconds
    double social_radius;        // Social interaction radius in meters
    double comfort_zone;         // Personal comfort zone radius in meters
    double max_velocity;         // Maximum pedestrian velocity in m/s
    double min_velocity;         // Minimum pedestrian velocity in m/s
    double goal_attraction;      // Goal attraction weight
    double obstacle_repulsion;   // Obstacle repulsion weight
    double social_force;         // Social force weight
    int history_length;          // Number of historical poses to consider
    
    NPSNParams() : prediction_horizon(3.0), time_step(0.1), social_radius(5.0),
                   comfort_zone(1.2), max_velocity(2.0), min_velocity(0.1),
                   goal_attraction(1.0), obstacle_repulsion(2.0), social_force(1.5),
                   history_length(10) {}
};

/**
 * @brief Main pedestrian trajectory predictor class implementing NPSN algorithm
 */
class PedestrianPredictor
{
public:
    using Ptr = std::shared_ptr<PedestrianPredictor>;
    
    /**
     * @brief Constructor
     */
    PedestrianPredictor();
    
    /**
     * @brief Destructor
     */
    ~PedestrianPredictor();
    
    /**
     * @brief Initialize the predictor with ROS node handle
     * @param nh ROS node handle
     * @return true if initialization successful, false otherwise
     */
    bool initialize(ros::NodeHandle& nh);
    
    /**
     * @brief Update pedestrian state from detection
     * @param pedestrian_id Unique pedestrian identifier
     * @param pose Current pose of the pedestrian
     * @param velocity Current velocity of the pedestrian
     */
    void updatePedestrianState(int pedestrian_id, const geometry_msgs::PoseStamped& pose, 
                              const geometry_msgs::Twist& velocity);
    
    /**
     * @brief Predict trajectory for a specific pedestrian using NPSN algorithm
     * @param pedestrian_id Pedestrian identifier
     * @param trajectory Output predicted trajectory
     * @return true if prediction successful, false otherwise
     */
    bool predictTrajectory(int pedestrian_id, PedestrianTrajectory& trajectory);
    
    /**
     * @brief Predict trajectories for all tracked pedestrians
     * @param trajectories Output vector of predicted trajectories
     * @return true if prediction successful, false otherwise
     */
    bool predictAllTrajectories(std::vector<PedestrianTrajectory>& trajectories);
    
    /**
     * @brief Get current pedestrian states
     * @return Map of pedestrian states keyed by ID
     */
    const std::map<int, PedestrianState>& getPedestrianStates() const { return pedestrian_states_; }
    
    /**
     * @brief Set NPSN prediction parameters
     * @param params NPSN parameters structure
     */
    void setNPSNParams(const NPSNParams& params) { npsn_params_ = params; }
    
    /**
     * @brief Get current NPSN parameters
     * @return Current NPSN parameters
     */
    const NPSNParams& getNPSNParams() const { return npsn_params_; }

private:
    // ROS communication
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_;
    
    // Pedestrian tracking
    std::map<int, PedestrianState> pedestrian_states_;
    ros::Time last_cleanup_time_;
    double state_timeout_;  // Time after which a pedestrian state is considered stale
    
    // NPSN algorithm parameters
    NPSNParams npsn_params_;
    
    // Visualization
    ros::Publisher marker_pub_;
    bool publish_markers_;
    
    /**
     * @brief Load parameters from ROS parameter server
     * @param nh ROS node handle
     */
    void loadParameters(ros::NodeHandle& nh);
    
    /**
     * @brief Clean up stale pedestrian states
     */
    void cleanupStaleStates();
    
    /**
     * @brief Compute social forces between pedestrians (NPSN core algorithm)
     * @param target_id Target pedestrian ID
     * @param target_state Current state of target pedestrian
     * @param social_force Output social force vector
     */
    void computeSocialForce(int target_id, const PedestrianState& target_state, 
                           Eigen::Vector2d& social_force);
    
    /**
     * @brief Compute goal attraction force
     * @param current_pose Current pedestrian pose
     * @param velocity Current pedestrian velocity
     * @param goal_force Output goal attraction force
     */
    void computeGoalForce(const geometry_msgs::PoseStamped& current_pose,
                         const geometry_msgs::Twist& velocity,
                         Eigen::Vector2d& goal_force);
    
    /**
     * @brief Compute obstacle repulsion force
     * @param current_pose Current pedestrian pose
     * @param obstacle_force Output obstacle repulsion force
     */
    void computeObstacleForce(const geometry_msgs::PoseStamped& current_pose,
                             Eigen::Vector2d& obstacle_force);
    
    /**
     * @brief Integrate motion using computed forces
     * @param current_state Current pedestrian state
     * @param dt Time step
     * @param predicted_pose Output predicted pose
     * @param predicted_velocity Output predicted velocity
     */
    void integrateMotion(const PedestrianState& current_state, double dt,
                        geometry_msgs::PoseStamped& predicted_pose,
                        geometry_msgs::Twist& predicted_velocity);
    
    /**
     * @brief Publish visualization markers for predicted trajectories
     * @param trajectories Vector of predicted trajectories
     */
    void publishVisualization(const std::vector<PedestrianTrajectory>& trajectories);
    
    /**
     * @brief Validate and clamp velocity to reasonable bounds
     * @param velocity Input/output velocity to validate
     */
    void validateVelocity(geometry_msgs::Twist& velocity);
};

} // namespace pedestrian_prediction

#endif // PEDESTRIAN_PREDICTION_PREDICTOR_H