/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
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
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <ros/ros.h>

#include <Eigen/Core>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_local_planner_ros.h>
#include <mpc_local_planner/utils/publisher.h>
#include <teb_local_planner/obstacles.h>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <memory>

namespace mpc = mpc_local_planner;

/**
 * @brief Dedicated ROS node for trajectory prediction using MPC
 * 
 * This node provides trajectory prediction services by accepting start and goal poses
 * and computing optimal trajectories using Model Predictive Control (MPC).
 */
class TrajectoryPredictionNode
{
 public:
    TrajectoryPredictionNode() = default;

    void start(ros::NodeHandle& nh);

 protected:
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
    void startCallback(const geometry_msgs::PoseStampedConstPtr& start_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void obstacleCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
    void viaPointsCallback(const nav_msgs::Path::ConstPtr& via_points_msg);
    
    void predictTrajectory();
    void publishPrediction();

    // Core components
    std::unique_ptr<mpc::Controller> controller_;
    std::unique_ptr<mpc::Publisher> publisher_;
    teb_local_planner::RobotFootprintModelPtr robot_model_;
    
    // State variables
    teb_local_planner::PoseSE2 current_pose_;
    teb_local_planner::PoseSE2 goal_pose_;
    geometry_msgs::Twist current_velocity_;
    bool has_goal_;
    bool has_start_;
    bool auto_predict_;
    
    // Prediction results
    corbo::TimeSeries::Ptr predicted_states_;
    corbo::TimeSeries::Ptr predicted_controls_;
    
    // Obstacles and via points
    teb_local_planner::ObstContainer obstacles_;
    std::vector<teb_local_planner::PoseSE2> via_points_;
    
    // ROS interface
    ros::Subscriber goal_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber via_points_sub_;
    ros::Publisher trajectory_pub_;
    ros::Timer prediction_timer_;
    
    // Parameters
    std::string map_frame_;
    double prediction_rate_;
};

void TrajectoryPredictionNode::start(ros::NodeHandle& nh)
{
    // Initialize parameters
    nh.param("map_frame", map_frame_, std::string("map"));
    nh.param("prediction_rate", prediction_rate_, 10.0);
    nh.param("auto_predict", auto_predict_, true);
    
    // Initialize state
    has_goal_ = false;
    has_start_ = false;
    current_pose_ = teb_local_planner::PoseSE2(0, 0, 0);
    goal_pose_ = teb_local_planner::PoseSE2(5, 2, 0);
    
    // Setup robot model
    robot_model_ = mpc::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh);
    
    // Initialize controller
    controller_ = std::make_unique<mpc::Controller>();
    if (!controller_->configure(nh, obstacles_, robot_model_, via_points_))
    {
        ROS_ERROR("Trajectory prediction controller configuration failed.");
        return;
    }
    
    // Initialize publisher
    publisher_ = std::make_unique<mpc::Publisher>(nh, controller_->getRobotDynamics(), map_frame_);
    
    // Initialize time series for results
    predicted_states_ = std::make_shared<corbo::TimeSeries>();
    predicted_controls_ = std::make_shared<corbo::TimeSeries>();
    
    // Setup subscribers
    goal_sub_ = nh.subscribe("goal", 1, &TrajectoryPredictionNode::goalCallback, this);
    start_sub_ = nh.subscribe("start", 1, &TrajectoryPredictionNode::startCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &TrajectoryPredictionNode::odomCallback, this);
    obstacle_sub_ = nh.subscribe("obstacles", 1, &TrajectoryPredictionNode::obstacleCallback, this);
    via_points_sub_ = nh.subscribe("via_points", 1, &TrajectoryPredictionNode::viaPointsCallback, this);
    
    // Setup publishers
    trajectory_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory", 1);
    
    // Setup prediction timer
    if (auto_predict_)
    {
        prediction_timer_ = nh.createTimer(ros::Duration(1.0 / prediction_rate_), 
                                         [this](const ros::TimerEvent&) { predictTrajectory(); });
    }
    
    ROS_INFO("Trajectory Prediction Node initialized");
    ROS_INFO("Listening for goals on: %s", goal_sub_.getTopic().c_str());
    ROS_INFO("Publishing predictions on: %s", trajectory_pub_.getTopic().c_str());
    
    if (!auto_predict_)
    {
        // Manual prediction mode - predict once with default values
        predictTrajectory();
    }
    
    ros::spin();
}

void TrajectoryPredictionNode::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg)
{
    goal_pose_ = teb_local_planner::PoseSE2(goal_msg->pose);
    has_goal_ = true;
    
    ROS_INFO("Received new goal: (%.2f, %.2f, %.2f)", 
             goal_pose_.x(), goal_pose_.y(), goal_pose_.theta());
    
    if (!auto_predict_)
    {
        predictTrajectory();
    }
}

void TrajectoryPredictionNode::startCallback(const geometry_msgs::PoseStampedConstPtr& start_msg)
{
    current_pose_ = teb_local_planner::PoseSE2(start_msg->pose);
    has_start_ = true;
    
    ROS_INFO("Received new start: (%.2f, %.2f, %.2f)", 
             current_pose_.x(), current_pose_.y(), current_pose_.theta());
    
    if (!auto_predict_)
    {
        predictTrajectory();
    }
}

void TrajectoryPredictionNode::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    current_pose_ = teb_local_planner::PoseSE2(odom_msg->pose.pose);
    current_velocity_ = odom_msg->twist.twist;
    has_start_ = true;
}

void TrajectoryPredictionNode::obstacleCallback(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
    // Clear existing obstacles
    obstacles_.clear();
    
    // Add new obstacles
    for (const auto& obstacle : obst_msg->obstacles)
    {
        if (obstacle.polygon.points.size() == 1)
        {
            if (obstacle.radius == 0)
            {
                obstacles_.push_back(std::make_shared<teb_local_planner::PointObstacle>(
                    obstacle.polygon.points.front().x, obstacle.polygon.points.front().y));
            }
            else
            {
                obstacles_.push_back(std::make_shared<teb_local_planner::CircularObstacle>(
                    obstacle.polygon.points.front().x, obstacle.polygon.points.front().y, obstacle.radius));
            }
        }
        else
        {
            auto polyobst = std::make_shared<teb_local_planner::PolygonObstacle>();
            for (const auto& point : obstacle.polygon.points)
            {
                polyobst->pushBackVertex(point.x, point.y);
            }
            polyobst->finalizePolygon();
            obstacles_.push_back(polyobst);
        }
        
        if (!obstacles_.empty())
        {
            obstacles_.back()->setCentroidVelocity(obstacle.velocities, obstacle.orientation);
        }
    }
    
    ROS_DEBUG("Updated %zu obstacles", obstacles_.size());
}

void TrajectoryPredictionNode::viaPointsCallback(const nav_msgs::Path::ConstPtr& via_points_msg)
{
    via_points_.clear();
    for (const auto& pose : via_points_msg->poses)
    {
        via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y, 0);
    }
    
    ROS_INFO("Updated %zu via points", via_points_.size());
}

void TrajectoryPredictionNode::predictTrajectory()
{
    if (!controller_)
    {
        ROS_WARN("Controller not initialized");
        return;
    }
    
    // Use current state and goal for prediction
    bool success = controller_->step(current_pose_, goal_pose_, current_velocity_, 
                                   1.0 / prediction_rate_, ros::Time::now(), 
                                   predicted_controls_, predicted_states_);
    
    if (success)
    {
        publishPrediction();
        ROS_DEBUG("Trajectory prediction successful");
    }
    else
    {
        ROS_WARN("Trajectory prediction failed");
    }
}

void TrajectoryPredictionNode::publishPrediction()
{
    if (!publisher_ || !predicted_states_)
    {
        return;
    }
    
    // Publish the predicted trajectory
    publisher_->publishLocalPlan(*predicted_states_);
    
    // Publish visualization markers
    publisher_->publishObstacles(obstacles_);
    publisher_->publishRobotFootprintModel(current_pose_, *robot_model_);
    publisher_->publishViaPoints(via_points_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_prediction_node");
    ros::NodeHandle nh("~");

    TrajectoryPredictionNode prediction_node;
    prediction_node.start(nh);

    return 0;
}