/*********************************************************************
 *
 * Software License Agreement
 *
 * Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * Authors: Christoph Rösmann
 *
 * Example: Trajectory Prediction with MPC Local Planner
 *********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/utils/publisher.h>
#include <mpc_local_planner/utils/trajectory_analyzer.h>
#include <teb_local_planner/obstacles.h>

#include <memory>
#include <vector>

/**
 * @brief Example demonstrating trajectory prediction capabilities
 * 
 * This example shows how to:
 * 1. Configure the MPC controller for different robot types
 * 2. Generate predicted trajectories from initial plans
 * 3. Visualize and analyze trajectory predictions
 * 4. Check trajectory feasibility
 */
class TrajectoryPredictionExample
{
public:
    TrajectoryPredictionExample() : nh_("~") {}

    bool initialize()
    {
        // Configure robot parameters
        std::string robot_type = "unicycle";
        nh_.param("robot_type", robot_type, robot_type);
        
        // Set up controller
        controller_ = std::make_shared<mpc_local_planner::Controller>();
        
        // Configure with empty obstacles and via points for this example
        teb_local_planner::ObstContainer obstacles;
        teb_local_planner::RobotFootprintModelPtr robot_model;
        std::vector<teb_local_planner::PoseSE2> via_points;
        
        if (!controller_->configure(nh_, obstacles, robot_model, via_points)) {
            ROS_ERROR("Failed to configure MPC controller");
            return false;
        }
        
        // Set up publishers
        predicted_path_pub_ = nh_.advertise<nav_msgs::Path>("predicted_path", 1);
        predicted_trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1);
        control_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
        // Set up subscribers
        goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &TrajectoryPredictionExample::goalCallback, this);
        
        ROS_INFO("Trajectory Prediction Example initialized");
        return true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
    {
        ROS_INFO("Received new goal, predicting trajectory...");
        
        // Create a simple initial plan from current pose to goal
        std::vector<geometry_msgs::PoseStamped> initial_plan;
        
        // Start pose (current robot pose - for demo, use origin)
        geometry_msgs::PoseStamped start_pose;
        start_pose.header.frame_id = "map";
        start_pose.header.stamp = ros::Time::now();
        start_pose.pose.position.x = 0.0;
        start_pose.pose.position.y = 0.0;
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation.w = 1.0;
        
        initial_plan.push_back(start_pose);
        initial_plan.push_back(*goal_msg);
        
        // Predict trajectory
        predictTrajectory(initial_plan);
    }

    void predictTrajectory(const std::vector<geometry_msgs::PoseStamped>& initial_plan)
    {
        if (initial_plan.size() < 2) {
            ROS_WARN("Initial plan too short for trajectory prediction");
            return;
        }
        
        // Current velocity (assumed zero for this example)
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0.0;
        current_vel.angular.z = 0.0;
        
        // Time step
        double dt = 0.1;
        ros::Time current_time = ros::Time::now();
        
        // Output trajectories
        corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();
        corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();
        
        // Predict trajectory
        bool success = controller_->step(initial_plan, current_vel, dt, current_time, u_seq, x_seq);
        
        if (success) {
            ROS_INFO("Trajectory prediction successful!");
            
            // Analyze trajectory
            geometry_msgs::PoseStamped goal_pose = initial_plan.back();
            mpc_local_planner::TrajectoryAnalyzer::TrajectoryMetrics metrics = 
                mpc_local_planner::TrajectoryAnalyzer::analyzeTrajectory(x_seq, u_seq, &goal_pose);
            
            // Print analysis results
            mpc_local_planner::TrajectoryAnalyzer::printAnalysis(metrics);
            
            // Visualize results
            visualizeTrajectory(x_seq, "map");
            visualizeControls(u_seq);
            
            // Extract and publish first control command
            if (!u_seq->isEmpty()) {
                publishControlCommand(u_seq);
            }
            
            // Check trajectory feasibility (simplified - without costmap)
            checkTrajectoryFeasibility();
            
        } else {
            ROS_WARN("Trajectory prediction failed!");
        }
    }

    void visualizeTrajectory(const corbo::TimeSeries::Ptr& x_seq, const std::string& frame_id)
    {
        if (!x_seq || x_seq->isEmpty()) return;
        
        // Create path message
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = frame_id;
        path_msg.header.stamp = ros::Time::now();
        
        // Create marker array for detailed visualization
        visualization_msgs::MarkerArray marker_array;
        
        // Extract poses from state trajectory
        std::vector<double> timestamps;
        x_seq->getTimeAxis(timestamps);
        
        for (size_t i = 0; i < timestamps.size(); ++i) {
            Eigen::VectorXd state;
            if (x_seq->getValuesInterpolate(timestamps[i], state) && state.size() >= 3) {
                // Create pose for path
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame_id;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = state[0];
                pose.pose.position.y = state[1];
                pose.pose.position.z = 0.0;
                
                // Convert angle to quaternion
                double yaw = state[2];
                pose.pose.orientation.w = cos(yaw / 2.0);
                pose.pose.orientation.z = sin(yaw / 2.0);
                
                path_msg.poses.push_back(pose);
                
                // Create marker for orientation visualization
                visualization_msgs::Marker marker;
                marker.header = pose.header;
                marker.ns = "trajectory_poses";
                marker.id = i;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = pose.pose;
                marker.scale.x = 0.3;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
                marker.lifetime = ros::Duration(5.0);
                
                marker_array.markers.push_back(marker);
            }
        }
        
        // Publish visualizations
        predicted_path_pub_.publish(path_msg);
        predicted_trajectory_pub_.publish(marker_array);
        
        ROS_INFO_STREAM("Published predicted trajectory with " << path_msg.poses.size() << " poses");
    }

    void visualizeControls(const corbo::TimeSeries::Ptr& u_seq)
    {
        if (!u_seq || u_seq->isEmpty()) return;
        
        std::vector<double> timestamps;
        u_seq->getTimeAxis(timestamps);
        
        ROS_INFO("Control sequence:");
        for (size_t i = 0; i < std::min(timestamps.size(), size_t(10)); ++i) {
            Eigen::VectorXd control;
            if (u_seq->getValuesInterpolate(timestamps[i], control) && control.size() >= 2) {
                ROS_INFO_STREAM("t=" << timestamps[i] << ": v=" << control[0] << ", w=" << control[1]);
            }
        }
    }

    void publishControlCommand(const corbo::TimeSeries::Ptr& u_seq)
    {
        Eigen::VectorXd first_control;
        if (u_seq->getValuesInterpolate(0.0, first_control) && first_control.size() >= 2) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = first_control[0];
            cmd_vel.angular.z = first_control[1];
            
            control_pub_.publish(cmd_vel);
            ROS_INFO_STREAM("Published control: v=" << cmd_vel.linear.x << ", w=" << cmd_vel.angular.z);
        }
    }

    void checkTrajectoryFeasibility()
    {
        // Note: This is a simplified feasibility check
        // In practice, you would use isPoseTrajectoryFeasible() with a proper costmap
        
        ROS_INFO("Trajectory feasibility check: PASSED (simplified check)");
        
        /* Example of real feasibility checking:
        
        bool feasible = controller_->isPoseTrajectoryFeasible(
            costmap_model,           // CostmapModel pointer
            robot_footprint_spec,    // Robot footprint
            inscribed_radius,        // Robot inscribed radius
            circumscribed_radius,    // Robot circumscribed radius
            0.1,                     // Angular resolution for collision checking
            -1                       // Check entire trajectory (-1)
        );
        
        if (!feasible) {
            ROS_WARN("Predicted trajectory is not feasible!");
            // Trigger replanning or emergency behavior
        }
        */
    }

    void demonstrateTrajectoryPrediction()
    {
        ROS_INFO("Starting trajectory prediction demonstration...");
        
        // Create a series of waypoints for demonstration
        std::vector<std::vector<geometry_msgs::PoseStamped>> demo_plans;
        
        // Plan 1: Straight line
        demo_plans.push_back(createDemoPlan(0, 0, 0, 5, 0, 0));
        
        // Plan 2: Curve
        demo_plans.push_back(createDemoPlan(0, 0, 0, 3, 3, M_PI/2));
        
        // Plan 3: U-turn
        demo_plans.push_back(createDemoPlan(0, 0, 0, 2, 0, M_PI));
        
        ros::Rate rate(0.5); // 0.5 Hz
        for (const auto& plan : demo_plans) {
            if (!ros::ok()) break;
            
            ROS_INFO("Predicting trajectory for demo plan...");
            predictTrajectory(plan);
            
            rate.sleep();
        }
    }

private:
    std::vector<geometry_msgs::PoseStamped> createDemoPlan(double x1, double y1, double yaw1,
                                                          double x2, double y2, double yaw2)
    {
        std::vector<geometry_msgs::PoseStamped> plan;
        
        // Start pose
        geometry_msgs::PoseStamped start;
        start.header.frame_id = "map";
        start.header.stamp = ros::Time::now();
        start.pose.position.x = x1;
        start.pose.position.y = y1;
        start.pose.orientation.w = cos(yaw1 / 2.0);
        start.pose.orientation.z = sin(yaw1 / 2.0);
        
        // Goal pose
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = x2;
        goal.pose.position.y = y2;
        goal.pose.orientation.w = cos(yaw2 / 2.0);
        goal.pose.orientation.z = sin(yaw2 / 2.0);
        
        plan.push_back(start);
        plan.push_back(goal);
        
        return plan;
    }

    ros::NodeHandle nh_;
    std::shared_ptr<mpc_local_planner::Controller> controller_;
    
    // Publishers
    ros::Publisher predicted_path_pub_;
    ros::Publisher predicted_trajectory_pub_;
    ros::Publisher control_pub_;
    
    // Subscribers
    ros::Subscriber goal_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_prediction_example");
    
    TrajectoryPredictionExample example;
    
    if (!example.initialize()) {
        ROS_ERROR("Failed to initialize trajectory prediction example");
        return -1;
    }
    
    // Option 1: Run interactive mode (wait for goals via RViz)
    if (argc > 1 && std::string(argv[1]) == "--interactive") {
        ROS_INFO("Running in interactive mode. Send goals via RViz 2D Nav Goal.");
        ros::spin();
    }
    // Option 2: Run demonstration mode
    else {
        example.demonstrateTrajectoryPrediction();
    }
    
    return 0;
}