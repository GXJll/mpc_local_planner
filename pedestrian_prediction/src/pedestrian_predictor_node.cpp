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

#include <ros/ros.h>
#include <pedestrian_prediction/predictor.h>
#include <pedestrian_prediction/PedestrianTrajectory.h>
#include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/TwistArray.h>  // This message type may not exist
#include <geometry_msgs/Twist.h>
#include <mutex>

/**
 * @brief ROS node for pedestrian trajectory prediction
 */
class PedestrianPredictorNode
{
public:
    PedestrianPredictorNode() : nh_("~"), prediction_rate_(10.0)
    {
        // Initialize predictor
        predictor_ = std::make_shared<pedestrian_prediction::PedestrianPredictor>();
        
        if (!predictor_->initialize(nh_))
        {
            ROS_ERROR("Failed to initialize pedestrian predictor");
            return;
        }
        
        // Load node parameters
        loadParameters();
        
        // Setup publishers
        trajectory_pub_ = nh_.advertise<pedestrian_prediction::PedestrianTrajectory>("pedestrian_trajectory", 10);
        trajectories_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("pedestrian_trajectories", 10);
        
        // Setup subscribers for pedestrian detection
        pose_array_sub_ = nh_.subscribe("pedestrian_poses", 10, 
                                       &PedestrianPredictorNode::poseArrayCallback, this);
        // Note: Using PoseArray for now, TwistArray might not be standard ROS message
        // twist_array_sub_ = nh_.subscribe("pedestrian_velocities", 10, 
        //                                &PedestrianPredictorNode::twistArrayCallback, this);
        
        // Setup timer for periodic prediction
        prediction_timer_ = nh_.createTimer(ros::Duration(1.0 / prediction_rate_), 
                                          &PedestrianPredictorNode::predictionTimerCallback, this);
        
        ROS_INFO("Pedestrian Predictor Node initialized successfully");
        ROS_INFO("Subscribing to: %s", 
                 pose_array_sub_.getTopic().c_str());
        ROS_INFO("Publishing to: %s, %s", 
                 trajectory_pub_.getTopic().c_str(), 
                 trajectories_pub_.getTopic().c_str());
    }
    
    ~PedestrianPredictorNode() = default;

private:
    ros::NodeHandle nh_;
    pedestrian_prediction::PedestrianPredictor::Ptr predictor_;
    
    // Parameters
    double prediction_rate_;
    std::string pedestrian_frame_;
    
    // Publishers
    ros::Publisher trajectory_pub_;
    ros::Publisher trajectories_pub_;
    
    // Subscribers
    ros::Subscriber pose_array_sub_;
    // ros::Subscriber twist_array_sub_;  // Commented out for now
    
    // Timer
    ros::Timer prediction_timer_;
    
    // Data storage
    std::map<int, geometry_msgs::PoseStamped> latest_poses_;
    std::map<int, geometry_msgs::Twist> latest_velocities_;
    std::mutex data_mutex_;
    
    void loadParameters()
    {
        nh_.param("prediction_rate", prediction_rate_, prediction_rate_);
        nh_.param("pedestrian_frame", pedestrian_frame_, std::string("map"));
        
        ROS_INFO("Prediction rate: %.1f Hz", prediction_rate_);
        ROS_INFO("Pedestrian frame: %s", pedestrian_frame_.c_str());
    }
    
    void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Update poses for all pedestrians
        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            int pedestrian_id = static_cast<int>(i); // Use array index as ID
            
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.pose = msg->poses[i];
            
            latest_poses_[pedestrian_id] = pose_stamped;
        }
        
        // Update predictor with new poses
        updatePredictorStates();
    }
    
    // Commented out TwistArray callback as the message type may not be standard
    /*
    void twistArrayCallback(const geometry_msgs::TwistArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Update velocities for all pedestrians
        for (size_t i = 0; i < msg->twists.size(); ++i)
        {
            int pedestrian_id = static_cast<int>(i); // Use array index as ID
            latest_velocities_[pedestrian_id] = msg->twists[i];
        }
        
        // Update predictor with new velocities
        updatePredictorStates();
    }
    */
    
    void updatePredictorStates()
    {
        // Update predictor with latest pose and velocity data
        for (const auto& pose_pair : latest_poses_)
        {
            int id = pose_pair.first;
            const geometry_msgs::PoseStamped& pose = pose_pair.second;
            
            geometry_msgs::Twist velocity;
            auto vel_it = latest_velocities_.find(id);
            if (vel_it != latest_velocities_.end())
            {
                velocity = vel_it->second;
            }
            // If no velocity data, use zero velocity
            
            predictor_->updatePedestrianState(id, pose, velocity);
        }
    }
    
    void predictionTimerCallback(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // Generate predictions for all tracked pedestrians
        std::vector<pedestrian_prediction::PedestrianTrajectory> trajectories;
        if (predictor_->predictAllTrajectories(trajectories))
        {
            // Publish individual trajectories
            for (const auto& trajectory : trajectories)
            {
                trajectory_pub_.publish(trajectory);
            }
            
            // Publish summary of pedestrian IDs with predictions
            std_msgs::Int32MultiArray trajectory_ids;
            trajectory_ids.data.clear();
            for (const auto& trajectory : trajectories)
            {
                trajectory_ids.data.push_back(trajectory.pedestrian_id);
            }
            trajectories_pub_.publish(trajectory_ids);
            
            ROS_DEBUG("Published predictions for %zu pedestrians", trajectories.size());
        }
        else
        {
            ROS_DEBUG("No pedestrian trajectories to predict");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pedestrian_predictor_node");
    
    try
    {
        PedestrianPredictorNode node;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in pedestrian predictor node: %s", e.what());
        return 1;
    }
    
    return 0;
}