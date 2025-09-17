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
 *********************************************************************/

#include <mpc_local_planner/utils/trajectory_analyzer.h>
#include <tf2/utils.h>
#include <corbo-core/console.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace mpc_local_planner {

TrajectoryAnalyzer::TrajectoryMetrics TrajectoryAnalyzer::analyzeTrajectory(
    const corbo::TimeSeries::Ptr& x_seq,
    const corbo::TimeSeries::Ptr& u_seq,
    const geometry_msgs::PoseStamped* goal_pose)
{
    TrajectoryMetrics metrics;
    
    if (!x_seq || !u_seq || x_seq->isEmpty() || u_seq->isEmpty()) {
        return metrics; // Invalid trajectory
    }
    
    metrics.trajectory_valid = true;
    
    // Calculate total time
    std::vector<double> x_times, u_times;
    x_seq->getTimeAxis(x_times);
    u_seq->getTimeAxis(u_times);
    
    if (!x_times.empty()) {
        metrics.total_time = x_times.back() - x_times.front();
    }
    
    // Calculate path length
    metrics.total_distance = calculatePathLength(x_seq);
    
    // Calculate velocity metrics
    metrics.max_velocity = calculateMaxVelocity(u_seq);
    if (metrics.total_time > 0) {
        metrics.average_velocity = metrics.total_distance / metrics.total_time;
    }
    
    // Calculate acceleration metrics
    metrics.max_acceleration = calculateMaxAcceleration(u_seq);
    
    // Calculate angular velocity
    for (const auto& t : u_times) {
        Eigen::VectorXd u;
        if (u_seq->getValuesInterpolate(t, u) && u.size() >= 2) {
            metrics.max_angular_velocity = std::max(metrics.max_angular_velocity, std::abs(u[1]));
        }
    }
    
    // Calculate curvature
    std::vector<double> curvatures;
    metrics.max_curvature = calculateCurvature(x_seq, curvatures);
    
    // Calculate smoothness
    metrics.smoothness = calculateSmoothness(u_seq);
    
    // Calculate goal accuracy if goal is provided
    if (goal_pose) {
        calculateGoalAccuracy(x_seq, *goal_pose, metrics.goal_distance_error, metrics.goal_angle_error);
    }
    
    return metrics;
}

double TrajectoryAnalyzer::calculatePathLength(const corbo::TimeSeries::Ptr& x_seq)
{
    if (!x_seq || x_seq->isEmpty()) return 0.0;
    
    std::vector<double> times;
    x_seq->getTimeAxis(times);
    
    double total_length = 0.0;
    Eigen::VectorXd prev_state, curr_state;
    
    if (times.size() < 2) return 0.0;
    
    for (size_t i = 1; i < times.size(); ++i) {
        if (x_seq->getValuesInterpolate(times[i-1], prev_state) &&
            x_seq->getValuesInterpolate(times[i], curr_state) &&
            prev_state.size() >= 2 && curr_state.size() >= 2) {
            
            double dx = curr_state[0] - prev_state[0];
            double dy = curr_state[1] - prev_state[1];
            total_length += std::sqrt(dx*dx + dy*dy);
        }
    }
    
    return total_length;
}

double TrajectoryAnalyzer::calculateMaxVelocity(const corbo::TimeSeries::Ptr& u_seq)
{
    if (!u_seq || u_seq->isEmpty()) return 0.0;
    
    std::vector<double> times;
    u_seq->getTimeAxis(times);
    
    double max_vel = 0.0;
    for (const auto& t : times) {
        Eigen::VectorXd u;
        if (u_seq->getValuesInterpolate(t, u) && u.size() >= 1) {
            max_vel = std::max(max_vel, std::abs(u[0]));
        }
    }
    
    return max_vel;
}

double TrajectoryAnalyzer::calculateMaxAcceleration(const corbo::TimeSeries::Ptr& u_seq)
{
    if (!u_seq || u_seq->isEmpty()) return 0.0;
    
    std::vector<double> times;
    u_seq->getTimeAxis(times);
    
    if (times.size() < 2) return 0.0;
    
    double max_acc = 0.0;
    Eigen::VectorXd prev_u, curr_u;
    
    for (size_t i = 1; i < times.size(); ++i) {
        if (u_seq->getValuesInterpolate(times[i-1], prev_u) &&
            u_seq->getValuesInterpolate(times[i], curr_u) &&
            prev_u.size() >= 1 && curr_u.size() >= 1) {
            
            double dt = times[i] - times[i-1];
            if (dt > 1e-6) {
                double acc = std::abs((curr_u[0] - prev_u[0]) / dt);
                max_acc = std::max(max_acc, acc);
            }
        }
    }
    
    return max_acc;
}

double TrajectoryAnalyzer::calculateCurvature(const corbo::TimeSeries::Ptr& x_seq,
                                             std::vector<double>& curvatures)
{
    curvatures.clear();
    if (!x_seq || x_seq->isEmpty()) return 0.0;
    
    std::vector<double> times;
    x_seq->getTimeAxis(times);
    
    if (times.size() < 3) return 0.0;
    
    double max_curvature = 0.0;
    
    for (size_t i = 1; i < times.size() - 1; ++i) {
        Eigen::VectorXd x1, x2, x3;
        if (x_seq->getValuesInterpolate(times[i-1], x1) &&
            x_seq->getValuesInterpolate(times[i], x2) &&
            x_seq->getValuesInterpolate(times[i+1], x3) &&
            x1.size() >= 2 && x2.size() >= 2 && x3.size() >= 2) {
            
            // Calculate curvature using three consecutive points
            double dx1 = x2[0] - x1[0];
            double dy1 = x2[1] - x1[1];
            double dx2 = x3[0] - x2[0];
            double dy2 = x3[1] - x2[1];
            
            double cross = dx1 * dy2 - dy1 * dx2;
            double ds1 = std::sqrt(dx1*dx1 + dy1*dy1);
            double ds2 = std::sqrt(dx2*dx2 + dy2*dy2);
            
            if (ds1 > 1e-6 && ds2 > 1e-6) {
                double curvature = std::abs(cross) / (ds1 * ds2 * (ds1 + ds2) / 2.0);
                curvatures.push_back(curvature);
                max_curvature = std::max(max_curvature, curvature);
            } else {
                curvatures.push_back(0.0);
            }
        }
    }
    
    return max_curvature;
}

double TrajectoryAnalyzer::calculateSmoothness(const corbo::TimeSeries::Ptr& u_seq)
{
    if (!u_seq || u_seq->isEmpty()) return 0.0;
    
    std::vector<double> times;
    u_seq->getTimeAxis(times);
    
    if (times.size() < 3) return 0.0;
    
    double smoothness = 0.0;
    Eigen::VectorXd u1, u2, u3;
    
    for (size_t i = 2; i < times.size(); ++i) {
        if (u_seq->getValuesInterpolate(times[i-2], u1) &&
            u_seq->getValuesInterpolate(times[i-1], u2) &&
            u_seq->getValuesInterpolate(times[i], u3) &&
            u1.size() >= 1 && u2.size() >= 1 && u3.size() >= 1) {
            
            double dt1 = times[i-1] - times[i-2];
            double dt2 = times[i] - times[i-1];
            
            if (dt1 > 1e-6 && dt2 > 1e-6) {
                // Calculate jerk (derivative of acceleration)
                double acc1 = (u2[0] - u1[0]) / dt1;
                double acc2 = (u3[0] - u2[0]) / dt2;
                double jerk = (acc2 - acc1) / ((dt1 + dt2) / 2.0);
                
                smoothness += jerk * jerk;
            }
        }
    }
    
    return smoothness;
}

void TrajectoryAnalyzer::calculateGoalAccuracy(const corbo::TimeSeries::Ptr& x_seq,
                                              const geometry_msgs::PoseStamped& goal_pose,
                                              double& distance_error,
                                              double& angle_error)
{
    distance_error = 0.0;
    angle_error = 0.0;
    
    if (!x_seq || x_seq->isEmpty()) return;
    
    std::vector<double> times;
    x_seq->getTimeAxis(times);
    
    if (times.empty()) return;
    
    // Get final state
    Eigen::VectorXd final_state;
    if (x_seq->getValuesInterpolate(times.back(), final_state) && final_state.size() >= 3) {
        // Calculate distance error
        double dx = final_state[0] - goal_pose.pose.position.x;
        double dy = final_state[1] - goal_pose.pose.position.y;
        distance_error = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angular error
        double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
        angle_error = normalizeAngle(final_state[2] - goal_yaw);
    }
}

bool TrajectoryAnalyzer::checkConstraints(const corbo::TimeSeries::Ptr& u_seq,
                                         double max_vel,
                                         double max_omega,
                                         double max_acc)
{
    if (!u_seq || u_seq->isEmpty()) return false;
    
    double measured_max_vel = calculateMaxVelocity(u_seq);
    double measured_max_acc = calculateMaxAcceleration(u_seq);
    
    std::vector<double> times;
    u_seq->getTimeAxis(times);
    
    // Check angular velocity constraints
    double measured_max_omega = 0.0;
    for (const auto& t : times) {
        Eigen::VectorXd u;
        if (u_seq->getValuesInterpolate(t, u) && u.size() >= 2) {
            measured_max_omega = std::max(measured_max_omega, std::abs(u[1]));
        }
    }
    
    return (measured_max_vel <= max_vel + 1e-6) &&
           (measured_max_omega <= max_omega + 1e-6) &&
           (measured_max_acc <= max_acc + 1e-6);
}

void TrajectoryAnalyzer::sampleTrajectory(const corbo::TimeSeries::Ptr& x_seq,
                                         double dt,
                                         std::vector<geometry_msgs::PoseStamped>& sampled_poses)
{
    sampled_poses.clear();
    if (!x_seq || x_seq->isEmpty() || dt <= 0) return;
    
    std::vector<double> times;
    x_seq->getTimeAxis(times);
    
    if (times.empty()) return;
    
    double t_start = times.front();
    double t_end = times.back();
    
    for (double t = t_start; t <= t_end; t += dt) {
        Eigen::VectorXd state;
        if (x_seq->getValuesInterpolate(t, state) && state.size() >= 3) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time(t);
            pose.pose.position.x = state[0];
            pose.pose.position.y = state[1];
            pose.pose.position.z = 0.0;
            
            // Convert yaw to quaternion
            double yaw = state[2];
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            
            sampled_poses.push_back(pose);
        }
    }
}

void TrajectoryAnalyzer::printAnalysis(const TrajectoryMetrics& metrics)
{
    if (!metrics.trajectory_valid) {
        PRINT_ERROR("Trajectory is invalid!");
        return;
    }
    
    PRINT_INFO("=== Trajectory Analysis ===");
    PRINT_INFO_STREAM("Total Time: " << std::fixed << std::setprecision(3) << metrics.total_time << " s");
    PRINT_INFO_STREAM("Total Distance: " << std::fixed << std::setprecision(3) << metrics.total_distance << " m");
    PRINT_INFO_STREAM("Average Velocity: " << std::fixed << std::setprecision(3) << metrics.average_velocity << " m/s");
    PRINT_INFO_STREAM("Max Velocity: " << std::fixed << std::setprecision(3) << metrics.max_velocity << " m/s");
    PRINT_INFO_STREAM("Max Acceleration: " << std::fixed << std::setprecision(3) << metrics.max_acceleration << " m/s²");
    PRINT_INFO_STREAM("Max Angular Velocity: " << std::fixed << std::setprecision(3) << metrics.max_angular_velocity << " rad/s");
    PRINT_INFO_STREAM("Max Curvature: " << std::fixed << std::setprecision(6) << metrics.max_curvature << " 1/m");
    PRINT_INFO_STREAM("Smoothness: " << std::fixed << std::setprecision(6) << metrics.smoothness);
    PRINT_INFO_STREAM("Goal Distance Error: " << std::fixed << std::setprecision(3) << metrics.goal_distance_error << " m");
    PRINT_INFO_STREAM("Goal Angle Error: " << std::fixed << std::setprecision(3) << metrics.goal_angle_error << " rad");
}

std::string TrajectoryAnalyzer::metricsToString(const TrajectoryMetrics& metrics)
{
    std::ostringstream oss;
    
    if (!metrics.trajectory_valid) {
        oss << "Trajectory is invalid!";
        return oss.str();
    }
    
    oss << "Trajectory Metrics:\n";
    oss << "  Total Time: " << std::fixed << std::setprecision(3) << metrics.total_time << " s\n";
    oss << "  Total Distance: " << std::fixed << std::setprecision(3) << metrics.total_distance << " m\n";
    oss << "  Average Velocity: " << std::fixed << std::setprecision(3) << metrics.average_velocity << " m/s\n";
    oss << "  Max Velocity: " << std::fixed << std::setprecision(3) << metrics.max_velocity << " m/s\n";
    oss << "  Max Acceleration: " << std::fixed << std::setprecision(3) << metrics.max_acceleration << " m/s²\n";
    oss << "  Max Angular Velocity: " << std::fixed << std::setprecision(3) << metrics.max_angular_velocity << " rad/s\n";
    oss << "  Max Curvature: " << std::fixed << std::setprecision(6) << metrics.max_curvature << " 1/m\n";
    oss << "  Smoothness: " << std::fixed << std::setprecision(6) << metrics.smoothness << "\n";
    oss << "  Goal Distance Error: " << std::fixed << std::setprecision(3) << metrics.goal_distance_error << " m\n";
    oss << "  Goal Angle Error: " << std::fixed << std::setprecision(3) << metrics.goal_angle_error << " rad";
    
    return oss.str();
}

void TrajectoryAnalyzer::calculateDerivative(const corbo::TimeSeries::Ptr& ts,
                                            corbo::TimeSeries::Ptr& derivative)
{
    if (!ts || ts->isEmpty()) return;
    
    std::vector<double> times;
    ts->getTimeAxis(times);
    
    if (times.size() < 2) return;
    
    derivative = std::make_shared<corbo::TimeSeries>();
    
    for (size_t i = 1; i < times.size(); ++i) {
        Eigen::VectorXd val1, val2;
        if (ts->getValuesInterpolate(times[i-1], val1) &&
            ts->getValuesInterpolate(times[i], val2) &&
            val1.size() == val2.size()) {
            
            double dt = times[i] - times[i-1];
            if (dt > 1e-6) {
                Eigen::VectorXd deriv = (val2 - val1) / dt;
                derivative->add(times[i], deriv);
            }
        }
    }
}

double TrajectoryAnalyzer::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace mpc_local_planner