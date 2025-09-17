#!/usr/bin/env python3
"""
轨迹预测示例脚本 (Trajectory Prediction Example)

This script demonstrates how to use the trajectory prediction node
by sending goals and start positions, and visualizing the results.

使用方法 (Usage):
1. 启动轨迹预测节点: roslaunch mpc_local_planner trajectory_prediction_node.launch
2. 运行此脚本: rosrun mpc_local_planner trajectory_prediction_example.py
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, Polygon, Quaternion
from tf.transformations import quaternion_from_euler

class TrajectoryPredictionExample:
    def __init__(self):
        rospy.init_node('trajectory_prediction_example', anonymous=True)
        
        # Publishers
        self.goal_pub = rospy.Publisher('/trajectory_prediction_node/goal', PoseStamped, queue_size=1)
        self.start_pub = rospy.Publisher('/trajectory_prediction_node/start', PoseStamped, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/trajectory_prediction_node/obstacles', ObstacleArrayMsg, queue_size=1)
        self.via_points_pub = rospy.Publisher('/trajectory_prediction_node/via_points', Path, queue_size=1)
        
        # Subscriber to predicted trajectory
        self.trajectory_sub = rospy.Subscriber('/trajectory_prediction_node/local_plan', Path, self.trajectory_callback)
        
        self.predicted_trajectory = None
        
        rospy.loginfo("Trajectory Prediction Example initialized")
        
    def trajectory_callback(self, msg):
        """处理预测轨迹回调 (Handle predicted trajectory callback)"""
        self.predicted_trajectory = msg
        rospy.loginfo(f"Received predicted trajectory with {len(msg.poses)} poses")
        
    def create_pose_stamped(self, x, y, theta, frame_id="map"):
        """创建PoseStamped消息 (Create PoseStamped message)"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
        
    def send_start_goal(self, start_x, start_y, start_theta, goal_x, goal_y, goal_theta):
        """发送起始点和目标点 (Send start and goal poses)"""
        start_pose = self.create_pose_stamped(start_x, start_y, start_theta)
        goal_pose = self.create_pose_stamped(goal_x, goal_y, goal_theta)
        
        rospy.loginfo(f"Sending start: ({start_x:.2f}, {start_y:.2f}, {start_theta:.2f})")
        rospy.loginfo(f"Sending goal: ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.2f})")
        
        self.start_pub.publish(start_pose)
        rospy.sleep(0.1)  # Small delay to ensure message is sent
        self.goal_pub.publish(goal_pose)
        
    def send_obstacles(self, obstacles):
        """发送障碍物信息 (Send obstacle information)"""
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.frame_id = "map"
        obstacle_msg.header.stamp = rospy.Time.now()
        
        for obs in obstacles:
            obstacle = ObstacleMsg()
            
            if obs['type'] == 'point':
                # Point obstacle
                point = Point32()
                point.x = obs['x']
                point.y = obs['y']
                point.z = 0.0
                obstacle.polygon.points.append(point)
                obstacle.radius = 0.0
                
            elif obs['type'] == 'circle':
                # Circular obstacle
                point = Point32()
                point.x = obs['x']
                point.y = obs['y']
                point.z = 0.0
                obstacle.polygon.points.append(point)
                obstacle.radius = obs['radius']
                
            elif obs['type'] == 'polygon':
                # Polygon obstacle
                for vertex in obs['vertices']:
                    point = Point32()
                    point.x = vertex[0]
                    point.y = vertex[1]
                    point.z = 0.0
                    obstacle.polygon.points.append(point)
                obstacle.radius = 0.0
            
            obstacle_msg.obstacles.append(obstacle)
        
        rospy.loginfo(f"Sending {len(obstacles)} obstacles")
        self.obstacle_pub.publish(obstacle_msg)
        
    def send_via_points(self, via_points):
        """发送通过点 (Send via points)"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for point in via_points:
            pose = self.create_pose_stamped(point[0], point[1], point[2] if len(point) > 2 else 0.0)
            path_msg.poses.append(pose)
        
        rospy.loginfo(f"Sending {len(via_points)} via points")
        self.via_points_pub.publish(path_msg)
        
    def run_example_scenarios(self):
        """运行示例场景 (Run example scenarios)"""
        rospy.sleep(2.0)  # Wait for node to initialize
        
        # Scenario 1: Simple point-to-point trajectory
        rospy.loginfo("=== Scenario 1: Simple point-to-point trajectory ===")
        self.send_start_goal(0.0, 0.0, 0.0, 5.0, 2.0, 0.0)
        rospy.sleep(3.0)
        
        # Scenario 2: Trajectory with obstacles
        rospy.loginfo("=== Scenario 2: Trajectory with obstacles ===")
        obstacles = [
            {'type': 'circle', 'x': 2.0, 'y': 1.0, 'radius': 0.5},
            {'type': 'circle', 'x': 3.5, 'y': 1.5, 'radius': 0.3},
            {'type': 'point', 'x': 4.0, 'y': 0.5}
        ]
        self.send_obstacles(obstacles)
        self.send_start_goal(0.0, 0.0, 0.0, 5.0, 2.0, 0.0)
        rospy.sleep(3.0)
        
        # Scenario 3: Trajectory with via points
        rospy.loginfo("=== Scenario 3: Trajectory with via points ===")
        via_points = [
            (1.5, 0.5),
            (3.0, 1.8),
            (4.5, 1.0)
        ]
        self.send_via_points(via_points)
        self.send_start_goal(0.0, 0.0, 0.0, 5.0, 2.0, 0.0)
        rospy.sleep(3.0)
        
        # Scenario 4: Complex scenario with obstacles and via points
        rospy.loginfo("=== Scenario 4: Complex scenario with obstacles and via points ===")
        complex_obstacles = [
            {'type': 'polygon', 'vertices': [(1.0, 0.5), (1.5, 0.5), (1.5, 1.5), (1.0, 1.5)]},
            {'type': 'circle', 'x': 3.0, 'y': 0.8, 'radius': 0.4}
        ]
        complex_via_points = [
            (2.0, 2.0),
            (4.0, -0.5)
        ]
        self.send_obstacles(complex_obstacles)
        self.send_via_points(complex_via_points)
        self.send_start_goal(0.0, 0.0, 0.0, 5.0, 1.0, math.pi/2)
        rospy.sleep(3.0)
        
        # Scenario 5: Dynamic trajectory updates
        rospy.loginfo("=== Scenario 5: Dynamic trajectory updates ===")
        for i in range(5):
            goal_x = 2.0 + i * 0.5
            goal_y = 1.0 + math.sin(i * 0.5) * 0.5
            self.send_start_goal(0.0, 0.0, 0.0, goal_x, goal_y, 0.0)
            rospy.sleep(1.0)
        
        rospy.loginfo("All example scenarios completed!")

def main():
    try:
        example = TrajectoryPredictionExample()
        
        rospy.loginfo("Starting trajectory prediction examples...")
        rospy.loginfo("Make sure to:")
        rospy.loginfo("1. Launch trajectory prediction node: roslaunch mpc_local_planner trajectory_prediction_node.launch")
        rospy.loginfo("2. Open RVIZ to visualize results")
        rospy.loginfo("3. Wait for examples to run...")
        
        example.run_example_scenarios()
        
        rospy.loginfo("Examples finished. Keeping node alive for visualization...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Trajectory prediction example interrupted")

if __name__ == '__main__':
    main()