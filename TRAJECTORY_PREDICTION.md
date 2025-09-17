# 轨迹预测功能 (Trajectory Prediction Functionality)

本仓库包含完整的基于模型预测控制(MPC)的轨迹预测功能，提供了多个ROS节点来实现轨迹预测和可视化。

## 主要功能 (Main Features)

### 1. 轨迹预测节点 (Trajectory Prediction Nodes)

#### `trajectory_prediction_node` - 专用轨迹预测节点
- **功能**: 专门用于轨迹预测的ROS节点
- **输入**: 起始点、目标点、障碍物信息、通过点
- **输出**: 预测轨迹、可视化标记
- **特点**: 实时轨迹预测，支持动态障碍物避让

#### `test_mpc_optim_node` - MPC优化测试节点  
- **功能**: 演示MPC轨迹优化的测试节点
- **特点**: 交互式障碍物，通过点支持，完整的可视化

#### `mpc_local_planner_ros` - MPC局部规划器
- **功能**: ROS navigation stack的局部规划器插件
- **特点**: 完整的路径跟踪和轨迹预测功能

### 2. 核心组件 (Core Components)

#### Controller类
- 实现MPC控制器逻辑
- 支持多种机器人动力学模型（独轮车、简单车辆、自行车模型）
- 轨迹优化和预测

#### Publisher类
- 发布预测轨迹到ROS话题
- 可视化机器人足迹、障碍物、通过点
- 支持多种可视化格式

#### 时间序列处理
- SE2空间的轨迹表示
- 轨迹插值和外推
- 角度正规化处理

### 3. 消息类型 (Message Types)

#### OptimalControlResult.msg
```
std_msgs/Header header
int64 dim_states
int64 dim_controls
float64[] time_states
float64[] states       # 状态轨迹 (列主序)
float64[] time_controls
float64[] controls     # 控制轨迹 (列主序)
bool optimal_solution_found
float64 cpu_time
```

#### StateFeedback.msg
```
std_msgs/Header header
float64[] state        # 状态反馈
```

## 使用方法 (Usage)

### 1. 启动轨迹预测节点

```bash
# 启动专用轨迹预测节点
roslaunch mpc_local_planner trajectory_prediction_node.launch

# 启动MPC优化测试节点
roslaunch mpc_local_planner test_mpc_optim_node.launch
```

### 2. 话题接口 (Topic Interface)

#### 订阅话题 (Subscribed Topics)
- `/trajectory_prediction_node/goal` (geometry_msgs/PoseStamped) - 目标点
- `/trajectory_prediction_node/start` (geometry_msgs/PoseStamped) - 起始点  
- `/trajectory_prediction_node/odom` (nav_msgs/Odometry) - 里程计信息
- `/trajectory_prediction_node/obstacles` (costmap_converter/ObstacleArrayMsg) - 障碍物
- `/trajectory_prediction_node/via_points` (nav_msgs/Path) - 通过点

#### 发布话题 (Published Topics)
- `/trajectory_prediction_node/local_plan` (nav_msgs/Path) - 预测轨迹
- `/trajectory_prediction_node/mpc_markers` (visualization_msgs/Marker) - 可视化标记
- `/trajectory_prediction_node/ocp_result` (mpc_local_planner_msgs/OptimalControlResult) - 优化结果

### 3. 参数配置 (Parameters)

主要参数配置在 `cfg/trajectory_prediction_params.yaml`:

```yaml
# 机器人模型
robot:
  type: "unicycle"
  unicycle:
    max_vel_x: 0.6        # 最大线速度
    max_vel_theta: 0.5    # 最大角速度

# 规划网格
grid:
  grid_size_ref: 25       # 网格大小
  dt_ref: 0.2            # 时间步长

# 碰撞避障
collision_avoidance:
  min_obstacle_dist: 0.3  # 最小障碍物距离

# 求解器
solver:
  type: "ipopt"
  ipopt:
    iterations: 50
    max_cpu_time: 0.1     # 实时性要求
```

### 4. 在RVIZ中可视化

1. 启动轨迹预测节点
2. 在RVIZ中设置初始位置（2D Pose Estimate）
3. 设置目标点（2D Nav Goal）
4. 观察预测轨迹和可视化效果

## 技术特点 (Technical Features)

### 1. 实时性能
- 优化的求解器参数，适合实时应用
- 最大CPU时间限制为100ms
- 支持热启动加速收敛

### 2. 多种机器人模型
- 独轮车模型 (Unicycle)
- 简单车辆模型 (Simple Car)  
- 运动学自行车模型 (Kinematic Bicycle)

### 3. 灵活的优化目标
- 最小时间轨迹
- 二次型代价函数
- 通过点约束
- 终端约束

### 4. 高级功能
- 动态障碍物避让
- 可变时间网格
- 轨迹平滑处理
- 多目标优化

## 依赖项 (Dependencies)

- ROS (Robot Operating System)
- control_box_rst - 控制和优化库
- Ipopt - 非线性优化求解器
- Eigen - 线性代数库
- teb_local_planner - TEB局部规划器（用于障碍物和足迹模型）

## 示例场景 (Example Scenarios)

### 1. 点到点轨迹预测
```bash
# 启动节点
roslaunch mpc_local_planner trajectory_prediction_node.launch

# 发布起始点
rostopic pub /trajectory_prediction_node/start geometry_msgs/PoseStamped "..."

# 发布目标点  
rostopic pub /trajectory_prediction_node/goal geometry_msgs/PoseStamped "..."
```

### 2. 带障碍物的轨迹规划
```bash
# 发布障碍物信息
rostopic pub /trajectory_prediction_node/obstacles costmap_converter/ObstacleArrayMsg "..."
```

### 3. 通过点轨迹优化
```bash
# 发布通过点
rostopic pub /trajectory_prediction_node/via_points nav_msgs/Path "..."
```

## 性能调优 (Performance Tuning)

### 1. 实时性优化
- 减少网格大小 (`grid_size_ref`)
- 降低求解器迭代次数 (`iterations`)
- 设置CPU时间限制 (`max_cpu_time`)

### 2. 轨迹质量优化  
- 增加网格密度
- 调整权重参数
- 启用可变网格 (`variable_grid.enable`)

### 3. 内存优化
- 限制轨迹长度
- 减少障碍物数量
- 优化数据结构

## 故障排除 (Troubleshooting)

### 常见问题
1. **轨迹预测失败**: 检查求解器参数和约束条件
2. **性能不佳**: 调整网格大小和求解器设置  
3. **可视化问题**: 确认RVIZ配置和话题连接

### 调试工具
- 启用CPU时间打印: `print_cpu_time: true`
- 发布优化结果: `publish_ocp_results: true`
- 使用Python绘图脚本分析轨迹

## 扩展开发 (Extension Development)

### 添加新的机器人模型
1. 继承 `RobotDynamicsInterface` 类
2. 实现动力学方程
3. 在配置文件中注册

### 自定义代价函数
1. 继承相应的代价函数基类
2. 实现梯度和Hessian计算
3. 在OCP中注册使用

### 新增约束条件
1. 继承约束基类
2. 实现约束函数和雅可比
3. 集成到优化问题中

---

## English Summary

This repository provides comprehensive trajectory prediction functionality using Model Predictive Control (MPC) with multiple ROS nodes:

- **trajectory_prediction_node**: Dedicated trajectory prediction service
- **test_mpc_optim_node**: Interactive MPC optimization demonstration  
- **mpc_local_planner_ros**: Full navigation stack integration

Key features include real-time trajectory prediction, multiple robot models, dynamic obstacle avoidance, and comprehensive visualization tools. The system is optimized for real-time performance with configurable parameters for different application requirements.