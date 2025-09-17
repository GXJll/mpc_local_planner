# MPC Local Planner - Trajectory Prediction

This document provides a comprehensive guide to the trajectory prediction capabilities of the MPC Local Planner for mobile robots.

## Overview

The MPC Local Planner implements Model Predictive Control (MPC) for mobile robot navigation. It provides robust trajectory prediction capabilities using various robot dynamics models and optimal control techniques.

## Key Components

### 1. Controller Class (`controller.h/cpp`)

The main controller class that implements trajectory prediction through MPC optimization:

- **Primary Functions:**
  - `step()`: Main trajectory computation function
  - `generateInitialStateTrajectory()`: Generates initial trajectory from a plan
  - `isPoseTrajectoryFeasible()`: Validates trajectory feasibility

### 2. Robot Dynamics Models

The planner supports multiple robot dynamics models for trajectory prediction:

#### Unicycle Model (`systems/unicycle_robot.h`)
- **State**: [x, y, θ] (position and orientation)
- **Control**: [v, ω] (linear and angular velocity)
- **Dynamics**: 
  ```
  ẋ = v * cos(θ)
  ẏ = v * sin(θ)  
  θ̇ = ω
  ```
- **Use Case**: Differential drive robots

#### Simple Car Model (`systems/simple_car.h`)
- **State**: [x, y, θ] with wheelbase constraint
- **Control**: [v, δ] (velocity and steering angle)
- **Use Case**: Car-like robots with front/rear wheel steering

#### Kinematic Bicycle Model (`systems/kinematic_bicycle_model.h`)
- **State**: [x, y, θ] with front and rear axle lengths
- **Control**: [v, δ] (velocity and steering angle)
- **Use Case**: More accurate car-like robot modeling

### 3. Trajectory Discretization

#### Finite Differences Grid (`optimal_control/finite_differences_grid_se2.h`)
- Fixed time step discretization
- Simple and efficient for basic trajectory prediction

#### Variable Grid (`optimal_control/finite_differences_variable_grid_se2.h`)
- Adaptive time step discretization
- Grid adaptation strategies:
  - **Time-based single step**: Adjusts grid based on time step requirements
  - **Time-based aggressive**: More aggressive grid adaptation for efficiency
  - **Simple shrinking horizon**: Reduces horizon for computational efficiency

## Trajectory Prediction Workflow

### 1. Initialization
```cpp
// Configure robot dynamics
RobotDynamicsInterface::Ptr dynamics = configureRobotDynamics(nh);

// Configure discretization grid
DiscretizationGridInterface::Ptr grid = configureGrid(nh);

// Configure optimal control problem
StructuredOptimalControlProblem::Ptr ocp = configureOcp(nh, obstacles, robot_model, via_points);
```

### 2. Trajectory Generation
```cpp
bool Controller::step(const std::vector<geometry_msgs::PoseStamped>& initial_plan, 
                      const geometry_msgs::Twist& vel, double dt, ros::Time t,
                      corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
{
    // 1. Extract start and goal poses
    PoseSE2 start(initial_plan.front().pose);
    PoseSE2 goal(initial_plan.back().pose);
    
    // 2. Get current state (from feedback or estimation)
    Eigen::VectorXd x(dynamics->getStateDimension());
    // ... state estimation logic
    
    // 3. Generate initial trajectory if needed
    if (grid->isEmpty()) {
        generateInitialStateTrajectory(x, xf, initial_plan, backward);
    }
    
    // 4. Solve optimal control problem
    bool success = PredictiveController::step(x, xref, uref, dt, time, u_seq, x_seq);
    
    return success;
}
```

### 3. Initial Trajectory Generation
```cpp
bool Controller::generateInitialStateTrajectory(const Eigen::VectorXd& x0, 
                                                const Eigen::VectorXd& xf,
                                                const std::vector<geometry_msgs::PoseStamped>& initial_plan, 
                                                bool backward)
{
    // 1. Create time series for trajectory
    TimeSeriesSE2::Ptr ts = std::make_shared<TimeSeriesSE2>();
    
    // 2. Add initial state
    ts->add(0.0, x0);
    
    // 3. Interpolate intermediate states from plan
    for (int i = 1; i < n_init - 1; ++i) {
        // Estimate orientation from path direction
        double yaw = atan2(dy, dx);
        if (backward) yaw += M_PI;
        
        // Convert pose to state
        PoseSE2 pose(x, y, yaw);
        dynamics->getSteadyStateFromPoseSE2(pose, x);
        ts->add(t, x);
    }
    
    // 4. Add final state
    ts->add(tf_ref, xf);
    
    // 5. Set as initial trajectory
    x_seq_init.setTrajectory(ts, TimeSeries::Interpolation::Linear);
    return true;
}
```

### 4. Trajectory Feasibility Checking
```cpp
bool Controller::isPoseTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model,
                                          const std::vector<geometry_msgs::Point>& footprint_spec,
                                          double inscribed_radius,
                                          double circumscribed_radius,
                                          double min_resolution_collision_check_angular,
                                          int look_ahead_idx)
{
    // 1. Check collision at discrete points
    for (int i = 0; i <= look_ahead_idx; ++i) {
        if (costmap_model->footprintCost(...) == -1) {
            return false; // Collision detected
        }
        
        // 2. Check intermediate points if needed
        if (distance_too_large || angle_diff_too_large) {
            // Add intermediate collision checks
            // ...
        }
    }
    return true;
}
```

## Configuration Parameters

### Robot Dynamics Configuration
```yaml
# Robot type selection
robot:
  type: "unicycle"  # Options: "unicycle", "simple_car", "kinematic_bicycle_vel_input"
  
  # Simple car parameters
  simple_car:
    wheelbase: 0.5
    front_wheel_driving: false
    
  # Kinematic bicycle parameters  
  kinematic_bicycle_vel_input:
    length_rear: 1.0
    length_front: 1.0
```

### Grid Configuration
```yaml
# Grid discretization
grid:
  type: "fd_grid"  # or "variable_fd_grid"
  n_min: 10        # Minimum grid points
  n_max: 100       # Maximum grid points
  dt_ref: 0.1      # Reference time step
  dt_hysteresis_ratio: 0.1
```

### Controller Configuration
```yaml
controller:
  outer_ocp_iterations: 1
  force_reinit_new_goal_dist: 1.0
  force_reinit_new_goal_angular: 1.57
  allow_init_with_backward_motion: true
  prefer_x_feedback: false
  publish_ocp_results: true
```

## Advanced Features

### 1. Adaptive Grid Management
The variable grid implementation automatically adjusts the prediction horizon and time discretization based on:
- Solution quality requirements
- Computational constraints  
- Trajectory complexity

### 2. Warm Starting
The controller maintains trajectory continuity by:
- Reusing previous solutions as initial guesses
- Handling goal changes gracefully
- Managing trajectory reinitializiation

### 3. State Feedback Integration
The controller can incorporate external state feedback:
- Sensor-based state estimation
- Odometry integration
- Custom state feedback via ROS topics

### 4. Multi-Robot Support
The architecture supports different robot types through the dynamics interface:
- Easy extension to new robot models
- Consistent interface across all models
- Automatic parameter configuration

## Usage Examples

### Basic Trajectory Prediction
```cpp
// Initialize controller
Controller controller;
controller.configure(nh, obstacles, robot_model, via_points);

// Predict trajectory
std::vector<geometry_msgs::PoseStamped> plan = global_planner.makePlan();
corbo::TimeSeries::Ptr u_seq, x_seq;
bool success = controller.step(plan, current_vel, dt, ros::Time::now(), u_seq, x_seq);

if (success) {
    // Use predicted trajectory
    geometry_msgs::Twist cmd_vel = extractControlCommand(u_seq);
    // ...
}
```

### Trajectory Feasibility Checking
```cpp
// Check if predicted trajectory is collision-free
bool feasible = controller.isPoseTrajectoryFeasible(costmap_model, 
                                                   robot_footprint,
                                                   inscribed_radius,
                                                   circumscribed_radius,
                                                   0.1,  // angular resolution
                                                   -1);  // check full trajectory
if (!feasible) {
    // Replan or take evasive action
    controller.reset();
}
```

## Performance Considerations

### Computational Efficiency
- Use appropriate grid resolution (balance accuracy vs. speed)
- Enable grid adaptation for varying computational loads
- Consider warm starting for trajectory continuity

### Real-time Requirements
- Typical computation times: 10-50ms per cycle
- Grid adaptation can help maintain real-time performance
- Solver selection impacts computation time

### Memory Usage
- Trajectory storage scales with prediction horizon
- Grid adaptation manages memory dynamically
- Consider trajectory compression for long horizons

## Integration with ROS Navigation Stack

The MPC Local Planner integrates seamlessly with the ROS navigation stack:

1. **Global Planner Integration**: Uses global plans as initial trajectory guesses
2. **Costmap Integration**: Incorporates dynamic obstacles and map constraints
3. **Localization Integration**: Uses odometry and sensor feedback for state estimation
4. **Velocity Commands**: Outputs standard geometry_msgs::Twist commands

## Troubleshooting

### Common Issues

1. **Trajectory Infeasibility**
   - Check obstacle constraints
   - Verify robot dynamics parameters
   - Adjust prediction horizon

2. **Poor Performance**
   - Optimize grid resolution
   - Enable grid adaptation
   - Check solver configuration

3. **Oscillatory Behavior**
   - Adjust cost function weights
   - Check trajectory initialization
   - Verify state feedback configuration

### Debug Output
Enable debug information:
```yaml
controller:
  publish_ocp_results: true
  print_cpu_time: true
```

## References

1. Rösmann, C., Makarow, A., & Bertram, T. (2020). Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups. arXiv preprint arXiv:2006.03534.

2. Rösmann, C. (2019). Time-optimal nonlinear model predictive control, Direct transcription methods with variable discretization and structural sparsity exploitation. Dissertation, Technische Universität Dortmund.

3. ROS Navigation Stack: http://wiki.ros.org/navigation