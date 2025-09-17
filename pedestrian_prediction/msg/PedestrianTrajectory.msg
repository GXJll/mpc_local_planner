# PedestrianTrajectory.msg
# Message for pedestrian trajectory prediction

Header header

# Pedestrian identification
int32 pedestrian_id
string pedestrian_label

# Current state
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist current_velocity

# Predicted trajectory
geometry_msgs/PoseStamped[] predicted_poses
float64[] prediction_times
float64[] confidence_scores

# Prediction metadata
float64 prediction_horizon
float64 time_step
string prediction_method
bool is_valid

# Additional information
float64 uncertainty
geometry_msgs/Polygon bounding_box