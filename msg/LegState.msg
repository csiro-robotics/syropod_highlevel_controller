Header header
string name

geometry_msgs/Pose walker_tip_pose
geometry_msgs/Pose poser_tip_pose
geometry_msgs/Pose model_tip_pose

geometry_msgs/Twist model_tip_velocity

float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts

float64 stance_progress
float64 swing_progress

geometry_msgs/Pose auto_pose

float64 tip_force
float64 delta_z
float64 virtual_stiffness


