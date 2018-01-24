Header header
string name

geometry_msgs/PoseStamped walker_tip_pose
geometry_msgs/PoseStamped target_tip_pose
geometry_msgs/PoseStamped poser_tip_pose
geometry_msgs/PoseStamped model_tip_pose

geometry_msgs/TwistStamped model_tip_velocity

float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts

float64 stance_progress
float64 swing_progress

geometry_msgs/Pose auto_pose

geometry_msgs/Vector3 tip_force
geometry_msgs/Vector3 admittance_delta
float64 virtual_stiffness


