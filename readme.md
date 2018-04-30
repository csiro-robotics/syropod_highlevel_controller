# Syropod High-level Controller:

High level controller for CSIRO multi-legged robots (Syropods).

Current version: v0.5.10

Please use readme in launch folder of individual platform for information on setting up platform for use with Syropod High-level Controller.

For information on parameters see readme in config folder.


## Features
* Fully configurable for a variety of platform designs with differing physical characteristics, including up to eight legs each with up to 5 degrees of freedom.
* Four dynamically switchable gait options (Wave, Amble, Ripple and Tripod) plus ability to easily design custom gaits. (see config/readme.md)
* User defined body clearance, step clearance and step frequency.
* Manual body posing in 6 degrees of freedom.
* Manual leg manipulation - select up to two legs simultaneously and toggle manual manipulation of either tip position in Cartesian space or direct control of joint positions (3DOF legs only).
* Choice between two modes of start-up: 
    * Direct mode which move leg tip positions linearly from initial position to default walking stance positions.
    * Full chain of start-up/shutdown sequences which allow a Syropod to start from a 'packed' state and generate a sequence to stand up off the ground into its default walking stance. Similarly able to shut-down and transition back to a packed state.
* Optional impedance controller with dynamic leg stiffness to ensure leg contact with ground and offer a degree of rough terrain walking ability.
* Optional IMU body compensation which uses IMU data to keep body horizontally level at all times.
* Optional inclination compensation which strives to keep body centre of gravity over the estimated centre of support whilst walking on inclined planes.
* Optional bespoke automatic body posing system which poses each robot leg cyclically as defined by auto-pose parameters. (see config/readme.md)
* Cruise control mode which may either force a constant predefined input velocity for the Syropod or set the current input velocity as constant.
* Auto navigation mode which when combined with the correct sensing capabilities and running syropod_auto_navigation node, give autonomous navigation with obstacles avoidance to an input waypoint.


## Inputs:
### Syropod\_Remote:
* System State:
    * Description: The desired state of the entire Syropod High-level Controller system.
    * Topic: */syropod\_remote/system\_state*
    * Type: std_msgs::Int8
* Robot State:
    * Description: The desired state of the robot.
    * Topic: */syropod\_remote/robot_state*
    * Type: std_msgs::Int8
* Desired Velocity:
    * Description: The desired body velocity of the robot.
    * Topic: */syropod\_remote/desired\_velocity*
    * Type: geometry_msgs::Twist
* Desired Pose:
    * Description: The desired body pose of the robot.
    * Topic: */syropod\_remote/desired\_pose*
    * Type: geometry_msgs::Twist
* Posing Mode:
    * Description: The desired manual body posing input mode.
    * Topic: */syropod\_remote/posing\_mode*
    * Type: std_msgs::Int8
* Pose Reset Mode:
    * Description: The desired manual body pose reset mode.
    * Topic: */syropod\_remote/pose\_reset\_mode*
    * Type: std_msgs::Int8
* Gait Selection:
    * Description: The desired gait selection for the walk controller of the robot.
    * Topic: */syropod\_remote/gait\_selection*
    * Type: std_msgs::Int8
* Cruise Control Mode:
    * Description: The desired cruise control mode.
    * Topic: */syropod\_remote/cruise\_control\_mode*
    * Type: std_msgs::Int8
* Auto-Nagivation Mode:
    * Description: The desired auto-navigation mode.
    * Topic: */syropod\_remote/auto\_navigation\_mode*
    * Type: std_msgs::Int8
* Primary Leg Selection:
    * Description: The desired leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_leg\_selection*
    * Type: std_msgs::Int8
* Primary Leg State:
    * Description: The desired state of the leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_leg\_state*
    * Type: std_msgs::Int8
* Primary Tip Velocity:
    * Description: The desired tip velocity for the leg selected for primary manipulation.
    * Topic: */syropod\_remote/primary\_tip\_velocity*
    * Type: geometry_msgs::Point
* Secondary Leg Selection:
    * Description: The desired leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_leg\_selection*
    * Type: std_msgs::Int8
* Secondary Leg State:
    * Description: The desired state of the leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_leg\_state*
    * Type: std_msgs::Int8
* Secondary Tip Velocity:
    * Description: The desired tip velocity for the leg selected for secondary manipulation.
    * Topic: */syropod\_remote/secondary\_tip\_velocity*
    * Type: geometry_msgs::Point
* Parameter Selection:
    * Description: The desired parameter selection for possible adjustment.
    * Topic: */syropod\_remote/parameter\_selection*
    * Type: std_msgs::Int8
* Parameter Adjustment:
    * Description: The desired adjustment of the selected parameter (increment/decrement).
    * Topic: */syropod\_remote/parameter\_adjustment*
    * Type: std_msgs::Int8

### Motor and Sensor Inputs
* IMU Data:
    * Description: The input data from onboard IMU.
    * Topic: */imu/data*
    * Type: sensor_msgs::Imu
* Tip State Data:
    * Description: Custom message containing force/torque and range sensor data at the tip of each leg.
    * Topic: */tip\_states*
    * Type: syropod_highlevel_controller::TipState
* Joint State Data:
    * Description: The actual state of joints within the Syropod as published by hardware.
    * Topic: "*/joint\_states*
    * Type: sensor_msgs::JointState


## Outputs:
### Dynamixel Motor Interface:
* Desired Joint Position (per joint)
    * Description: Desired joint position for each individual joint.
    * Topic: */syropod/\*LEG_ID\*\_\*JOINT_ID\*\_joint/command*
    * Type: std_msgs::Float64
* Desired Joint State (combined)
    * Description: Desired joint state array for all joints.
    * Topic: */desired\_joint\_state*
    * Type: sensor_msgs::JointState

### Misc.
* Body Velocity:
    * Description: The desired velocity of the robot body.
    * Topic: */syropod_highlevel_controller/body\_velocity*
    * Type: geometry_msgs::Twist
* Body Pose:
    * Description: The desired pose of the robot body.
    * Topic: */syropod_highlevel_controller/pose*
    * Type: geometry_msgs::Twist
* ASC Hexapod State: (To be removed)
    * Description: ASC Hexapod specific message for toggling of magnetic feet (bool of whether each leg is in stance state or not)
    * Topic: */leg\_state\_\*LEG_ID\*\_bool*
    * Type: std_msgs::Bool
* Leg State:
    * Description: The leg state message combines several leg specific data for use in debugging.
      * header: Header with timestamp
      * name: Leg designation
      * walker_tip_pose: Desired tip pose generated from the walk controller (walk_plane frame)
      * target_tip_pose: Future desired tip pose at end of swing period (walk_plane frame)
      * poser_tip_pose: Desired tip pose generated from the pose controller (base_link frame)
      * model_tip_pose: Desired tip pose finalised by the model inverse/forward kinematics (base_link frame)
      * model_tip_velocity: Desired tip velocity finalised by the model inverse/forward kinematics (model).
      * joint_positions: Array of desired joint positions for each joint.
      * joint_velocities: Array of desired joint velocities for each joint.
      * joint_efforts: Array of desired joint efforts for each joint.
      * stance_progress: Progress along stance state (0.0->1.0 OR -1 if not in stance)
      * swing_progress: Progress along swing state (0.0->1.0 OR -1 if not in swing)
      * time_to_swing_end: Time until this leg completes swing period.
      * pose_delta: The estimated change in pose of the walk_plane frame with regard to world frame from current time to end of swing period.
      * auto_pose: The automatic cyclic posing applied to this leg from the auto-pose system.
      * tip_force: Tip force vector used in impedance control calculations
      * admittance_delta: Vertical tip position offset - output of admittance control.
      * virtual_stiffness: Current virtual stiffness used in admittance control calculations 
    * Topic: */shc/\*LEG_ID\*\_leg/state*
    * Type: syropod_highlevel_controller::LegState (custom message)


## Changelog:

Note: Version control commenced at v0.4.0. No changes were logged before this version.

- v0.4.0
    - MAX ICRA paper submission version
- v0.4.1
    - Fixed bug with body velocity calculation using minFootprintRadius as a diameter
    - Fixed bug with tip position jumping upon gait switch
- v0.4.2
    - Redesigned tip trajectory engine using 3 quartic bezier curves - tip trajectory is now C2 smooth at zero body acceleration
    - Removed redundant transition_period from step cycle and modified gait parameters/characteristics to reflect.
    - Added auto-calculation option for linear and angular accelerations of body to ensure safe tip trajectories within bounds
    - Added stance_depth parameter which allows vertical tip position component in stance phase of step cycle
    - Changed various parameter names in line with new engine
    - Added various debugging publishers
- v0.4.3
    - Modified Tripod gait to have no 'underlap' between step cycles
    - Modified the way auto/imu compensation interact with manual compensation. Manual compensation now 'stacks' with auto/imu compensation
    - Added various debugging publishers
- v0.4.4
    - Added angular velocity parameter for testing
    - Refactored walk-controller velocity calculations
    - Added 'real-world' velocity input mode toggled using 'velocity\_input\_mode' parameter
- v0.4.5
    - Added inclination compensation feature which keeps body centre of gravity centred about the tip positions on inclined terrain
    - Refactored how compensation methods combine and interact with each other
- v0.4.6
    - Added body height compensation code to offset sinking body height from impedance controller
    - Redesigned dynamic stiffness engine for use in impedance controller
- v0.4.7
    - Changed gaits to remove 'underlap' that existed for non-dynamic impedance controller
    - Moved imu compensation code (and all other compensation code) to pose controller - deleted imuCompensation source file and header
    - Redesigned auto-compensation to work for all gaits
- v0.4.8
    - Redesigned manual compensation to be controlled using velocity and reset on R3
    - Moved compensation and impedance control outside of running state so compensation occurs during startup
- v0.4.9
    - Added functionality for manual leg manipulation (MLM)
        - Posing to keep centre of gravity centred over centre of support whilst manipulating legs
        - Added dynamic stifness on legs adjacent to MLM leg
    - Added leg name mapping
    - Added leg state message to consolidate all leg debugging topics
- v0.5.0
    - Name change from simple_hexapod_controller to syropod_highlevel_controller, aka SHC.
    - Full refactor of controller.
    - Conversion to new coordinate system, adhering to ROS REP103
    - Overhaul of generation of robot model using DH parameters.
    - Overhaul of inverse/forward kinematics engine to handle up to 5 DOF legs.
    - New control scheme (see syropod_remote readme)
    - Redesigned start-ip/shut-down sequences.
    - New bespoke auto posing system (see config/readme.md)
    - Implementation of dynamic_reconfigure for key parameters.
    - Modifications to tip trajectory calculations to prevent exceeding workspace.
    - Modifications to tip trajectory calculations to maximise stride length and body acceleration.
    - Convertion of walk controller parameters from percentage based to real world values (SI units).
    - Changes to adhere to ROSCPP style guide.
- v0.5.1
    - New simpler Inverse Kinematics calculation method
    - Improved workspace calculation
- v0.5.2
    - New iterative search based workspace generation method
- v0.5.3
    - Improved Impedance Controller to make use of multi-joint effort values
    - Added tip force estimate visualisation
    - Added time limit to cruise control feature for use in running experiments.
- v0.5.4
    - Modified direct startup sequence to operate in joint space instead of tip space to prevent joints jumping if initialised outside limits.
    - Renamed impedance controller to 'admittance controller'
    - Removed custom Quat class and instead replaced with Eigen::Quaterniond
    - Removed IMU data transformation - SHC now expects IMU data in ROS REP103 coordinate frame
    - Removed joint offset parameters - SHC now expects all joint offsetting to be enacted at the motor driver level
- v0.5.5
    - *Online specific foot placement
- v0.5.6
    - Added feature adding cost function to joints approaching limits to ensure joint space loop closure in redundant systems.
    - Refactored joint/tip frame position functions and leg current/desired positions - to use pose (i.e. position & rotation)
    - Added tip orientation visualisations for debugging
- v0.5.7
    - Added Rough Terrain mode which includes the following features:
        - Feature to allow tip range value to modify tip trajectory at end of swing to prevent early/late touchdown.
        - Posing feature to pose robot body such that the final link of each leg completes swing periods vertically.
        - Reworked trajectory engine to allow for vertical tip trajectory at end of swing during wave gait
        - Feature to estimate walk plane and pose body accordingly.
        - Added TipState message and callback to receive tip force/torque and range
    - Consolidated tip force callbacks into single callback with custom message on topic "/tip_states"
    - Added visualisations for the estimated walk plane and terrain
- v0.5.8
    - Implemented Plan Execution Mode (Transition to target configuration, body/tip pose/s) (Free Gait/Movement)
    - Merged Auto Navigation mode and Cruise Control mode to free up Y Button for Plan Execution Mode
    - Added callback and msg for externally setting target tip poses at end of swing period (Adaptive Gait)
    - Added feature for multi-configuration-step pack/unpacking
- v0.5.9
    - Implemented real-time default tip position modification via topic
    - Implemented estimation of gravity vector from IMU 
    - Added transform publisher between frames: odom, walk_plane, base_link and tip frames
    - Added joint torque visualisations
    - Added touchdown detection feature using data on tip state topic and tuned using new parameters: touchdown_threshold & liftoff_threshold
    - Added feature to find ground contact by extending 2nd half of swing below walk plane (distance defined by parameter step_depth)
    - Added feature to align tips with gravity using redundancy of leg or body posing.
    - Refactored dynamic parameter tuning such that stopping Syropod is no longer required.
- v0.5.10
    - Added dynamic tuning of step frequency parameter
    - Added each leg workspace generation
    - Added correct generation of 'walkspace', fit within leg workspace
    - Fixed issues with externally set target/default tip poses
    - Refactored published tf tree order for use with perception based odometry
