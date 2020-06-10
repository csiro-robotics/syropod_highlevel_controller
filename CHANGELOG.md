# Changelog

* v0.5.11
  * Added more Bezier curve functionalities:
    * Quadratic Bezier and curve to pass through control nodes.
    * Cubic Bezier to pass through control nodes.
    * Quartic Bezier to pass through control nodes.
  * Added feature to move the leg to desired position based on direct Cartesian coordinates in the robot frame.
  * Added topics that subscribes to topics published by Syropod Manipulation package.
* v0.5.10
  * Added dynamic tuning of step frequency parameter
  * Added each leg workspace generation
  * Added correct generation of 'walkspace', fit within leg workspace
  * Fixed issues with externally set target/default tip poses
  * Refactored published tf tree order for use with perception based odometry
* v0.5.9
  * Implemented real-time default tip position modification via topic
  * Implemented estimation of gravity vector from IMU
  * Added transform publisher between frames: odom, walk_plane, base_link and tip frames
  * Added joint torque visualisations
  * Added touchdown detection feature using data on tip state topic and tuned using new parameters: touchdown_threshold & liftoff_threshold
  * Added feature to find ground contact by extending 2nd half of swing below walk plane (distance defined by parameter step_depth)
  * Added feature to align tips with gravity using redundancy of leg or body posing.
  * Refactored dynamic parameter tuning such that stopping Syropod is no longer required.
* v0.5.8
  * Implemented Plan Execution Mode (Transition to target configuration, body/tip pose/s) (Free Gait/Movement)
  * Merged Auto Navigation mode and Cruise Control mode to free up Y Button for Plan Execution Mode
  * Added callback and msg for externally setting target tip poses at end of swing period (Adaptive Gait)
  * Added feature for multi-configuration-step pack/unpacking
* v0.5.7
  * Added Rough Terrain mode which includes the following features:
    * Feature to allow tip range value to modify tip trajectory at end of swing to prevent early/late touchdown.
    * Posing feature to pose robot body such that the final link of each leg completes swing periods vertically.
    * Reworked trajectory engine to allow for vertical tip trajectory at end of swing during wave gait
    * Feature to estimate walk plane and pose body accordingly.
    * Added TipState message and callback to receive tip force/torque and range
  * Consolidated tip force callbacks into single callback with custom message on topic "/tip_states"
  * Added visualisations for the estimated walk plane and terrain
* v0.5.6
  * Added feature adding cost function to joints approaching limits to ensure joint space loop closure in redundant systems.
  * Refactored joint/tip frame position functions and leg current/desired positions - to use pose (i.e. position & rotation)
  * Added tip orientation visualisations for debugging
* v0.5.5
  * *Online specific foot placement
* v0.5.4
  * Modified direct startup sequence to operate in joint space instead of tip space to prevent joints jumping if initialised outside limits.
  * Renamed impedance controller to 'admittance controller'
  * Removed custom Quat class and instead replaced with Eigen::Quaterniond
  * Removed IMU data transformation - SHC now expects IMU data in ROS REP103 coordinate frame
  * Removed joint offset parameters - SHC now expects all joint offsetting to be enacted at the motor driver level
* v0.5.3
  * Improved Impedance Controller to make use of multi-joint effort values
  * Added tip force estimate visualisation
  * Added time limit to cruise control feature for use in running experiments.
* v0.5.2
  * New iterative search based workspace generation method
* v0.5.1
  * New simpler Inverse Kinematics calculation method
  * Improved workspace calculation
* v0.5.0
  * Name change from simple_hexapod_controller to syropod_highlevel_controller, aka SHC.
  * Full refactor of controller.
  * Conversion to new coordinate system, adhering to ROS REP103
  * Overhaul of generation of robot model using DH parameters.
  * Overhaul of inverse/forward kinematics engine to handle up to 5 DOF legs.
  * New control scheme (see syropod_remote readme)
  * Redesigned start-ip/shut-down sequences.
  * New bespoke auto posing system (see config/readme.md)
  * Implementation of dynamic_reconfigure for key parameters.
  * Modifications to tip trajectory calculations to prevent exceeding workspace.
  * Modifications to tip trajectory calculations to maximise stride length and body acceleration.
  * Conversion of walk controller parameters from percentage based to real world values (SI units).
  * Changes to adhere to ROSCPP style guide.
* v0.4.9
  * Added functionality for manual leg manipulation (MLM)
    * Posing to keep centre of gravity centred over centre of support whilst manipulating legs
    * Added dynamic stiffness on legs adjacent to MLM leg
  * Added leg name mapping
  * Added leg state message to consolidate all leg debugging topics
* v0.4.8
  * Redesigned manual compensation to be controlled using velocity and reset on R3
  * Moved compensation and impedance control outside of running state so compensation occurs during startup
* v0.4.7
  * Changed gaits to remove 'underlap' that existed for non*dynamic impedance controller
  * Moved imu compensation code (and all other compensation code) to pose controller - deleted imuCompensation source file and header
  * Redesigned auto-compensation to work for all gaits
* v0.4.6
  * Added body height compensation code to offset sinking body height from impedance controller
  * Redesigned dynamic stiffness engine for use in impedance controller
* v0.4.5
  * Added inclination compensation feature which keeps body centre of gravity centred about the tip positions on inclined terrain
  * Refactored how compensation methods combine and interact with each other
* v0.4.4
  * Added angular velocity parameter for testing
  * Refactored walk-controller velocity calculations
  * Added 'real-world' velocity input mode toggled using 'velocity\_input\_mode' parameter
* v0.4.3
  * Modified Tripod gait to have no 'underlap' between step cycles
  * Modified the way auto/imu compensation interact with manual compensation. Manual compensation now 'stacks' with auto/imu compensation
  * Added various debugging publishers
* v0.4.2
  * Redesigned tip trajectory engine using 3 quartic bezier curves - tip trajectory is now C2 smooth at zero body acceleration
  * Removed redundant transition_period from step cycle and modified gait parameters/characteristics to reflect.
  * Added auto-calculation option for linear and angular accelerations of body to ensure safe tip trajectories within bounds
  * Added stance_depth parameter which allows vertical tip position component in stance phase of step cycle
  * Changed various parameter names in line with new engine
  * Added various debugging publishers
* v0.4.1
  * Fixed bug with body velocity calculation using minFootprintRadius as a diameter
  * Fixed bug with tip position jumping upon gait switch
* v0.4.0
  * MAX ICRA paper submission version

Note: Version control commenced at v0.4.0. No changes were logged before this version.
