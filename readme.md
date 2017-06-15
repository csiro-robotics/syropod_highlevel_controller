# Syropod High-level Controller:

High level controller for CSIRO multi-legged robots (Syropods).

Current version: v0.5.0

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
* Auto navigation mode which when combined with the correct sensing capabilities and running hexapod_auto_navigation node, give autonomous navigation with obstacles avoidance to an input waypoint.

  
## Inputs:
### Hexapod\_Remote:
* Desired system state (*hexapod\_remote/system\_state*)
* Desired robot state (*hexapod\_remote/robot\_state*)
* Desired walking velocity (*hexapod\_remote/desired\_velocity*)
* Desired body pose (*hexapod\_remote/desired\_pose*)
* Posing mode (*hexapod\_remote/posing\_mode*)
* Pose reset mode (*hexapod\_remote/pose\_reset\_mode*)
* Gait selection (*hexapod\_remote/gait\_mode*)
* Cruise control mode (*hexapod\_remote/cruise\_control\_mode*)
* Auto-navigation mode (*hexapod\_remote/auto\_navigation\_mode*)
* Leg selection commands (*hexapod\_remote/primary\_leg\_selection*) (*hexapod\_remote/secondary\_leg\_selection*)
* Desired leg state commands (*hexapod\_remote/primary\_leg\_state*) (*hexapod\_remote/secondary\_leg\_state*)
* Desired leg tip/joint velocity commands (*hexapod\_remote/primary\_tip\_velocity*) (*hexapod\_remote/secondary\_tip\_velocity*)
* Parameter selection command (*hexapod\_remote/param\_selection*)
* Parameter adjustment command (*hexapod\_remote/param\_adjustment*)

### Motor and Sensor Inputs
* IMU data input (*/imu/data*)
* Tip force data (*/motor\_encoders*) (MAX) (To be removed)
* Individual tip force data (*/AR_prs*) (*/BR_prs*) (*/CR_prs*) (*/CL_prs*) (*/BL_prs*) (*/AL_prs*) (To be removed)
* Joint state data (*/joint\_states*)

## Outputs:
### Dynamixel Motor Interface:
* Desired joint position for each individual joint (*/hexapod/\*leg_name\*\_\*joint_name\*\_joint/command*)
* Desired joint state array for all joints (*/desired\_hexapod\_joint\_states*)

### Misc.
* Current desired body velocity (*/syropod_highlevel_controller/body\_velocity*)
* Current desired body pose (*/syropod_highlevel_controller/pose*)
* IMU data output (*/syropod_highlevel_controller/imu\_data*)
* ASC Hexapod specific message for toggling of magnetic feet (bool of whether each leg is in stance state or not)(*/leg\_state\_\*leg\*\_\*side\*\_bool*)(To be removed)
* Leg state (*/hexapod/\*leg\*\_\*side\*\_leg/state*):
    * header: Header with timestamp
    * leg_name: Leg designation
    * walker_tip_position: Desired tip position generated from the walk controller.
    * poser_tip_position: Desired tip position with applied posing (poser).
    * model_tip_position: Desired tip position finalised by the model inverse/forward kinematics (model).
    * model_tip_velocity: Desired tip velocity finalised by the model inverse/forward kinematics (model).
    * joint_positions: Array of desired joint positions for each joint.
    * joint_velocities: Array of desired joint velocities for each joint.
    * stance_progress: Progress along stance state (0.0->1.0 OR -1 if not in stance)
    * swing_progress: Progress along swing state (0.0->1.0 OR -1 if not in swing)
    * auto_pose: The automatic cyclic posing applied to this leg from the auto-pose system.
    * tip_force: Tip force used in impedance control calculations
    * delta_z: Vertical tip position offset - output of impedance control.
    * virtual_stiffness: Current virtual stiffness used in impedance control calculations 

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
    - Full refactor of controller.
    - Conversion to new coordinate system, adhering to ROS REP103
    - Overhaul of generation of robot model using DH parameters.
    - Overhaul of inverse/forward kinematics engine to handle up to 5 DOF legs.
    - New control scheme (see hexapod_remote readme)
    - Redesigned start-ip/shut-down sequences.
    - New bespoke auto posing system (see config/readme.md)
    - Implementation of dynamic_reconfigure for key parameters.
    - Modifications to tip trajectory calculations to prevent exceeding workspace.
    - Modifications to tip trajectory calculations to maximise stride length and body acceleration.
    - Convertion of walk controller parameters from percentage based to real world values (SI units).
    - Changes to adhere to ROSCPP style guide.



