# Simple Hexapod Controller:

High level controller for CSIRO hexapods (syropods).

Current version: v0.4.9

Please use readme in launch folder of individual platform for information on setting up platform for use with Simple Hexapod Controller.

For information on parameters see readme in config folder.

## Features
* Fully configurable for a variety of platform designs with differing physical characteristics.
* Four dynamically switchable gait options (Wave, Amble, Ripple and Tripod)
* User defined body clearance, step clearance and step frequency.
* Manual body posing in 6 degrees of freedom.
* Manual leg manipulation - select a leg and toggle manual manipulation of either tip position in Cartesian space or direct control of joint positions.
* Optional impedance controller with dynamic leg stiffness to ensure leg contact with ground and offer a degree of rough terrain walking ability
* Optional automatic body compensation whilst walking which adds pre-defined body pitch/tilt to compensate for sag of body into stepping legs.
* Optional IMU body compensation which uses IMU data to keep body horizontally level at all times.
* Optional inclination compensation which strives to keep body centre of gravity over the estimated centre of support whilst walking on inclined planes.
* Optional designable Startup and Shutdown sequences to allow platform to transition from required 'shutdown' or 'packed' form to walking form and back again.
  
## Inputs:
### Hexapod\_Remote:
* Desired walking velocity (*hexapod\_remote/desired\_velocity*)
* Desired body pose (*hexapod\_remote/desired\_pose*)
* Leg selection command (*hexapod\_remote/leg\_selection*)
* Leg state toggle command (*hexapod\_remote/leg\_state\_toggle*)
* Parameter selection command (*hexapod\_remote/param\_selection*)
* Parameter adjustment command (*hexapod\_remote/param\_adjust*)
* Toggle test command (*hexapod\_remote/test\_state\_toggle*)
* Pose reset mode (*hexapod\_remote/pose\_reset\_mode*)
* Start state (*hexapod\_remote/start\_state*)
* Gait selection (*hexapod\_remote/gait\_mode*)

### Motor and Sensor Inputs
* IMU data input (*ig/imu/data*)
* Tip force data (*/motor\_encoders*)
* Joint state data (*/hexapod\_joint\_state* OR */hexapod/joint\_states*)

## Outputs:
### Dynamixel Motor Interface:
* Desired joint position for body\_coxa joint (*/hexapod/\*leg\*\_\*side\*\_body_coxa/command*)
* Desired joint position for coxa\_femur joint (*/hexapod/\*leg\*\_\*side\*\_coxa_femour/command*)
* Desired joint position for femur\_tibia joint (*/hexapod/\*leg\*\_\*side\*\_femour_tibia/command*)
* Desired joint velocity for body\_coxa joint (*/hexapod/\*leg\*\_\*side\*\_body_coxa/velocity*)
* Desired joint velocity for coxa\_femur joint (*/hexapod/\*leg\*\_\*side\*\_coxa_femour/velocity*)
* Desired joint velocity for femur\_tibia joint (*/hexapod/\*leg\*\_\*side\*\_femour_tibia/velocity*)

### Dynamixel Pro Interface:
* Desired joint state array for all joints (*/desired\_hexapod\_joint\_state*)


### Misc.
* Current desired body velocity (*/hexapod/body\_velocity*)
* Current desired body pose (*/hexapod/pose*)
* IMU rotation ouput (*/hexapod/imu\_rotation*)
*
* ASC Hexapod specific message for toggling of magnetic feet (bool of whether each leg is in stance state or not)(*/leg\_state\_\*leg\*\_\*side\*\_bool*)(To be removed)
* Leg state (*/hexapod/\*leg\*\_\*side\*\_leg/state*):
 * Header with timestamp
 * Leg designation
 * Desired tip position along the walk cycle (walker)
 * Desired tip position with applied posing (walker+poser) 
 * Desired local tip position (walker+poser+impedance)
 * Progress along stance state (0.0->1.0 OR -1 if not in stance)
 * Progress along swing state (0.0->1.0 OR -1 if not in swing)
 * Tip force used in impedance control calculations
 * Output of impedance control (delta z)
 * Current virtual stiffness used in impedance control calculations 

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



