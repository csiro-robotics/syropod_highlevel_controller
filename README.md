# Syropod High-level Controller

[![Syropod Banner](https://i.imgur.com/QyMTwG3.jpg "CSIRO Robotics")](https://research.csiro.au/robotics/)

[![Version](https://img.shields.io/badge/Current%20version-0.5.11-orange "Version")]() [![License](
https://img.shields.io/badge/License-BSD%2FMIT-blue "License")](https://github.com/csiro-robotics/syropod_highlevel_controller/blob/feature/update_readme/LICENSE)

Syropod High-level Controller (SHC) is a versatile controller capable of generating body poses and gaits for quasi-static multilegged robots. This ROS package implemented in C++ can be easily deployed on legged robots with different sensor, leg and joint configurations. SHC is designed to generate foot tip trajectories for a given gait sequence, step clearance, step frequency and input body velocity. Input sensors such as IMU and joint effort feedback can be utilised by the controller to provide robust trajectories even in inclined and uneven terrain. SHC was developed at CSIRO’s Robotics and Autonomous Systems Group to support its ongoing legged robot locomotion research. It is now being released as an open-sourced package, along with tutorials, for the benefit of the wider community as OpenSHC. The banner image and the video clip below shows a number of CSIRO's legged robots running SHC.

<p align="center">
<a href="https://research.csiro.au/robotics/our-work/research-areas/legged-robots/"><img alt="Gizmo Wizmo Zero" align="center" width="500" src="https://i.imgur.com/HCrmRDS.gif"/></a>
</p>

## Getting Started

If you haven't looked at the tutorials for using OpenSHC, see [OpenSHC Tutorials](https://github.com/csiro-robotics/shc_tutorials).

Please refer to the readme in the launch folder of individual platforms for information on setting up the platform for use with OpenSHC.

Video overview of [OpenSHC](https://youtu.be/-E7-2UMP5XU):

<p align="center">
<a href="https://youtu.be/-E7-2UMP5XU"><img alt="OpenSHC" align="center" width="500" src="https://imgur.com/VBabLVr.jpg"/></a>
</p>

### Requirements

* Ubuntu 18.04 LTS
* ROS Melodic

### Dependencies

* OpenSHC requires a robot specific configuration and launch repository such as [Bullet Syropod](https://github.com/csiro-robotics/bullet_syropod) to run. Please refer to that readme for more information.

### Installation

```bash
mkdir -p openshc_ws/src
cd openshc_ws/src
git clone https://github.com/csiro-robotics/syropod_highlevel_controller.git
cd ..
catkin build
```

### Publications

The details of OpenSHC is published in the following article:

*Benjamin Tam, Fletcher Talbot, Ryan Steindl, Alberto Elfes, and Navinda Kottege, "OpenSHC: A Versatile Multilegged Robot Controller", [arXiv:2006.04424](https://arxiv.org/abs/2006.04424) [cs.RO], June 2020*

#### How to cite

```bibtex
@article{tam2020openshc,
    title={OpenSHC: A Versatile Multilegged Robot Controller},
    author={Benjamin Tam and Fletcher Talbot and Ryan Steindl and Alberto Elfes and Navinda Kottege},
    year={2020},
    eprint={2006.04424},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

## Features

<p align="center">
<a href="https://research.csiro.au/robotics/our-work/research-areas/legged-robots/"><img alt="Bullet Syropod" align="center" width="500" src="https://i.imgur.com/cHAZ10Y.gif"/></a>
</p>

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

## Config Files

For information on parameters see [readme.md](config/readme.md) in config folder.

## Nodes

### syropod_highlevel_control

#### Subscribed Topics

##### [syropod_remote](https://github.com/csiro-robotics/syropod_remote)

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

##### syropod_manipulation

* Primary Leg Tip Pose:
  * Description: The desired pose for the leg selected for primary manipulation within Cartesian space.
  * Topic: */syropod_manipulation/primary_tip_pose*
  * Type:  geometry_msgs::Pose

* Secondary Leg Tip Pose:
  * Description: The desired pose for the leg selected for secondary manipulation within Cartesian space.
  * Topic: */syropod_manipulation/secondary_tip_pose*
  * Type:  geometry_msgs::Pose

##### Motor and Sensor Inputs

* IMU Data:
  * Description: The input data from onboard IMU.
  * Topic: */imu/data*
  * Type: sensor_msgs::Imu
* Tip State Data:
  * Description: Custom message containing force/torque and range sensor data at the tip of each leg.
  * Topic: */tip\_states*
  * Type: syropod_high-level_controller::TipState
* Joint State Data:
  * Description: The actual state of joints within the Syropod as published by hardware.
  * Topic: "*/joint\_states*
  * Type: sensor_msgs::JointState

#### Published Topics

##### Dynamixel Motor Interface

* Desired Joint Position (per joint)
  * Description: Desired joint position for each individual joint.
  * Topic: */syropod/\*LEG_ID\*\_\*JOINT_ID\*\_joint/command*
  * Type: std_msgs::Float64
* Desired Joint State (combined)
  * Description: Desired joint state array for all joints.
  * Topic: */desired\_joint\_state*
  * Type: sensor_msgs::JointState

##### Miscellaneous

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

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for release details.

## Authors

* Fletcher Talbot
* Ryan Steindl
* Thomas Molnar
* Thomas Lowe
* Oshada Jayasinghe
* Navinda Kottege

## License

This project is licensed under the CSIRO Open Source Software Licence Agreement (variation of the BSD / MIT License) - see the [LICENSE](LICENSE) file for details.

## Issues

Please report bugs using [Issue Tracker](https://github.com/csiro-robotics/syropod_highlevel_controller/issues) or contact us via email [shc-support@csiro.au](mailto:shc-support@csiro.au).
