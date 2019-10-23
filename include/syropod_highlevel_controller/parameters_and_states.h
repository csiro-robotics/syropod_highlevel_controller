//*********************************************************************************************************************
// * CSIRO Autonomous Systems Laboratory
// * Queensland Centre for Advanced Technologies
// * PO Box 883, Kenmore, QLD 4069, Australia
// *
// * (c) Copyright CSIRO 2017
// *
// * All rights reserved, no part of this program may be used
// * without explicit permission of CSIRO
//*********************************************************************************************************************

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_PARAMETERS_AND_STATES_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_PARAMETERS_AND_STATES_H

#include "standard_includes.h"

/**
 * Designation for potential states of the entire top-level controller system.
 */
enum SystemState
{
  SUSPENDED,          ///< Controller system is temporarily suspended, waiting for user input.
  OPERATIONAL,        ///< Controller system is operational and running.
  SYSTEM_STATE_COUNT, ///< Misc enum defining number of System States
};

/**
 * Designation for potential states of the robot
 */
enum RobotState
{
  PACKED,             ///< The robot is in a 'packed' state with all joints at defined 'packed' positions.
  READY,              ///< The robot is in a 'ready' state with all joints at defined 'unpacked' positions.
  RUNNING,            ///< The robot is in a 'running' state. This state is where all posing and walking occurs.
  ROBOT_STATE_COUNT,  ///< Misc enum defining number of Robot States
  UNKNOWN = -1,       ///< The robot is in an initial 'unknown' state from which the controller will estimate an actual state.
  OFF = -2,           ///< The robot is in 'off' state. Only used as alternative to 'running' state for direct start up.
};

/**
 * Designation for potential gait types.
 */
enum GaitDesignation
{
  WAVE_GAIT,              ///< Wave gait designation - as defined in config/gait.yaml
  AMBLE_GAIT,             ///< Amble gait designation - as defined in config/gait.yaml
  RIPPLE_GAIT,            ///< Ripple gait designation - as defined in config/gait.yaml
  TRIPOD_GAIT,            ///< Tripod gait designation - as defined in config/gait.yaml
  GAIT_DESIGNATION_COUNT, ///< Misc enum defining number of Gait Designations
  GAIT_UNDESIGNATED = -1, ///< Undesignated gait.
};

/**
 * Designation for potential manual body posing input modes.
 */
enum PosingMode
{
  NO_POSING,         ///< Mode allowing no manual body posing.
  X_Y_POSING,        ///< Mode allowing for manual posing of the robot body via x/y axis translation.
  PITCH_ROLL_POSING, ///< Mode allowing for manual posing of the robot body via pitch/roll rotation.
  Z_YAW_POSING,      ///< Mode allowing for manual posing of the robot body via z axis translation and yaw rotation.
  EXTERNAL_POSING,   ///< Mode allowing for posing input from external source.
  POSING_MODE_COUNT, ///< Misc enum defining number of Posing Modes
};

/**
 * Designation for potential cruise control modes.
 */
enum CruiseControlMode
{
  CRUISE_CONTROL_OFF,           ///< Cruise control mode is off
  CRUISE_CONTROL_ON,            ///< Cruise control mode is on
  CRUISE_CONTROL_MODE_COUNT,    ///< Misc enum defining number of Cruise Control Modes
  CRUISE_CONTROL_EXTERNAL = -1, ///< Cruise control mode is external
};

/**
 * Designation for potential planner modes.
 */
enum PlannerMode
{
  PLANNER_MODE_OFF,   ///< Planner mode is off
  PLANNER_MODE_ON,    ///< Planner mode is on
  PLANNER_MODE_COUNT, ///< Misc enum defining number of Planner Modes
};

/**
 * Designation for potential leg states.
 */
enum LegState
{
  WALKING,           ///< The leg is in a 'walking' state - participates in walking cycle.
  MANUAL,            ///< The leg is in a 'manual' state - able to move via manual manipulation inputs.
  LEG_STATE_COUNT,   ///< Misc enum defining number of LegStates  
  WALKING_TO_MANUAL = -1, ///< The leg is in 'walking to manual' state - transitioning from 'walking' to 'manual' state.
  MANUAL_TO_WALKING = -2, ///< The leg is in 'manual to walking' state - transitioning from 'manual' to 'walking' state.
};

/**
 * Designation for potential walk controller walk cycle states.
 */
enum WalkState
{
  STARTING,         ///< The walk controller cycle is in 'starting' state (transitioning from 'stopped' to 'moving').
  MOVING,           ///< The walk controller cycle is in a 'moving' state (the primary walking state).
  STOPPING,         ///< The walk controller cycle is in a 'stopping' state (transitioning from 'moving' to 'stopped').
  STOPPED,          ///< The walk controller cycle is in a 'stopped' state (state whilst velocity input is zero).
  WALK_STATE_COUNT, ///< Misc enum defining number of Walk States
};

/**
 * Designation for potential individual leg step cycle states.
 */
enum StepState
{
  SWING,            ///< The leg step cycle is in a 'swing' state - the forward 'in air' progression of the step cycle.
  STANCE,           ///< The leg step cycle is in a 'stance' state - the backward 'on ground' regression of the step cycle.
  FORCE_STANCE,     ///< State used to force a 'stance' state in non-standard instances.
  FORCE_STOP,       ///< State used to force the step cycle to stop iterating.
  STEP_STATE_COUNT, ///< Misc enum defining number of Step States
};

/**
 * Designation for potential posing states used in auto-posing.
 */
enum PosingState
{
  POSING,             ///< State signifiying that auto-poser objects should start their posing cycle.
  STOP_POSING,        ///< State signifiying that auto-poser objects should end their posing cycle.
  POSING_COMPLETE,    ///< State signifiying that ALL auto-poser objects have completed their individual posing cycles.
  POSING_STATE_COUNT, ///< Misc enum defining number of Posing States
};

/**
 * Designation for potential manual pose reset input modes.
 */
enum PoseResetMode
{
  NO_RESET,              ///< State requesting no reseting of manual body posing is requested.
  Z_AND_YAW_RESET,       ///< State requesting manual body z-axis translation or yaw rotation posing to reset to zero.
  X_AND_Y_RESET,         ///< State requesting manual body x-axis or y-axis translation posing to reset to zero.
  PITCH_AND_ROLL_RESET,  ///< State requesting manual body roll or pitch rotation posing to reset to zero.
  ALL_RESET,             ///< State requesting all manual body posing (in any axis) to reset to zero.
  IMMEDIATE_ALL_RESET,   ///< State forcing all manual body posing (in any axis) to reset to zero instantaneously.
  POSE_RESET_MODE_COUNT, ///< Misc enum defining number of Pose-Reset States
};

/**
 * Designations for potential legs within the robot model - up to 8 legs maximum.
 */
enum LegDesignation
{
  LEG_0,                 ///< 1st leg - The front right most leg of the robot.
  LEG_1,                 ///< 2nd leg - The leg following the 1st leg in a clockwise direction around the robot body.
  LEG_2,                 ///< 3rd leg - The leg following the 2nd leg in a clockwise direction around the robot body.
  LEG_3,                 ///< 4th leg - The leg following the 3rd leg in a clockwise direction around the robot body.
  LEG_4,                 ///< 5th leg - The leg following the 4th leg in a clockwise direction around the robot body.
  LEG_5,                 ///< 6th leg - The leg following the 5th leg in a clockwise direction around the robot body.
  LEG_6,                 ///< 7th leg - The leg following the 6th leg in a clockwise direction around the robot body
  LEG_7,                 ///< 8th leg - The leg following the 7th leg in a clockwise direction around the robot body.
  LEG_DESIGNATION_COUNT, ///< Misc enum defining number of Leg Designations
  LEG_UNDESIGNATED = -1, ///< A leg without any specific designation.
};

/**
 * Designation for potential adjustable parameter selections.
 */
enum ParameterSelection
{
  NO_PARAMETER_SELECTION,    ///< Designation for no parameter.
  STEP_FREQUENCY,            ///< Designation for the adjustable parameter 'step_frequency'.
  SWING_HEIGHT,              ///< Designation for the adjustable parameter 'swing_height'.
  SWING_WIDTH,               ///< Designation for the adjustable parameter 'swing_width'
  STEP_DEPTH,                ///< Designation for the adjustable parameter 'step_depth'.
  STANCE_SPAN_MODIFIER,      ///< Designation for the adjustable parameter 'stance_span_modifier'.
  VIRTUAL_MASS,              ///< Designation for the adjustable parameter 'virtual_mass'.
  VIRTUAL_STIFFNESS,         ///< Designation for the adjustable parameter 'virtual_stiffness'.
  VIRTUAL_DAMPING,           ///< Designation for the adjustable parameter 'virtual_damping'.
  FORCE_GAIN,                ///< Designation for the adjustable parameter 'force_gain'.
  PARAMETER_SELECTION_COUNT, ///< Misc enum defining number of Parameter Selections
};

/**
 * Designation for potential posing/stepping sequences able to be executed by the robot.
 */
enum SequenceSelection
{
  START_UP,                 ///< Start up sequence occurs when robot attempts to move from ground to a walking stance.
  SHUT_DOWN,                ///< Shut down sequence occurs when robot attempts to move from walking stance to ground.
  SEQUENCE_SELECTION_COUNT, ///< Misc enum defining number of System States
};

/*******************************************************************************************************************//**
 * This structure contains the data associated with a parameter acquired from the ros parameter server via a self
 * initialisation function.
***********************************************************************************************************************/
template <typename T>
struct Parameter
{
public:
  /**
   * Initialisation function which self populates parameter data from ros parameter server.
   * @param[in] name_input The unique name of the parameter to look for on ros parameter server
   * @param[in] base_parameter_name The base parameter name prepended to 'name_input' common to all parameters
   * @param[in] required_input Bool denoting if this parameter is required to be initialised.
   */
  inline void init(const string& name_input,
                   const string& base_parameter_name = "/syropod/parameters/",
                   const bool& required_input = true)
  {
    ros::NodeHandle n;
    name = name_input;
    required = required_input;
    initialised = n.getParam(base_parameter_name + name_input, data);
    ROS_ERROR_COND(!initialised && required_input, "Error reading parameter/s %s from rosparam."
                   " Check config file is loaded and type is correct\n", name.c_str());
  }

  string name;              ///< Name of the parameter
  T data;                   ///< Data which defines parameter
  bool required = true;     ///< Denotes if this parameter is required to be initialised
  bool initialised = false; ///< Denotes if this parameter has been initialised

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This structure contains the data associated with a dynamically adjustable parameter acquired from the ros parameter
 * server via a self initialisation function.
***********************************************************************************************************************/
struct AdjustableParameter : public Parameter<map<string, double>>
{
public:
  /**
    * Initialisation function which self populates parameter data from ros parameter server.
    * @param[in] name_input The unique name of the parameter to look for on ros parameter server
    * @param[in] base_parameter_name The base parameter name prepended to 'name_input' common to all parameters
    * @param[in] required_input Bool denoting if this parameter is required to be initialised.
    */
  inline void init(const string& name_input,
                   const string& base_parameter_name = "/syropod/parameters/",
                   const bool& required_input = true)
  {
    ros::NodeHandle n;
    name = name_input;
    required = required_input;
    initialised = n.getParam(base_parameter_name + name_input, data);
    ROS_ERROR_COND(!initialised && required_input, "Error reading parameter/s %s from rosparam."
                   " Check config file is loaded and type is correct\n", name.c_str());

    if (initialised)
    {
      current_value = data["default"];
      default_value = data["default"];
      max_value = data["max"];
      min_value = data["min"];
      adjust_step = data["step"];
    }
  }

  double current_value; ///< The current value of this adjustable parameter
  double max_value;     ///< The maximum allowed value of this adjustable parameter
  double min_value;     ///< The minimum allowed value of this adjustable parameter
  double default_value; ///< The default value of this adjustable parameter
  double adjust_step;   ///< The allowable increment or decrement of the current value of this adjustable parameter

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This structure contains the parameter objects for all parameters associated with control of the robot, as well as a
 * map object of adjustable parameters. It is used to easily pass parameters amongst controller objects.
 * @todo Remove leg_span_scale parameter if not used in WalkController.
***********************************************************************************************************************/
typedef map<ParameterSelection, AdjustableParameter*> AdjustableMapType;
struct Parameters
{
  AdjustableMapType adjustable_map;  ///< Map between adjustable parameter designations and associated Parameter object.

  // Control parameters
  Parameter<double> time_delta;        ///< The period of time between successive ros cycles.
  Parameter<bool> imu_posing;          ///< Flag denoting if the imu posing feature is on/off.
  Parameter<bool> auto_posing;         ///< Flag denoting if the auto posing feature is on/off.
  Parameter<bool> manual_posing;       ///< Flag denoting if the manual posing feature is on/off.
  Parameter<bool> inclination_posing;  ///< Flag denoting if the inclination posing feature is on/off.
  Parameter<bool> rough_terrain_mode;  ///< Flag denoting if rough terrain mode is on/off (affects various systems).
  Parameter<bool> admittance_control;  ///< Flag denoting if the admittance control feature is on/off.

  // Motor Interface parameters
  Parameter<bool> individual_control_interface;   ///< Flag requesting the individual desired joint position format.
  Parameter<bool> combined_control_interface;     ///< Flag requesting the combined desired joint position format.

  // Model parameters
  Parameter<string> syropod_type;                 ///< The type of the robot described by these parameters.
  Parameter<vector<string>> leg_id;               ///< A vector of identification names for each leg of the robot.
  Parameter<vector<string>> joint_id;             ///< A vector of identification names for each joint of each leg.
  Parameter<vector<string>> link_id;              ///< A vector of identification names for each link of each leg.
  Parameter<map<string, int>> leg_DOF;            ///< A map of the leg name and its degrees of freedom.
  Parameter<bool> clamp_joint_positions;          ///< A bool denoting if joint position limits are adhered to
  Parameter<bool> clamp_joint_velocities;         ///< A bool denoting if joint velocity limits are adhered to
  Parameter<bool> ignore_IK_warnings;             ///< A bool denoting if IK deviation warnings are displayed to user.

  Parameter<map<string, double>> joint_parameters[8][6];  ///< An array of maps of joint parameter names and values.*
  Parameter<map<string, double>> link_parameters[8][7];   ///< An array of maps of link parameter names and values.*
  // *Max possible: 8 legs with 6 joints and 7 links each.

  // Walk controller parameters
  Parameter<string> gait_type;                            ///< The default selected gait type
  Parameter<double> body_clearance;                       ///< The requested height of the robot body above ground.
  AdjustableParameter step_frequency;                     ///< The frequency of complete step cycles (Hz).
  AdjustableParameter swing_height;                       ///< Vertical displacement of swing trajectory above default
  AdjustableParameter swing_width;                        ///< Lateral displacement of swing trajectory out from default
  AdjustableParameter step_depth;                         ///< The stepping depth used to find ground contact.
  AdjustableParameter stance_span_modifier;               ///< The modifier for stance width (-1.0 = min, 1.0 = max)
  Parameter<string> velocity_input_mode;                  ///< Determines velocity input as 'real' or 'throttle' based.
  Parameter<double> body_velocity_scaler;                 ///< Scales all body velocity inputs
  Parameter<bool> force_cruise_velocity;                  ///< Flag denoting if cruise control mode uses set values.
  Parameter<map<string, double>> linear_cruise_velocity;  ///< Set values used in cruise control mode if requested.
  Parameter<double> angular_cruise_velocity;              ///< Set values used in cruise control mode if requested.
  Parameter<double> cruise_control_time_limit;            ///< Time limit after which cruise control input will zero.
  Parameter<map<string, double>> leg_stance_positions[8]; ///< An array of maps of default tip stance positions.
  Parameter<bool> overlapping_walkspaces;                 ///< Flag denoting if walkspaces are allowed to overlap
  Parameter<bool> force_normal_touchdown;                 ///< Flag denoting if tip touches down normal to walk plane
  Parameter<bool> gravity_aligned_tips;                   ///< Flag denoting if tip should align with gravity direction
  Parameter<double> touchdown_threshold;                  ///< Threshold of tip force before touchdown is recognized
  Parameter<double> liftoff_threshold;                    ///< Threshold of tip force before liftoff is recognized

  // Pose controller parameters
  Parameter<string> auto_pose_type;                     ///< String denoting the default auto posing cycle type.
  Parameter<bool> start_up_sequence;                    ///< Flag allowing execution of start up and shutdown sequences.
  Parameter<double> time_to_start;                      ///< The time to complete a direct start up.
  Parameter<map<string, double>> rotation_pid_gains;    ///< PID gains used in imu based automatic posing.
  Parameter<map<string, double>> max_translation;       ///< The maximum allowable linear translation positions.
  Parameter<double> max_translation_velocity;           ///< The maximum allowable linear translation velocity.
  Parameter<map<string, double>> max_rotation;          ///< The maximum allowable angular rotation positions.
  Parameter<double> max_rotation_velocity;              ///< The maximum allowable angular rotation velocity.
  Parameter<string> leg_manipulation_mode;              ///< String denoting the type of leg manipulation.

  // Admittance controller parameters
  Parameter<bool> dynamic_stiffness;         ///< Flag denoting whether the virtual stiffness variable is dynamic.
  Parameter<bool> use_joint_effort;          ///< Flag denoting whether the tip force input is derived from joint effort.
  Parameter<double> integrator_step_time;    ///< The step time used in admittance controller calculations.
  AdjustableParameter virtual_mass;          ///< The virtual mass value used in admittance controller calculations.
  AdjustableParameter virtual_stiffness;     ///< The virtual stiffness value used in admittance controller calculations.
  Parameter<double> load_stiffness_scaler;   ///< The value used to scale the virtual stiffness value for loaded legs.
  Parameter<double> swing_stiffness_scaler;  ///< The value used to scale the virtual stiffness value for swinging legs.
  AdjustableParameter virtual_damping_ratio; ///< The virtual damping value used in admittance controller calculations.
  AdjustableParameter force_gain;            ///< The value used to scale the default tip force input.

  // Gait parameters
  Parameter<int> stance_phase;                   ///< The ratio of the entire step cycle which is in 'stance'.
  Parameter<int> swing_phase;                    ///< The ratio of the entire step cycle which is in 'swing'.
  Parameter<int> phase_offset;                   ///< The base phase offset between step cycles of successive legs.
  Parameter<map<string, int>> offset_multiplier; ///< The leg dependent multiplier used to set the step cycle offset.

  //Auto pose parameters
  Parameter<double> pose_frequency;                         ///< The frequency at which all auto posing cycles run.
  Parameter<int> pose_phase_length;                         ///< The length of all auto posing cycles.
  Parameter<vector<int>> pose_phase_starts;                 ///< The phase at which each auto pose cycle starts.
  Parameter<vector<int>> pose_phase_ends;                   ///< The phase at which each auto pose cycle ends.
  Parameter<map<string, int>> pose_negation_phase_starts;   ///< The phase for start of auto posing negation.
  Parameter<map<string, int>> pose_negation_phase_ends;     ///< The phase for end of auto posing negation.
  Parameter<map<string, double>> negation_transition_ratio; ///< The ratio of the negation period used to transition
  Parameter<vector<double>> x_amplitudes;                   ///< The max amplitudes of x posing in each auto pose cycle.
  Parameter<vector<double>> y_amplitudes;                   ///< The max amplitudes of y posing in each auto pose cycle.
  Parameter<vector<double>> z_amplitudes;                   ///< The max amplitudes of z posing in each auto pose cycle.
  Parameter<vector<double>> gravity_amplitudes;             ///< The max amplitudes of posing in each auto pose cycle.
  Parameter<vector<double>> roll_amplitudes;                ///< The max amplitudes of roll in each auto pose cycle.
  Parameter<vector<double>> pitch_amplitudes;               ///< The max amplitudes of pitch in each auto pose cycle.
  Parameter<vector<double>> yaw_amplitudes;                 ///< The max amplitudes of yaw in each auto pose cycle.

  // Debug Parameters
  Parameter<string> console_verbosity;       ///< The level of verbosity required from the rosconsole output.
  Parameter<bool> debug_moveToJointPosition; ///< Flag determining if function moveToJointPosition() outputs debug info.
  Parameter<bool> debug_stepToPosition;      ///< Flag determining if function stepToPosition() outputs debug info.
  Parameter<bool> debug_swing_trajectory;    ///< Flag determining if swing trajectory generation outputs debug info.
  Parameter<bool> debug_stance_trajectory;   ///< Flag determining if stance trajectory generation outputs debug info.
  Parameter<bool> debug_execute_sequence;    ///< Flag determining if function executeSequence() outputs debug info.
  Parameter<bool> debug_workspace_calc;      ///< Flag determining if workspace calculations output debug info.
  Parameter<bool> debug_IK;                  ///< Flag determining if inverse kinematics engine outputs debug info.
  Parameter<bool> debug_rviz;                ///< Flag determining if visualisation markers are output for debugging.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_PARAMETERS_AND_STATES_H */

