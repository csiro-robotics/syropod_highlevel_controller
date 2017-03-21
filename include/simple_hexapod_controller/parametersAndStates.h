#ifndef SIMPLE_HEXAPOD_CONTROLLER_PARAMETERS_AND_STATES_H
#define SIMPLE_HEXAPOD_CONTROLLER_PARAMETERS_AND_STATES_H
/** 
 *  \file    parameters_and_states.h
 *  \brief   Defines various hexapod and system parameters and states. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   January 2017
 *  \version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
 */

#include "standardIncludes.h"
#define THROTTLE_PERIOD 5  // seconds

enum SystemState
{
  OFF,
  PACKED,
  READY,
  RUNNING,
  UNKNOWN = -1,
  WAITING_FOR_USER = -2,
};

enum GaitDesignation
{
  WAVE_GAIT,
  AMBLE_GAIT,
  RIPPLE_GAIT,
  TRIPOD_GAIT,
  GAIT_UNDESIGNATED = -1,
};

enum PosingMode
{
  NO_POSING,
  X_Y_POSING,
  PITCH_ROLL_POSING,
  Z_YAW_POSING,
};

enum CruiseControlMode
{
  CRUISE_CONTROL_OFF,
  CRUISE_CONTROL_ON,
};

enum AutoNavigationMode
{
  AUTO_NAVIGATION_OFF,
  AUTO_NAVIGATION_ON,
};

enum LegState
{
  WALKING,
  MANUAL,
  WALKING_TO_MANUAL,
  MANUAL_TO_WALKING,
};

enum WalkState
{
  STARTING,
  MOVING,
  STOPPING,
  STOPPED
};

enum StepState
{
  SWING,
  STANCE,
  FORCE_STANCE,
  FORCE_STOP
};

enum LegCoordinationMode
{
  SIMULTANEOUS_MODE,
  TRIPOD_MODE,
  SEQUENTIAL_MODE,
};

enum PoseResetMode
{
  NO_RESET,
  Z_AND_YAW_RESET,
  X_AND_Y_RESET,
  PITCH_AND_ROLL_RESET,
  ALL_RESET,
  IMMEDIATE_ALL_RESET,  // overrides input from user
};

enum LegDesignation
{
  LEG_0,
  LEG_1,
  LEG_2,
  LEG_3,
  LEG_4,
  LEG_5,
  LEG_6,
  LEG_7,
  LEG_UNDESIGNATED = -1,
};

enum ParameterSelection
{
  NO_PARAMETER_SELECTION,
  STEP_FREQUENCY,
  STEP_CLEARANCE,
  BODY_CLEARANCE,
  LEG_SPAN_SCALE,
  VIRTUAL_MASS,
  VIRTUAL_STIFFNESS,
  VIRTUAL_DAMPING,
  FORCE_GAIN,
};

template <typename T>
struct Parameter
{
  string name; 
  T data;
  bool required = true;
  bool initialised = false;
  
  void init(ros::NodeHandle n, string name_input, string base_parameter_name = "/hexapod/parameters/", bool required_input = true)
  {
    name = name_input;
    required = required_input;
    bool parameter_found = n.getParam(base_parameter_name + name_input, data);
    if (!parameter_found && required)
    {
      ROS_ERROR("Error reading parameter/s %s from rosparam. Check config file is loaded and type is correct\n",
		name.c_str());
    }
    else
    {
      initialised = true;
    }
  }
};

struct AdjustableParameter : public Parameter<map<string, double>>
{
	double current_value;
	double max_value;
	double min_value;
	double default_value;
	double adjust_step;

	void init(ros::NodeHandle n, 
			string name_input,
			string base_parameter_name = "/hexapod/parameters/",
			bool required_input = true)
	{
		name = name_input;
		required = required_input;
		bool parameter_found = n.getParam(base_parameter_name + name_input, data);
		if (!parameter_found && required)
		{
			ROS_ERROR("Error reading parameter/s %s from rosparam. Check config file is loaded and type is correct\n", name.c_str());
		}
		else
		{
			current_value = data["default"];
			default_value = data["default"];
			max_value = data["max"];
			min_value = data["min"];
			adjust_step = data["step"];
			initialised = true;
		}
	}
};

typedef map<ParameterSelection, AdjustableParameter*> AdjustableMapType;

struct Parameters
{      
	AdjustableMapType adjustable_map;

	// Control parameters
	Parameter<double> time_delta;
	Parameter<bool> imu_compensation;
	Parameter<bool> auto_compensation;
	Parameter<bool> manual_compensation;
	Parameter<bool> inclination_compensation;
	Parameter<bool> impedance_control;
	Parameter<vector<double>> imu_rotation_offset;
	Parameter<double> interface_setup_speed;
	// Model parameters
	Parameter<string> 	hexapod_type;
	Parameter<vector<string>> leg_id;
	Parameter<vector<string>> joint_id;
	Parameter<vector<string>> link_id;
	Parameter<map<string, int>> leg_DOF;
	Parameter<map<string, double>> leg_stance_yaws;
	Parameter<map<string, double>> joint_parameters[8][6]; //Max 8 legs with 6 joints
	Parameter<map<string, double>> link_parameters[8][7];
	// Walk controller parameters
	Parameter<string> gait_type;
	AdjustableParameter step_frequency;
	AdjustableParameter step_clearance;
	Parameter<double> step_depth;
	AdjustableParameter body_clearance;
	AdjustableParameter leg_span;
	Parameter<string> velocity_input_mode;
	Parameter<bool> force_cruise_velocity;
	Parameter<map<string, double>> linear_cruise_velocity;
	Parameter<double> angular_cruise_velocity;
	// Pose controller parameters
	Parameter<bool> start_up_sequence;
	Parameter<double> time_to_start;
	Parameter<map<string, double>> rotation_pid_gains;
	Parameter<map<string, double>> translation_pid_gains;
	Parameter<map<string, double>> auto_compensation_parameters;
	Parameter<map<string, double>> max_translation;
	Parameter<double> max_translation_velocity;
	Parameter<map<string, double>> max_rotation;
	Parameter<double> max_rotation_velocity;
	Parameter<string> leg_manipulation_mode;
	// Impedance controller parameters  
	Parameter<bool> dynamic_stiffness;
	Parameter<bool> use_joint_effort;
	Parameter<double> integrator_step_time;
	AdjustableParameter virtual_mass;
	AdjustableParameter virtual_stiffness;
	Parameter<double> load_stiffness_scaler;
	Parameter<double> swing_stiffness_scaler;
	AdjustableParameter virtual_damping_ratio;
	AdjustableParameter force_gain;

	// Gait parameters
	Parameter<int> stance_phase;
	Parameter<int> swing_phase;
	Parameter<int> phase_offset;
	Parameter<vector<int>> offset_multiplier;
	// Debug Parameters
	Parameter<bool> debug_rviz;
	Parameter<bool> debug_rviz_static_display;
	Parameter<string> console_verbosity;
	Parameter<bool> debug_moveToJointPosition;
	Parameter<bool> debug_stepToPosition;
	Parameter<bool> debug_swing_trajectory;
	Parameter<bool> debug_stance_trajectory;
	Parameter<bool> debug_IK;
};
#endif /* SIMPLE_HEXAPOD_CONTROLLER_PARAMETERS_AND_STATES_H */
  
