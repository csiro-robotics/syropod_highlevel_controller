#pragma once
#include "standardIncludes.h"

#define THROTTLE_PERIOD 5  // seconds

enum LegType
{
  UNKNOWN_LEG_TYPE,
  ONE_DOF,
  TWO_DOF,
  THREE_DOF, //Only type implemented
  FOUR_DOF,
  FIVE_DOF,
};

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
  LEG_AL,
  LEG_AR,
  LEG_BL,
  LEG_BR,
  LEG_CL,
  LEG_CR,
  LEG_DL,
  LEG_DR,
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

struct Parameters
{
  // Control parameters
  Parameter time_delta;
  Parameter imu_compensation;
  Parameter auto_compensation;
  Parameter manual_compensation;
  Parameter inclination_compensation;
  Parameter impedance_control;
  Parameter use_dynamixel_pro_interface;
  Parameter imu_rotation_offset;
  Parameter interface_setup_speed;
  // Model parameters
  Parameter hexapod_type;
  Parameter num_legs;
  Parameter leg_DOF;
  Parameter leg_id_names;
  Vector3d coxa_offset[3][2];
  Vector3d femur_offset[3][2];
  Vector3d tibia_offset[3][2];
  Vector3d tip_offset[3][2];
  Vector3d stance_coxa_angles;
  Vector3d physical_coxa_angle_offset;
  double physical_tibia_angle_offset;
  Vector3d coxa_joint_limits;
  Vector2d tibia_joint_limits;
  Vector2d femur_joint_limits;
  Vector3d joint_max_angular_speeds;
  
  

  // Walk Controller Parameters
  std::string gait_type;
  double step_frequency;
  double step_clearance;
  double step_depth;
  double body_clearance;
  double leg_span_scale;
  double max_linear_acceleration;
  double max_angular_acceleration;
  double footprint_downscale;
  
  std::string velocity_input_mode;

  bool force_cruise_velocity;
  Vector2d linear_cruise_velocity;
  double angular_cruise_velocity;

  // Pose Controller Parameters
  bool start_up_sequence;
  double time_to_start;

  double rotation_gain_p;
  double rotation_gain_i;
  double rotation_gain_d;
  double translation_gain_p;
  double translation_gain_i;
  double translation_gain_d;

  double pitch_amplitude;
  double roll_amplitude;
  double z_translation_amplitude;

  Vector3d max_translation;
  double max_translation_velocity;

  Vector3d max_rotation;
  double max_rotation_velocity;

  std::string leg_manipulation_mode;

  Vector3d packed_joint_positions_AL;
  Vector3d packed_joint_positions_AR;
  Vector3d packed_joint_positions_BL;
  Vector3d packed_joint_positions_BR;
  Vector3d packed_joint_positions_CL;
  Vector3d packed_joint_positions_CR;
  Vector3d unpacked_joint_positions_AL;
  Vector3d unpacked_joint_positions_AR;
  Vector3d unpacked_joint_positions_BL;
  Vector3d unpacked_joint_positions_BR;
  Vector3d unpacked_joint_positions_CL;
  Vector3d unpacked_joint_positions_CR;

  // Impedance Controller Parameters
  
  bool dynamic_stiffness;
  double integrator_step_time;
  double virtual_mass;
  double virtual_stiffness;
  double load_stiffness_scaler;
  double swing_stiffness_scaler;
  double virtual_damping_ratio;
  double force_gain;
  std::string impedance_input;

  // Gait Parameters
  double stance_phase;
  double swing_phase;
  double phase_offset;
  std::vector<int> offset_multiplier;

  // Debug Parameters
  bool debug_rviz;
  std::string console_verbosity;
  bool debug_moveToJointPosition;
  bool debug_stepToPosition;
  bool debug_swing_trajectory;
  bool debug_stance_trajectory;
};

template<typename T>
struct Parameter
{
  std::string name;
  T current_value;
  T default_value;
};
  
