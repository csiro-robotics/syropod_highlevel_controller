// Top level controller that handles state of hexapod and publishes joint values
#pragma once
#include "standardIncludes.h"
#include "parametersAndStates.h"
#include "model.h"
#include "walkController.h"
#include "poseController.h"
#include "debugOutput.h"
#include "motorInterface.h"
#include "dynamixelMotorInterface.h"
#include "dynamixelProMotorInterface.h"
#include "impedanceController.h"
#include "imu.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "simple_hexapod_controller/legState.h"

#define MAX_MANUAL_LEGS 2
#define UNASSIGNED_VALUE 1e10

class StateController
{
public:  
  inline Parameters* getParameters(void) { return &params_; };
  inline Model* getModel(void) { return model_; };
  inline SystemState getSystemState(void) { return system_state_; };  
  
  inline bool receivingImuData(void) { return imu_data_subscriber_; };
  inline bool receivingTipForces(void) { return tip_force_subscriber_; };
  inline bool receivingJointStates(void) { return joint_state_subscriber_1_ || joint_state_subscriber_2_; };
  inline bool hasAllJointPositions(void) { return received_all_joint_positions_; };
  inline void resetDebug(void) { debug_.reset; };
  
private:
  ros::NodeHandle n_;

  SystemState system_state_ = WAITING_FOR_USER;
  SystemState new_system_state_ = OFF;

  GaitDesignation gait_selection_ = GAIT_UNDESIGNATED;
  PosingMode posing_mode_ = NO_POSING;
  CruiseControlMode cruise_control_mode_ = CRUISE_CONTROL_OFF;
  AutoNavigationMode auto_navigation_mode_ = AUTO_NAVIGATION_OFF;

  ParameterSelection parameter_selection_ = NO_PARAMETER_SELECTION;

  LegDesignation primary_leg_selection_ = LEG_UNDESIGNATED;
  LegDesignation secondary_leg_selection_ = LEG_UNDESIGNATED;
  LegState primary_leg_state_ = WALKING;
  LegState secondary_leg_state_ = WALKING;

  Parameters params_;
  std::map<std::string, Parameter> params_map_;
  std::map<std::string, Parameter> gait_params_map_;  

  Model* model_;
  MotorInterface* interface_;

  WalkController* walker_;
  PoseController* poser_;
  ImpedanceController* impedance_;

  DebugOutput debug_;
  int manual_leg_count_ = 0;
  
  ros::Subscriber imu_data_subscriber_;
  ros::Subscriber tip_force_subscriber_;
  ros::Subscriber joint_state_subscriber_1_;
  ros::Subscriber joint_state_subscriber_2_;

  ros::Publisher pose_publisher_;
  ros::Publisher imu_data_publisher_;
  ros::Publisher body_velocity_publisher_;

  ros::Publisher rotation_pose_error_publisher_;
  ros::Publisher translation_pose_error_publisher_;
  ros::Publisher z_tip_error_publisher_;

  // Trigger Flags
  bool gait_change_flag_ = false;
  bool toggle_primary_leg_state_ = false;
  bool toggle_secondary_leg_state_ = false;
  bool parameter_adjust_flag_ = false;
  bool new_parameter_set_ = false;

  ImuData imu_data_;

  // Joint states callback variables
  // sensor_msgs::JointState jointStates;
  vector<double> joint_positions_;
  double jointVelocities[18];
  double jointEfforts[18];
  double tipForces[6];
  bool received_all_joint_positions_ = false;

  // Param selection/adjust callback variables
  double param_scaler_ = 1.0;
  double param_value_;
  double param_adjust_sensitivity_;

  // Body Velocity callback variables
  Vector2d linear_velocity_input_;
  double angular_velocity_input_ = 0;
  Vector3d primaryManualTipVelocity;
  Vector3d secondaryManualTipVelocity;

  // Cruise control callback variables
  Vector2d linearCruiseVelocity;
  double angularCruiseVelocity;

  // Joypad pose callback variables
  double pitch_input_ = 0;
  double roll_input_ = 0;
  double yawJoy = 0;
  double xJoy = 0;
  double yJoy = 0;
  double zJoy = 0;
  PoseResetMode poseResetMode = ALL_RESET;

  // Impedance control variables
  double deltaZ[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  double tipForce[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  bool use_tip_force_ = false;
  bool use_joint_effort_ = false;

  // Misc variables
  bool legStateDebounce = false;
  bool startTestDebounce = false;
  int firstIteration = 0;

  // Initialisation functions
  StateController(ros::NodeHandle nodeHandle);
  ~StateController();
  void init();
  void getParameters();
  void getControlParameters(void);
  void getModelParameters(void);
  void getWalkerParameters(void);
  void getGaitParameters(GaitDesignation gaitSelection);
  void getPoserParameters(void);
  void getImpedanceControlParameters(void);
  void getDebugParameters(void);
  void setJointPositions(bool useDefaults = false);
  void populateLegMaps();
  void populateParameterNameMap();

  // Top level functions
  void publishJointValues();

  // Debugging functions
  void RVIZDebugging();
  void publishLegState();

  void publishPose();
  void publishIMUData();
  void publishBodyVelocity();

  void publishRotationPoseError();
  void publishTranslationPoseError();
  void publishZTipError();

  // Loop and state functions
  void loop();
  void transitionSystemState();
  void unpackState();
  void startUpState();
  void shutDownState();
  void packState();
  void packedState();
  void directState();
  void unknownState();
  void runningState();

  // Running state sub-functions
  void updatePose();
  void impedanceControl();
  void parameterAdjust();
  void gaitChange();
  void legStateToggle();

  // Callbacks
  void bodyVelocityInputCallback(const geometry_msgs::Twist &input);
  void primaryTipVelocityInputCallback(const geometry_msgs::Point &input);
  void secondaryTipVelocityInputCallback(const geometry_msgs::Point &input);
  void bodyPoseInputCallback(const geometry_msgs::Twist &input);

  void systemStateCallback(const std_msgs::Int8 &input);
  void gaitSelectionCallback(const std_msgs::Int8 &input);
  void posingModeCallback(const std_msgs::Int8 &input);
  void cruiseControlCallback(const std_msgs::Int8 &input);
  void autoNavigationCallback(const std_msgs::Int8 &input);

  void parameterSelectionCallback(const std_msgs::Int8 &input);
  void parameterAdjustCallback(const std_msgs::Int8 &input);

  void primaryLegSelectionCallback(const std_msgs::Int8 &input);
  void secondaryLegSelectionCallback(const std_msgs::Int8 &input);
  void primaryLegStateCallback(const std_msgs::Int8 &input);
  void secondaryLegStateCallback(const std_msgs::Int8 &input);

  void poseResetCallback(const std_msgs::Int8 &input);

  void jointStatesCallback(const sensor_msgs::JointState &jointStates);
  void imuCallback(const sensor_msgs::Imu &imuData);
  void tipForceCallback(const sensor_msgs::JointState &jointStates);
};