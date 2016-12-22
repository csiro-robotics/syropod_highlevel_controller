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

struct StateController
{
  ros::NodeHandle n;

  SystemState systemState = UNKNOWN;
  SystemState newSystemState = OFF;

  GaitDesignation gaitSelection = GAIT_UNDESIGNATED;
  PosingMode posingMode = NO_POSING;
  CruiseControlMode cruiseControlMode = CRUISE_CONTROL_OFF;
  AutoNavigationMode autoNavigationMode = AUTO_NAVIGATION_OFF;

  ParameterSelection parameterSelection = NO_PARAMETER_SELECTION;

  LegDesignation primaryLegSelection = LEG_UNDESIGNATED;
  LegDesignation secondaryLegSelection = LEG_UNDESIGNATED;
  LegState primaryLegState = WALKING;
  LegState secondaryLegState = WALKING;

  Parameters params;
  Parameters defaultParams;

  double runningTime = 0;

  bool isInitialised = false;

  Model *hexapod;

  MotorInterface *interface;

  WalkController *walker;
  PoseController *poser;
  ImpedanceController *impedance;

  DebugOutput debug;
  int count = 0;
  int manualLegs = 0;

  std::map<std::string, int> legIndexMap;
  std::map<int, std::string> legNameMap;
  std::map<ParameterSelection, std::string> parameterNameMap;

  ros::Publisher legStatePublishers[3][2];
  ros::Publisher ascLegStatePublishers[3][2];

  ros::Publisher posePublisher;
  ros::Publisher IMUDataPublisher;
  ros::Publisher bodyVelocityPublisher;

  ros::Publisher rotationPoseErrorPublisher;
  ros::Publisher translationPoseErrorPublisher;
  ros::Publisher zTipErrorPublisher;

  // Trigger Flags
  bool transitionSystemStateFlag = false;
  bool gaitChangeFlag = false;
  bool togglePrimaryLegState = false;
  bool toggleSecondaryLegState = false;
  bool parameterAdjustFlag = false;
  bool newParamSet = false;
  bool unstable = false;

  ImuData imuData;

  // Joint states callback variables
  // sensor_msgs::JointState jointStates;
  double jointPositions[18];
  double jointVelocities[18];
  double jointEfforts[18];
  double tipForces[6];
  bool recievedAllJointPositions = false;

  // Start callback variables
  bool startFlag = false;

  // Param selection/adjust callback variables
  double paramScaler = 1.0;
  std::string paramString;
  double paramVal;
  double paramAdjustSensitivity;

  // Body Velocity callback variables
  Vector2d linearVelocityInput;
  double angularVelocityInput = 0;
  Vector3d primaryManualTipVelocity;
  Vector3d secondaryManualTipVelocity;

  // Cruise control callback variables
  Vector2d linearCruiseVelocity;
  double angularCruiseVelocity;

  // Joypad pose callback variables
  double pitchJoy = 0;
  double rollJoy = 0;
  double yawJoy = 0;
  double xJoy = 0;
  double yJoy = 0;
  double zJoy = 0;
  PoseResetMode poseResetMode = ALL_RESET;

  // Impedance control variables
  double deltaZ[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  double tipForce[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  bool useTipForce = false;
  bool useJointEffort = false;

  // Packed/Unpacked joint position arrays
  Vector3d packedJointPositions[3][2];
  Vector3d unpackedJointPositions[3][2];

  // Misc variables
  bool legStateDebounce = false;
  bool startTestDebounce = false;
  int firstIteration = 0;

  // Initialisation functions
  StateController(ros::NodeHandle nodeHandle);
  ~StateController();
  void init();
  void getParameters();
  void getGaitParameters(GaitDesignation gaitSelection);
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
  void updatePose()();
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