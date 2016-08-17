//Top level controller that handles state of hexapod and publishes joint values
#pragma once
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/walkController.h"
#include "../include/simple_hexapod_controller/poseController.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include "../include/simple_hexapod_controller/imuCompensation.h"
#include "../include/simple_hexapod_controller/impedanceController.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32MultiArray.h"
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/Joy.h"

struct StateController
{
  ros::NodeHandle n;
  
  State state = UNKNOWN;

  Gait gait = DEFAULT;
  LegSelection legSelection = NO_LEG_SELECTION;
  ParamSelection paramSelection = NO_PARAM_SELECTION;
  LoadShareMode loadShareMode = EQUAL;

  Parameters params;
  Parameters defaultParams;
  
  double runningTime = 0;
  
  Model *hexapod;
  
  MotorInterface *interface;
  
  WalkController *walker;
  PoseController *poser;
  ImpedanceController *impedance;
  
  Imu *imu;
  
  DebugOutput debug;
  int count = 0;
  
  ros::Publisher tipPositionPublishers[3][2];
  ros::Publisher tipForcePublisher;
  ros::Publisher deltaZPublisher;
  ros::Publisher posePublisher;
  ros::Publisher IMURotationPublisher;
  ros::Publisher stiffnessPublisher;
  ros::Publisher rotationPoseErrorPublisher;  
  ros::Publisher translationPoseErrorPublisher; 
  ros::Publisher zTipErrorPublisher;
  
  //Trigger Flags
  bool changeGait = false;
  bool toggleLegState = false;
  bool adjustParam = false;
  bool unstable = false;
  
  //Joint states callback variables
  //sensor_msgs::JointState jointStates;
  double jointPositions[18];
  double jointEfforts[18];
  double tipForces[6];
  bool jointPosFlag = false;
  
  //Start callback variables
  bool startFlag = false;
  
  //Param selection/adjust callback variables
  double paramScaler = 1.0;
  double paramAdjustSensitivity;
  
  //Joypad velocity callback variables
  Vector2d localVelocity;
  double turnRate = 0; 
  double velocityMultiplier = 1.0;
  double poseTimeJoy = 2.0;
  
  //Joypad pose callback variables
  double pitchJoy = 0;
  double rollJoy = 0;
  double yawJoy = 0;
  double xJoy = 0;
  double yJoy = 0;
  double zJoy = 0;  
  
  //Impedance control variables
  double deltaZ[3][2] = {{0,0},{0,0},{0,0}};
  double tipForce[3][2] = {{0,0},{0,0},{0,0}};
  bool useTipForce = false;
  bool useJointEffort = false;
  
  //Packed/Unpacked joint position arrays
  Vector3d packedJointPositions[3][2];
  Vector3d unpackedJointPositions[3][2];
  
  //Misc variables
  bool debounce = false;
  int firstIteration = 0;
  
  //Initialisation functions
  StateController(ros::NodeHandle nodeHandle);
  ~StateController();
  void init();
  void getParameters();
  void getGaitParameters(std::string forceGait);
  void setJointPositions(bool useDefaults = false);
  
  //Top level functions
  void publishJointValues();
  
  //Debugging functions
  void RVIZDebugging();
  void publishTipPositions();
  void publishTipForces();
  void publishDeltaZ();
  void publishPose();
  void publishIMURotation();
  void publishStiffness();
  void publishRotationPoseError();
  void publishTranslationPoseError();
  void publishZTipError();
  
  //Loop and state functions
  void loop();
  void unpackState();
  void startUpState();
  void shutDownState();
  void packState();
  void packedState();
  void directState();
  void unknownState();
  void runningState();
  
  //Running state sub-functions
  void compensation();
  void impedanceControl();
  void paramAdjust();
  void gaitChange();
  void legStateToggle();
  
  //Callbacks
  void jointStatesCallback(const sensor_msgs::JointState &jointStates);
  void imuCallback(const sensor_msgs::Imu &imuData);
  void tipForceCallback(const sensor_msgs::JointState &jointStates);
  void joypadVelocityCallback(const geometry_msgs::Twist &twist);
  void joypadPoseCallback(const geometry_msgs::Twist &twist);
  void imuControllerCallback(const sensor_msgs::Joy &input);
  void gaitSelectionCallback(const std_msgs::Int8 &input);
  void legSelectionCallback(const std_msgs::Int8 &input);
  void legStateCallback(const std_msgs::Bool &input);
  void startCallback(const std_msgs::Bool &input);
  void paramSelectionCallback(const std_msgs::Int8 &input);
  void paramAdjustCallback(const std_msgs::Int8 &input);
};