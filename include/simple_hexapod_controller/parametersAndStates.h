#pragma once
#include "standardIncludes.h"

#define THROTTLE_PERIOD 5 //seconds

enum SystemState
{
  OFF,
  PACKED,
  READY,
  RUNNING,
  UNKNOWN = -1,
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

enum StepToPositionModes
{
  NO_STEP_MODE,
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
  IMMEDIATE_ALL_RESET, //overrides input from user
};

enum LegDesignation
{
  FRONT_LEFT,
  FRONT_RIGHT,
  MIDDLE_LEFT,
  MIDDLE_RIGHT,
  REAR_LEFT,
  REAR_RIGHT,
  LEG_UNDESIGNATED,
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
  std::string hexapodType;
  double timeDelta;  
  bool imuCompensation;
  bool autoCompensation;  
  bool manualCompensation;
  bool inclinationCompensation;

  //Hexapod Parameters
  Vector3d rootOffset[3][2];
  Vector3d hipOffset[3][2];
  Vector3d kneeOffset[3][2];
  Vector3d tipOffset[3][2];
  Vector3d stanceLegYaws;
  Vector3d physicalYawOffset;
  double physicalKneeOffset;
  Vector3d yawLimits;
  Vector2d kneeLimits;
  Vector2d hipLimits;
  Vector3d jointMaxAngularSpeeds;
  bool dynamixelInterface;
  Vector3d imuRotationOffset;

  //Walk Controller Parameters
  std::string gaitType;  
  double stepFrequency;
  double stepClearance;
  double stepDepth;
  double bodyClearance;
  double legSpanScale; 
  double maxLinearAcceleration;
  double maxAngularAcceleration;
  double footprintDownscale;
  double interfaceSetupSpeed;
  std::string velocityInputMode; 
  
  bool forceCruiseVelocity;
  Vector2d linearCruiseVelocity;
  double angularCruiseVelocity;
  
  //Pose Controller Parameters
  bool startUpSequence;
  bool moveLegsSequentially;
  double timeToStart;
  
  double rotationCompensationProportionalGain;
  double rotationCompensationIntegralGain;
  double rotationCompensationDerivativeGain;
  double translationCompensationProportionalGain;
  double translationCompensationIntegralGain;
  double translationCompensationDerivativeGain;  
  
  double pitchAmplitude;
  double rollAmplitude;
  double zTransAmplitude;
  
  Vector3d maxTranslation;
  double maxTranslationVelocity;
  
  Vector3d maxRotation;
  double maxRotationVelocity;
  
  std::string legManipulationMode;
  
  Vector3d packedJointPositionsAL;
  Vector3d packedJointPositionsAR;
  Vector3d packedJointPositionsBL;
  Vector3d packedJointPositionsBR;
  Vector3d packedJointPositionsCL;
  Vector3d packedJointPositionsCR;
  Vector3d unpackedJointPositionsAL;
  Vector3d unpackedJointPositionsAR;
  Vector3d unpackedJointPositionsBL;
  Vector3d unpackedJointPositionsBR;
  Vector3d unpackedJointPositionsCL;
  Vector3d unpackedJointPositionsCR;
  
  //Impedance Controller Parameters
  bool impedanceControl;
  bool dynamicStiffness;
  double integratorStepTime;
  double virtualMass;
  double virtualStiffness;
  double loadStiffnessScaler;
  double swingStiffnessScaler;
  double virtualDampingRatio;
  double forceGain;
  std::string impedanceInput;
    
  //Gait Parameters
  double stancePhase;
  double swingPhase;
  double phaseOffset;  
  std::vector<int> offsetMultiplier;
  
  //Debug Parameters
  bool debug_rviz;
  std::string consoleVerbosity;
  bool debugMoveToJointPosition;
  bool debugStepToPosition;
  bool debugSwingTrajectory;
  bool debugStanceTrajectory;
  bool debugManualCompensationRotation;
  bool debugManualCompensationTranslation;  
};



