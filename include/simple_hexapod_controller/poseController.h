//Controller that handles changes in body pose
#pragma once
#include "parametersAndStates.h"
#include "model.h"
#include "debugOutput.h"
#include "walkController.h"
#include "imu.h"

struct PoseController
{
  Model *model;
  WalkController *walker;
  Parameters params;
  
  double timeDelta;
  bool firstIteration = true;
  int masterIterationCount = 0;
  
  double moveToPoseTime = 0.0;
  
  Vector3d tipPositions[3][2];
  
  Vector3d originTipPositions[3][2];
  Vector3d midTipPositions[3][2];
  bool hasStepped[3][2];
  
  Vector3d originJointPositions[3][2];
  
  //Used in startup and shutdown sequences
  int sequenceStep = 0;
  double startHeightRatio;
  Vector3d phase1TipPositions[3][2];
  Vector3d phase2TipPositions[3][2];
  Vector3d phase3TipPositions[3][2];
  Vector3d phase4TipPositions[3][2];
  Vector3d phase5TipPositions[3][2];
  Vector3d phase6TipPositions[3][2];
  Vector3d phase7TipPositions[3][2];
  Vector3d phase8TipPositions[3][2];  
  
  Pose currentPose;//Current pose of body including all compensation posing  
  
  Pose targetPose; //Target pose of body for use in manual posing bezier curve
  Pose originPose; //Origin pos of body for use in manual posing bezier curve
  
  Pose manualPose; //Current pose of body only using manual compensation
  Pose imuPose; //Current pose of body only using imu compensation
  Pose inclinationPose; //Current pose of body only using inclination compensation
  Pose deltaZPose; //Current pose of body only using impedance control body height compensation
  
  Pose defaultPose;
  
  Pose autoPoseDefault; //Current pose of body only using auto compensation
  Pose autoPose[3][2]; //Leg specific auto compensation - equal to default but with zero pose compensation on leg swing phase
  
  bool recalculateOffset = true;
  
  //Imu compensation PID error vectors
  Vector3d rotationAbsementError;
  Vector3d rotationPositionError;
  Vector3d rotationVelocityError;
  
  Vector3d translationAbsementError;
  Vector3d translationPositionError;
  Vector3d translationVelocityError;
  Vector3d translationAccelerationError;
  
  
  bool correctPose = false; //Flag for correcting pose to a zero roll/pitch pose used for auto compensation
  
  PoseController(Model *model, WalkController *walker, Parameters params);
  void updateStance(Vector3d targetTipPositions[3][2], 
                    bool excludeSwingingLegs=false);
  double stepToPosition(Vector3d targetTipPositions[3][2], 
			Pose targetPose,
			double deltaZ[3][2],
			int mode=NO_STEP_MODE,                      
			double stepHeight = 0.0, 
			double stepSpeed = 2.0);
  bool moveToJointPosition(Vector3d targetJointPositions[3][2], double speed=2.0);
  bool startUpSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode);
  bool shutDownSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode);
  double createSequence(Vector3d targetTipPositions[3][2]); 
  void resetSequence(void);
  
  double poseForLegManipulation(LegState state, int l, int s, double deltaZ[3][2]);
  
  void calculateDefaultPose();
  
  //Compensation functions
  void autoCompensation(void);
  void manualCompensation(Vector3d translationVelocityInput, 
			  Vector3d rotationVelocityInput, 
			  PoseResetMode poseResetMode, 
			  Pose defaultPose = Pose::identity());
  void imuCompensation (ImuData imuData, Quat targetRotation);
  void inclinationCompensation(ImuData imuData);
  void impedanceControllerCompensation(double deltaZ[3][2]);
  
  double debugTime = 0.0;
  
};
