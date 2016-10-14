//Controller that handles changes in body pose
#pragma once
#include "model.h"
#include "debugOutput.h"
#include "walkController.h"

//stepToPosition modes
#define NO_STEP_MODE 0
#define SIMULTANEOUS_MODE 1
#define TRIPOD_MODE 2
#define SEQUENTIAL_MODE 3

struct PoseController
{
  Model *model;
  WalkController *walker;
  Parameters params;
  
  double timeDelta;
  bool firstIteration = true;
  int masterIterationCount = 0;
  
  double moveToPoseTime = 0.0;
  
  Vector3d originTipPositions[3][2];
  Vector3d midTipPositions[3][2];
  
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
  Pose autoPose; //Current pose of body only using auto compensation
  Pose imuPose; //Current pose of body only using imu compensation
  Pose inclinationPose; //Current pose of body only using inclination compensation
  
  bool correctPose = false; //Flag for correcting pose to a zero roll/pitch pose used for auto compensation
  
  PoseController(Model *model, WalkController *walker, Parameters params);
  bool updateStance(Vector3d targetTipPositions[3][2], 
                    bool excludeSwingingLegs=false);
  bool stepToPosition(Vector3d targetTipPositions[3][2], 
                      double deltaZ[3][2],
                      int mode=NO_STEP_MODE,                      
                      double stepHeight = 0.0, 
                      double stepSpeed = 2.0);
  bool moveToJointPosition(Vector3d targetJointPositions[3][2], double speed=2.0);
  bool startUpSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode);
  bool shutDownSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode);
  double createSequence(Vector3d targetTipPositions[3][2]); 
  void resetSequence(void);
  void autoCompensation(void);
  bool manualCompensation(Pose requestedTargetPose, double timeToPose);
};
