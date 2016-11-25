#pragma once
#include "standardIncludes.h"
#include "parametersAndStates.h"
#include "quat.h"
#include "pose.h"
#include <Eigen/src/Core/Matrix.h>
#include <boost/concept_check.hpp>



// hard coded to 3 joints
struct Leg
{
  LegState legState;
  
  int legIndex;
  int sideIndex;
  
  double yaw;
  double liftAngle;
  double kneeAngle;
  
  // these are all local to parent
  Vector3d rootOffset;
  Vector3d hipOffset;
  Vector3d kneeOffset;
  Vector3d tipOffset;
  
  Vector3d localTipPosition; //actual tip position relative to root

  void init(double startYaw, double startLiftAngle, double startKneeAngle);
  // sets angles to reach local position relative to root
  Vector3d applyLocalIK(Vector3d tipTarget);
  // works out local tip position from angles
  Vector3d applyFK();
  Vector3d calculateFK(double yaw, double liftAngle, double kneeAngle);
  void setState(LegState);
  
  double hipLength;
  double femurLength;
  double femurAngleOffset;
  double tibiaLength;
  double tibiaAngleOffset;
  double minLegLength;
  double maxLegLength;
  double mirrorDir;  // 1 or -1 for mirrored

  double oldYaw;
  double oldLiftAngle;
  double oldKneeAngle;
  
  
  
  struct Model *model; // so it can refer to model's joint limits
};

// defines the hexapod model
struct Model
{
  Leg legs[3][2]; // front to back, left to right
  LegState legStates[3][2];
  Vector3d stanceLegYaws;
  Vector3d yawLimitAroundStance;
  Vector2d minMaxKneeBend;
  Vector2d minMaxHipLift;
  Vector3d localTipPositions[3][2];
  Vector3d stanceTipPositions[3][2];
  Model(Parameters params);
  Vector3d jointMaxAngularSpeeds;
  void setLegStartAngles(int side, int leg, const Vector3d &startAngles);
  vector<Vector3d> getJointPositions(const Pose &pose);
  void clampToLimits(std::map<int, std::string> legNameMap);
  void updateLocal(Vector3d targetTipPositions[3][2], double deltaZ[3][2]);
};
