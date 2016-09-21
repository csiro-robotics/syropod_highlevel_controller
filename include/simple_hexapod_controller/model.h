#pragma once
#include "standardIncludes.h"
#include "quat.h"
#include "pose.h"
#include <Eigen/src/Core/Matrix.h>
#include <boost/concept_check.hpp>

enum LegState
{
  WALKING,
  OFF,
  WALKING_TO_OFF,
  OFF_TO_WALKING,
};

// hard coded to 3 joints
struct Leg
{
  LegState state;
  
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
  Vector3d stanceTipPosition;//default tip position at rest relative to root

  void init(double startYaw, double startLiftAngle, double startKneeAngle);
  // sets angles to reach local position relative to root
  Vector3d applyLocalIK(Vector3d tipTarget, bool updateStance = false);
  void applyWorldIK(const Pose &rootPose, const Vector3d &worldTipTarget){ applyLocalIK(rootPose.inverseTransformVector(worldTipTarget)); }
  // works out local tip position from angles
  Vector3d applyFK(bool updateStance = false);
  Vector3d calculateFK(double yaw, double liftAngle, double kneeAngle);
  
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
  void clampToLimits();
};
