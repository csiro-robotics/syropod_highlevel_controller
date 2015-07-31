#pragma once
#include "standardIncludes.h"
#include "quat.h"
#include "pose.h"
#include <Eigen/src/Core/Matrix.h>
#include <boost/concept_check.hpp>

// hard coded to 3 joints
struct Leg
{
  double yaw;
  double liftAngle;
  double kneeAngle;
  
  // these are all local to parent
  Vector3d rootOffset;
  Vector3d hipOffset;
  Vector3d kneeOffset;
  Vector3d tipOffset;
  
  Vector3d localTipPosition; // relative to root

  void init(double startYaw, double startLiftAngle, double startKneeAngle);
  // sets angles to reach local position relative to root
  void applyLocalIK(Vector3d tipTarget, bool updateTipPos = true);
  void applyWorldIK(const Pose &rootPose, const Vector3d &worldTipTarget){ applyLocalIK(rootPose.inverseTransformVector(worldTipTarget)); }
  // works out local tip position from angles
  void applyFK();
  double hipLength;
  double femurLength;
  double femurAngleOffset;
  double tibiaLength;
  double tibiaAngleOffset;
  double minLegLength;
  double maxLegLength;
  double mirrorDir;  // 1 or -1 for mirrored

  double debugOldYaw;
  double debugOldLiftAngle;
  double debugOldKneeAngle;
  
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
  Model(const Vector3d &stanceLegYaws, const Vector3d &yawLimitAroundStance, const Vector2d &minMaxKneeBend = Vector2d(0,3), const Vector2d &minMaxHipLift = Vector2d(-3,3));
  void setLegStartAngles(int side, int leg, const Vector3d &startAngles);
  vector<Vector3d> getJointPositions(const Pose &pose);
  void clampToLimits();
};
