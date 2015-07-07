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
  double getMinLength(double maximumKneeBend) const { return max(0.0, sqrt(sqr(tibiaLength) + sqr(femurLength) - 2.0*femurLength*tibiaLength*cos(pi-maximumKneeBend))); }
  // works out local tip position from angles
  void applyFK();
  double hipLength;
  double femurLength;
  double femurAngleOffset;
  double tibiaLength;
  double tibiaAngleOffset;
  double legLength;
  double mirrorDir;  // 1 or -1 for mirrored
};

// defines the hexapod model
struct Model
{
  Leg legs[3][2]; // front to back, left to right
  Model();
  vector<Vector3d> getJointPositions(const Pose &pose);
};
