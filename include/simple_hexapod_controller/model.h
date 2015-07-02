#pragma once
#include "standardIncludes.h"
#include "quat.h"
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
  
  bool onGround;
  Vector3d localTipPosition; // relative to root

  // sets angles to reach local position relative to root
  void applyLocalIK(const Vector3d &tipTarget)
  {
    // application of cosine rule
    Vector3d target = tipTarget - rootOffset; // since rootOffset is fixed in root's space
    yaw = atan2(target[1], target[0]);
    Quat quat(Vector3d(0,0,yaw);
    target = quat.inverseRotateVector(target); // localise
    
    target -= hipOffset;
    target[1] = 0; // any offset here cannot be reached
    double targetLength = target.norm();
    double targetAngleOffset = atan2(target[2], target[0]);
    
    double lift = acos((sqr(targetLength)+sqr(femurLength)-sqr(tibiaLength))/(2.0*targetLength*femurLength));
    liftAngle = targetAngleOffset + lift;
    double kneeBend = acos((sqr(femurLength)+sqr(tibiaLength)-sqr(targetLength))/(2.0*femurLength*tibiaLength));
    kneeAngle = tibiaAngleOffset + kneeBend;
    applyFK();
  }
  void applyWorldIK(const Pose &rootPose, const Vector3d &worldTipTarget)
  {
    applyLocalIK(rootPose.inverseTransformVector(worldTipTarget));
  }
  // works out local tip position from angles
  void applyFK()
  {
    localTipPosition = tipOffset;
    localTipPosition.rotateVector(Quat(Vector3d(0, -kneeAngle, 0)));
    localTipPosition += kneeOffset;
    localTipPosition.rotateVector(Quat(Vector3d(0, liftAngle, 0)));
    localTipPosition += hipOffset;
    localTipPosition.rotateVector(Quat(Vector3d(0, 0, yaw)));
    localTipPosition += rootOffset;
  }
  void init(double startYaw, double startLiftAngle, double startKneeAngle)
  {
    yaw = startYaw;
    liftAngle = startLiftAngle;
    kneeAngle = startKneeAngle;
    hipLength = hipOffset.norm();
    femurLength = kneeOffset.norm();
    femurAngleOffset = atan2(kneeOffset[2], kneeOffset[0]);
    tibiaLength = tipOffset.norm();
    tibiaAngleOffset = atan2(tipOffset[2], tipOffset[0]);
    legLength = hipLength+femurLength+tibiaLength;
    applyFK();
  }
  double hipLength;
  double femurLength;
  double femurAngleOffset;
  double tibiaLength;
  double tibiaAngleOffset;
  double legLength;  
};

// defines the hexapod model
struct Model
{
  Leg legs[3][2]; // front to back, left to right
  Pose rootPose;
  Model()
  {
    for (int l = 0; l<3; l++)
    {
      for (int side = 0; side<2; side++)
      {
        Leg &leg = legs[l][side];
        leg.rootOffset = Vector3d(0.5, 0.5*(double)(1-l), 0);
        leg.hipOffset  = Vector3d(0.05, 0, 0);
        leg.kneeOffset = Vector3d(0.5, 0, 0);
        leg.tipOffset  = Vector3d(1.0, 0, 0);
        leg.onGround = true;
        leg.init(0,0,0);
      }
    }
  }
  
  // move leg tip (normally for swing leg)
  void moveLeg(int leg, int side, const Vector3d &worldTipTarget)
  {
    legs[leg][side].applyWorldIK(worldTipTarget);
  }
  // keeps grounded leg tips stationary, doesn't touch swing leg angles
  void moveBody(const Pose &targetPose)
  {
    for (int l = 0; l<3; l++)
    {
      for (int side = 0; side<2; side++)
      {
        Leg &leg = legs[l][side];
        if (leg.onGround)
          leg.applyWorldIK(targetPose, rootPose.transformVector(leg.localTipPosition));
      }
    }
    rootPose = targetPose; 
  }
};

struct TripodWalk
{
  Model *model;
  Pose targetPose;
  double clearance;
  Vector3d stanceLegYaws;
  Vector3d footSpreadDistances;
  double minFootprintRadius;
  double stanceRadius; // affects turning circle
  Vector3d localStanceTipPositions[3][2];
  
  // Determines the basic stance pose which the hexapod will try to maintain, by finding the largest footprint radius that each leg can achieve for the specified level of clearance
  // clearance- 0 to 1, 1 is vertical legs
  // stanceLegYaws- natural yaw pose per leg
  // minYawLimits- the minimum yaw (or hip) joint limit around centre for each leg
  TripodWalk(Model *model, double clearance, const Vector3d &stanceLegYaws, const Vector3d &minYawLimits) : model(model), clearance(clearance), stanceLegYaws(stanceLegYaws)
  {
    ASSERT(clearance >= 0 && clearance < 1.0);
    minFootprintRadius = 1e10;
    for (int l = 0; l<3; l++)
    {
      double horizontalRange = sqrt(sqr(model->legs[l].legLength) - sqr(clearance*model->legs[l].legLength));
      double theta = minYawLimits[l] - abs(stanceLegYaws[l]);
      double cotanTheta = tan(0.5*pi - theta);
      double rad = solveQuadratic(sqr(cotanTheta), -2.0*horizontalRange, sqr(horizontalRange));
      footSpreadDistances[l] = rad * cotanTheta;
      minFootprintRadius = min(minFootprintRadius, rad);
      for (int side = 0; side<2; side++)
      {
        localStanceTipPositions[l][s] = model->legs[l].rootOffset + footSpreadDistances[l]*Vector3d(cos(stanceLegYaws[l]), sin(stanceLegYaws[l]), -clearance*model->legs[l].legLength);
      }
    }
    stanceRadius = abs(localStanceTipPositions[1][0][1]);
  }

  // curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
  void walk(const Vector3d &localDirection, double curvature)
  {
  }
protected:
  
};