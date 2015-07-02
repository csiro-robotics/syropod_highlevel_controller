#pragma once
#include "standardIncludes.h"
#include "quat.h"
#include <../../../bullet3/btgui/lua-5.2.3/src/llimits.h>
#include <Eigen/src/Core/Matrix.h>
#include <boost/concept_check.hpp>
#define timeDelta (1.0/50.0)

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
  
//  bool onGround;
  Vector3d localTipPosition; // relative to root

  // sets angles to reach local position relative to root
  void applyLocalIK(const Vector3d &tipTarget, bool updateTipPos = true)
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
    if (updateTipPos)
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
   //     leg.onGround = true;
        leg.init(0,0,0);
      }
    }
  }
/*  
  Pose rootPose;
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
  */
};

struct TripodWalk
{
  Model *model;
  Pose targetPose;
  double stepFrequency;
  double bodyClearance;
  double stepClearance;
  Vector3d stanceLegYaws;
  Vector3d footSpreadDistances;
  double minFootprintRadius;
  double stanceRadius; // affects turning circle
  Vector3d localStanceTipPositions[3][2];
  Vector2d localVelocity;
  double curvature;
  struct LegStepper
  {
    double phase; // 0 to 2pi
    Vector2d velocity; // length gives ground velocity
    Vector2d strideVector; // length gives stride length
    
    Vector3d getPosition(double liftHeight)
    {
      Vector3d strideVec(strideVector[0], strideVector[1], 0);
      if (abs(phase) < pi/2.0) // stance
      {
        // I use a quartic to let the trajectory decelerate, but much flatter stance height than using a quadratic
        double t = phase*2.0/pi;
        Vector3d pos = strideVec * 0.5 * t;
        pos[2] = liftHeight*0.5 * (pow(t, 4.0) - 1.0);
      }
      else // swing
      {
        Vector3d nodes[4];
        nodes[0] = -strideVec*0.5;
        nodes[3] = strideVec*0.5;
        Vector3d lift(0,0,liftHeight*2.0);
        nodes[1] = nodes[0] + (lift - strideVec)/3.0;
        nodes[2] = nodes[0] + (lift + strideVec)/3.0;
        return cubicBezier(nodes, (phase - pi/2.0)/pi);
      }
    }
  } legSteppers[3][2];
  
  // Determines the basic stance pose which the hexapod will try to maintain, by finding the largest footprint radius that each leg can achieve for the specified level of clearance
  // stepFrequency- preferred step cycles per second
  // bodyClearance, stepClearance- 0 to 1, 1 is vertical legs
  // stanceLegYaws- natural yaw pose per leg
  // minYawLimits- the minimum yaw (or hip) joint limit around centre for each leg
  TripodWalk(Model *model, double stepFrequency, double bodyClearance, double stepClearance, const Vector3d &stanceLegYaws, const Vector3d &minYawLimits) : model(model), stepFrequency(stepFrequency), bodyClearance(bodyClearance), stepClearance(stepClearance), stanceLegYaws(stanceLegYaws)
  {
    ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);
    ASSERT(stepClearance >= 0 && stepClearance < 1.0);
    minFootprintRadius = 1e10;
    for (int l = 0; l<3; l++)
    {
      double horizontalRange = sqrt(sqr(model->legs[l].legLength) - sqr(bodyClearance*model->legs[l].legLength));
      double theta = minYawLimits[l] - abs(stanceLegYaws[l]);
      double cotanTheta = tan(0.5*pi - theta);
      double rad = solveQuadratic(sqr(cotanTheta), -2.0*horizontalRange, sqr(horizontalRange));
      footSpreadDistances[l] = rad * cotanTheta;
      double footprintDownscale = 0.8; // this is because the step cycle exceeds the ground footprint in order to maintain velocity
      minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
      for (int side = 0; side<2; side++)
      {
        localStanceTipPositions[l][s] = model->legs[l].rootOffset + footSpreadDistances[l]*Vector3d(cos(stanceLegYaws[l]), sin(stanceLegYaws[l]), -bodyClearance*model->legs[l].legLength);
      }
    }
    stanceRadius = abs(localStanceTipPositions[1][0][1]);
    int i = 0;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        legSteppers[l][s].phase = pi * double((i++)%2); 
        legSteppers[l][s].velocity = Vector3d(0,0,0);
      }
    }
    localVelocity = Vector2d(0,0);
    curvature = 0;
  }

  
  // curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
  // bodyOffset is body pose relative to the basic stance pose, note that large offsets may prevent achievable leg positions
  void walk(Vector2d newLocalVelocity, double newCurvature, const Pose *bodyOffset = NULL)
  {
    // this block assures the local velocity and curvature values don't change too quickly
    double maxAcceleration = 3.0;
    double maxCurvatureSpeed = 1.0;
    Vector3d diff = localVelocity - newLocalVelocity;
    double diffLength = diff.norm();
    localVelocity = newLocalVelocity + diff * max(0, (diffLength-maxAcceleration*timeDelta)/diffLength);
    double dif = curvature - newCurvature;
    curvature = newCurvature + dif * max(0, (abs(dif)-maxCurvatureSpeed*timeDelta)/abs(dif));
    
    double speed = localVelocity.norm();
    
    // max out the walk speed to what is achievable with stepFrequency. Later we could increase step frequency instead
    double stride = speed / (2.0*stepFrequency);
    if (stride > minFootprintRadius)
    {
      localVelocity *= minFootprintRadius / stride;
      speed *= minFootprintRadius / stride;
    }
    
    double turningRadius = getTurningRadius(curvature);
    Vector2d turningPoint = turningRadius * Vector2d(-localVelocity[1], localVelocity[0])/speed; // the point it is turning around
    double angularVelocity = speed/(turningRadius + stanceRadius); // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
    
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        LegStepper &legStepper = legSteppers[l][s];
        Leg &leg = model->legs[s][l];
        Vector2d toTip = turningPoint + Vector2d(leg.localTipPosition[0], leg.localTipPosition[1]);
        legStepper.velocity = angularVelocity * Vector2d(-toTip[1], toTip[0]) / (2.0*stepFrequency);
        
        legStepper.phase += 2.0*pi*stepFrequency*timeDelta;
        if (legStepper.phase > 2.0*pi)
          legStepper.phase -= 2.0*pi;
        leg.applyLocalIK(localStanceTipPositions[l][s] + legStepper.getPosition(stepClearance*leg.legLength));
      }
    }
    
    if (bodyOffset != NULL)
    {
      for (int l = 0; l<3; l++)
        for (int side = 0; side<2; side++)
          model->legs[l][side].applyLocalIK(bodyOffset->inverseTransformVector(leg.localTipPosition), false); // false means we don't update local tip position, as it is needed above in step calculations
    }
  }
protected:
  double getTurningRadius(double curvature)
  {
    return (stanceRadius / max(0.0001, curvature)) - stanceRadius;
  }
};