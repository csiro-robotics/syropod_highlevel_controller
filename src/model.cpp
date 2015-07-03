#pragma once
#include "../include/simple_hexapod_controller/model.h"

void Leg::init(double startYaw, double startLiftAngle, double startKneeAngle)
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

void Leg::applyLocalIK(const Vector3d &tipTarget, bool updateTipPos)
{
  // application of cosine rule
  Vector3d target = tipTarget - rootOffset; // since rootOffset is fixed in root's space
  yaw = atan2(target[1], target[0]);
  Quat quat(Vector3d(0,0,yaw));
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

void Leg::applyFK()
{
  localTipPosition = tipOffset;
  localTipPosition = Quat(Vector3d(0, -kneeAngle, 0)).rotateVector(localTipPosition) + kneeOffset;
  localTipPosition = Quat(Vector3d(0, liftAngle, 0)).rotateVector(localTipPosition) + hipOffset;
  localTipPosition = Quat(Vector3d(0, 0, yaw)).rotateVector(localTipPosition) + rootOffset;
}

// defines the hexapod model
Model::Model()
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

vector<Vector3d> Model::getJointPositions(const Pose &pose)
{
  vector<Vector3d> positions;
  for (int s = 0; s<2; s++)
  {
    for (int l = 0; l<3; l++)
    {
      Leg &leg = legs[l][s];
      Pose transform;
      transform = pose * Pose(leg.rootOffset, Quat(Vector3d(0, 0, leg.yaw)));
      positions.push_back(transform.position);
      transform *= Pose(leg.hipOffset, Quat(Vector3d(0, leg.liftAngle, 0)));
      positions.push_back(transform.position);
      transform *= Pose(leg.kneeOffset, Quat(Vector3d(0, -leg.kneeAngle, 0)));
      positions.push_back(transform.position);
      transform *= Pose(leg.tipOffset, Quat(Vector3d(0, 0, 0)));
      positions.push_back(transform.position);
    }
  }
  return positions;
}

