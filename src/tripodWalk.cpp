#pragma once
#include "../include/simple_hexapod_controller/tripodWalk.h"

Vector3d TripodWalk::LegStepper::getPosition(double liftHeight)
{
  Vector3d strideVec(strideVector[0], strideVector[1], 0);
  if (abs(phase) < pi/2.0) // stance
  {
    // I use a quartic to let the trajectory decelerate, but much flatter stance height than using a quadratic
    double t = phase*2.0/pi;
    Vector3d pos = strideVec * 0.5 * t;
    pos[2] = liftHeight*0.5 * (pow(t, 4.0) - 1.0);
    return pos;
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
  
// Determines the basic stance pose which the hexapod will try to maintain, by finding the largest footprint radius that each leg can achieve for the specified level of clearance
// stepFrequency- preferred step cycles per second
// bodyClearance, stepClearance- 0 to 1, 1 is vertical legs
// stanceLegYaws- natural yaw pose per leg
// minYawLimits- the minimum yaw (or hip) joint limit around centre for each leg
TripodWalk::TripodWalk(Model *model, double stepFrequency, double bodyClearance, double stepClearance, const Vector3d &stanceLegYaws, const Vector3d &minYawLimits) : 
 model(model), stepFrequency(stepFrequency), bodyClearance(bodyClearance), stepClearance(stepClearance), stanceLegYaws(stanceLegYaws), walkPhase(0)
{
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);
  ASSERT(stepClearance >= 0 && stepClearance < 1.0);
  minFootprintRadius = 1e10;
  for (int l = 0; l<3; l++)
  {
    double horizontalRange = sqrt(sqr(model->legs[l][0].legLength) - sqr(bodyClearance*model->legs[l][0].legLength));
    double theta = minYawLimits[l] - abs(stanceLegYaws[l]);
    double cotanTheta = tan(0.5*pi - theta);
    double rad = solveQuadratic(sqr(cotanTheta), -2.0*horizontalRange, sqr(horizontalRange));
    footSpreadDistances[l] = rad * cotanTheta;
    double footprintDownscale = 0.8; // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    for (int side = 0; side<2; side++)
    {
      localStanceTipPositions[l][side] = model->legs[l][side].rootOffset + footSpreadDistances[l]*Vector3d(cos(stanceLegYaws[l]), sin(stanceLegYaws[l]), -bodyClearance*model->legs[l][side].legLength);
    }
  }
  stanceRadius = abs(localStanceTipPositions[1][0][1]);
  int i = 0;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      legSteppers[l][s].phaseOffset = pi * double((i++)%2); 
      legSteppers[l][s].phase = 2.0*pi; // this ensures that the feet start stepping naturally (don't pop to up position)
      legSteppers[l][s].strideVector = Vector2d(0,0);
    }
  }
  localVelocity = Vector2d(0,0);
  curvature = 0;
}


// curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
// bodyOffset is body pose relative to the basic stance pose, note that large offsets may prevent achievable leg positions
void TripodWalk::update(Vector2d newLocalVelocity, double newCurvature, const Pose *bodyOffset)
{
  // this block assures the local velocity and curvature values don't change too quickly
  if (newLocalVelocity.norm() > 0.0 && localVelocity.norm()==0.0) // started walking again
  {
    // reset, and we want to pick the walkPhase closest to its current phase or antiphase...
    if (walkPhase > pi*0.5 && walkPhase < pi*1.5)
      walkPhase = pi;
    else
      walkPhase = 0;
  }
  double maxAcceleration = 3.0;
  double maxCurvatureSpeed = 1.0;
  Vector2d diff = localVelocity - newLocalVelocity;
  double diffLength = diff.norm();
  localVelocity = newLocalVelocity + diff * max(0.0, (diffLength-maxAcceleration*timeDelta)/diffLength);
  double dif = curvature - newCurvature;
  curvature = newCurvature + dif * max(0.0, (abs(dif)-maxCurvatureSpeed*timeDelta)/abs(dif));
  
  double speed = localVelocity.norm();
  
  // max out the walk speed to what is achievable with stepFrequency. Later we could increase step frequency instead
  double stride = speed / (2.0*stepFrequency);
  if (stride > minFootprintRadius)
  {
    localVelocity *= minFootprintRadius / stride;
    speed *= minFootprintRadius / stride;
  }
  
  walkPhase += 2.0*pi*stepFrequency*timeDelta;
  if (walkPhase > 2.0*pi)
    walkPhase -= 2.0*pi;
  
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
      legStepper.strideVector = angularVelocity * Vector2d(-toTip[1], toTip[0]) / (2.0*stepFrequency);
      
      double phase = fmod(walkPhase + legStepper.phaseOffset, 2.0*pi);
      if (legStepper.phase == 2.0*pi && phase >= pi) // if stopped then do nothing until phase gets reset to 0 (new step)
      {
      }
      else if (phase < legStepper.phase && speed == 0) // if just stopped, don't continue leg cycle beyond 2pi
        legStepper.phase = 2.0*pi;
      else
        legStepper.phase = phase; // otherwise follow the step cycle exactly
      leg.applyLocalIK(localStanceTipPositions[l][s] + legStepper.getPosition(stepClearance*leg.legLength));
    }
  }
  
  if (bodyOffset != NULL)
  {
    for (int l = 0; l<3; l++)
      for (int side = 0; side<2; side++)
        model->legs[l][side].applyLocalIK(bodyOffset->inverseTransformVector(model->legs[l][side].localTipPosition), false); // false means we don't update local tip position, as it is needed above in step calculations
  }
}
