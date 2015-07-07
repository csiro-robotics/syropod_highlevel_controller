#pragma once
#include "../include/simple_hexapod_controller/tripodWalk.h"

Vector3d TripodWalk::LegStepper::getPosition(double liftHeight)
{
  Vector3d strideVec(strideVector[0], strideVector[1], 0);
  if (abs(phase-pi) < pi/2.0) // swing
  {
    Vector3d nodes[4];
    nodes[0] = -strideVec*0.5;
    nodes[3] = strideVec*0.5;
    Vector3d lift(0,0,liftHeight*2.0);
    nodes[1] = nodes[0] + (lift - strideVec)/3.0;
    nodes[2] = nodes[3] + (lift + strideVec)/3.0;
    Vector3d pos = cubicBezier(nodes, (phase - pi/2.0)/pi);
    ASSERT(pos.squaredNorm() < 1000.0);
    return pos;
  }
  else // stance
  {
    // I use a quartic to let the trajectory decelerate, but much flatter stance height than using a quadratic
    double t = 2.0*phase/pi;
    if (t > 1.0)
      t -= 4.0;
    Vector3d pos = strideVec * 0.5 * -t;
    pos[2] = liftHeight*0.5 * (pow(-t, 4.0) - 1.0);
    ASSERT(pos.squaredNorm() < 1000.0);
    return pos;
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
    
    double rad = solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange));
    footSpreadDistances[l] = horizontalRange - rad;
    double footprintDownscale = 0.8; // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    for (int side = 0; side<2; side++)
    {
      localStanceTipPositions[l][side] = model->legs[l][side].rootOffset + footSpreadDistances[l]*Vector3d(cos(stanceLegYaws[l]), sin(stanceLegYaws[l]), 0) + Vector3d(0,0,-bodyClearance*model->legs[l][side].legLength);
      localStanceTipPositions[l][side][0] *= model->legs[l][side].mirrorDir;
    }
  }
  stanceRadius = abs(localStanceTipPositions[1][0][0]);
  int i = 0;
  for (int s = 0; s<2; s++)
  {
    for (int l = 0; l<3; l++)
    {
      legSteppers[l][s].phaseOffset = pi * double((i++)%2); 
      legSteppers[l][s].phase = 2.0*pi; // this ensures that the feet start stepping naturally (don't pop to up position)
      legSteppers[l][s].strideVector = Vector2d(0,0);
    }
  }
  localVelocity = Vector2d(0,0);
  curvature = 0;
  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*model->legs[0][0].legLength);
}


// curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
// bodyOffset is body pose relative to the basic stance pose, note that large offsets may prevent achievable leg positions
void TripodWalk::update(Vector2d localNormalisedVelocity, double newCurvature, const Pose *bodyOffset)
{
  targets.clear();
  double normalSpeed = localNormalisedVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  // this block assures the local velocity and curvature values don't change too quickly
  if (normalSpeed > 0.0 && localVelocity.norm()==0.0) // started walking again
  {
    // reset, and we want to pick the walkPhase closest to its current phase or antiphase...
    if (walkPhase > pi*0.5 && walkPhase < pi*1.5)
      walkPhase = pi;
    else
      walkPhase = 0;
  }
  double maxAcceleration = 0.1;
  double maxCurvatureSpeed = 0.4;
  Vector2d diff = localNormalisedVelocity*minFootprintRadius*2.0*stepFrequency - localVelocity;
  double diffLength = diff.norm();
  if (diffLength > 0.0)
    localVelocity += diff * min(1.0, maxAcceleration*timeDelta/diffLength);
  double speed = localVelocity.norm();
  double dif = newCurvature - curvature;
  if (abs(dif)>0.0)
    curvature += dif * min(1.0, maxCurvatureSpeed*timeDelta/abs(dif));
  
  
  walkPhase += 2.0*pi*stepFrequency*timeDelta;
  if (walkPhase > 2.0*pi)
    walkPhase -= 2.0*pi;
  
  double turningRadius = getTurningRadius(curvature);
  Vector2d localTurningPoint = sign(curvature)* turningRadius * Vector2d(-localVelocity[1], localVelocity[0])/(speed+1e-10); // the point it is turning around
  double angularVelocity = sign(curvature) * speed/(turningRadius + stanceRadius); // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      LegStepper &legStepper = legSteppers[l][s];
      Leg &leg = model->legs[l][s];
      Vector2d toTip = localTurningPoint + Vector2d(leg.localTipPosition[0], leg.localTipPosition[1]);
      legStepper.strideVector = angularVelocity * Vector2d(toTip[1], -toTip[0]) / (2.0*stepFrequency);
      
      double phase = fmod(walkPhase + legStepper.phaseOffset, 2.0*pi);
      if (legStepper.phase == 2.0*pi && phase >= pi && speed>0) // if stopped then do nothing until phase gets reset to 0 (new step)
        localVelocity = Vector2d(1e-10,1e-10); // don't accelerate until stance leg is ready to go. [The use of 1e-10 to mean 'legs starting but not moving yet' isn't great, better to make hasStarted state]
      else if (phase < legStepper.phase && speed == 0) // if just stopped, don't continue leg cycle beyond 2pi
        legStepper.phase = 2.0*pi;
      else
        legStepper.phase = phase; // otherwise follow the step cycle exactly
      Vector3d pos = localStanceTipPositions[l][s] + legStepper.getPosition(stepClearance*leg.legLength);
      if (legStepper.phase < pi*0.5 || legStepper.phase > pi*1.5)
        targets.push_back(pose.transformVector(pos));
      leg.applyLocalIK(pos);
//      targets.push_back(pose.transformVector(leg.localTipPosition));
    }
  }
  
  if (bodyOffset != NULL)
  {
    for (int l = 0; l<3; l++)
      for (int side = 0; side<2; side++)
        model->legs[l][side].applyLocalIK(bodyOffset->inverseTransformVector(model->legs[l][side].localTipPosition), false); // false means we don't update local tip position, as it is needed above in step calculations
  }
  Vector3d turningPoint = pose.transformVector(Vector3d(-localTurningPoint[0], -localTurningPoint[1], 0));
//  targets.push_back(turningPoint);
  pose.position -= turningPoint;
  pose = Pose(Vector3d(0,0,0), Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta))) * pose;
  pose.position += turningPoint;
}
