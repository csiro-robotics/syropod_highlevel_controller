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
    Vector3d lift(0,0,liftHeight*4.0);
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
    pos[2] = liftHeight*0.25 * (pow(-t, 16.0) - 1.0);
    ASSERT(pos.squaredNorm() < 1000.0);
    return pos;
  }
}
  
// Determines the basic stance pose which the hexapod will try to maintain, by finding the largest footprint radius that each leg can achieve for the specified level of clearance
// stepFrequency- preferred step cycles per second
// stepClearance- 1 is full leg length, smaller values allow the leg more lateral movement (which is stored as minFootprintRadius)
// stanceLegYaws- natural yaw pose per leg
// minYawLimits- the minimum yaw (or hip) joint limit around centre for each leg
// maximumKneeBend- in radians, 0 can't bend knee at all
// bodyClearance- 0 to 1, 1 is vertical legs. Default calculates best clearance for given leg clearance
TripodWalk::TripodWalk(Model *model, double stepFrequency, double stepClearance, const Vector3d &stanceLegYaws, const Vector3d &yawLimitAroundStance, double maximumKneeBend, double bodyClearance) : 
 model(model), stepFrequency(stepFrequency), bodyClearance(bodyClearance), stepClearance(stepClearance), stanceLegYaws(stanceLegYaws), walkPhase(0)
{
  ASSERT(stepClearance >= 0 && stepClearance < 1.0);
  ASSERT(stepClearance <= 2.0*model->legs[0][0].femurLength / model->legs[0][0].legLength); // impossible to lift this high
  minFootprintRadius = 1e10;
  const double stepCurvatureAllowance = 0.7; // dont need full height cylinder (when 1) because the top of the step is rounded
  if (bodyClearance == -1) // if we haven't defined this then lets work out a sort of best value (to maximise circular footprint for given step clearance)
    bodyClearance = model->legs[0][0].getMinLength(maximumKneeBend)/model->legs[0][0].legLength + stepCurvatureAllowance*stepClearance; // in this case we assume legs have equal characteristics
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);

  for (int l = 0; l<3; l++)
  {
    // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
    Leg &leg = model->legs[l][0];
    double horizontalRange = sqrt(sqr(leg.legLength) - sqr(bodyClearance*leg.legLength));
    double theta = yawLimitAroundStance[l];
    double cotanTheta = tan(0.5*pi - theta);
    double rad = solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange));
    
    // we should also take into account the stepClearance not getting too high for the leg to reach
    double minLegLength = leg.getMinLength(maximumKneeBend);
    double legTipBodyClearance = max(0.0, bodyClearance - stepCurvatureAllowance*stepClearance)*leg.legLength; 
    if (legTipBodyClearance < minLegLength)
    {
      double liftRad = (horizontalRange - sqrt(sqr(minLegLength) - sqr(legTipBodyClearance))) / 2.0;
      if (liftRad < rad) // footprint radius due to lift is smaller than due to yaw limits, so reduce this minimum radius
        rad = liftRad;
    }
    
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
  localCentreVelocity = Vector2d(0,0);
  angularVelocity = 0;
  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*model->legs[0][0].legLength);
  stopped = true;
  started = false;
}


// curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
// bodyOffset is body pose relative to the basic stance pose, note that large offsets may prevent achievable leg positions
void TripodWalk::update(Vector2d localNormalisedVelocity, double newCurvature, const Pose *bodyOffset)
{
  targets.clear();
  Vector2d localVelocity = localNormalisedVelocity*minFootprintRadius*2.0*stepFrequency;
  double normalSpeed = localVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  // this block assures the local velocity and curvature values don't change too quickly
  bool isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;
  if (normalSpeed > 0.0 && !isMoving) // started walking again
  {
    // reset, and we want to pick the walkPhase closest to its current phase or antiphase...
    if (walkPhase > pi)
      walkPhase = pi*1.5;
    else
      walkPhase = pi*0.5;
  }

  {
    double maxAcceleration = 0.1;
    double maxCurvatureSpeed = 0.4;
    double newAngularVelocity = newCurvature * normalSpeed/stanceRadius; // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
    double dif = newAngularVelocity - angularVelocity;
    if (abs(dif)>0.0)
      angularVelocity += dif * min(1.0, maxCurvatureSpeed*timeDelta/abs(dif));
    Vector2d centralVelocity = localVelocity * (1 - abs(newCurvature));
    Vector2d diff = centralVelocity - localCentreVelocity;
    double diffLength = diff.norm();
    if (diffLength > 0.0)
      localCentreVelocity += diff * min(1.0, maxAcceleration*timeDelta/diffLength);
  }
  
  walkPhase += 2.0*pi*stepFrequency*timeDelta;
  if (walkPhase > 2.0*pi)
    walkPhase -= 2.0*pi;
  isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;

  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      LegStepper &legStepper = legSteppers[l][s];
      Leg &leg = model->legs[l][s];
      legStepper.strideVector = (localCentreVelocity + angularVelocity * Vector2d(leg.localTipPosition[1], -leg.localTipPosition[0])) / (2.0*stepFrequency);
      
      double phase = fmod(walkPhase + legStepper.phaseOffset, 2.0*pi);
      if (legStepper.phase == 2.0*pi && phase >= pi && normalSpeed>0) // if stopped then do nothing until phase gets reset to 0 (new step)
      {
        localCentreVelocity = Vector2d(1e-10, 1e-10);
        angularVelocity = 1e-10;
      }
      else if (phase < legStepper.phase && !isMoving) // if just stopped, don't continue leg cycle beyond 2pi
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
  Vector2d push = localCentreVelocity*timeDelta;
  pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
  pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));
}
