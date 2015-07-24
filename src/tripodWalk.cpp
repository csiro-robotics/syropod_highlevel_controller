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
    pos[2] = liftHeight*0.25 * (pow(-t, 8.0) - 1.0);
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
TripodWalk::TripodWalk(Model *model, double stepFrequency, double stepClearance, double bodyClearance) : 
 model(model), stepFrequency(stepFrequency), bodyClearance(bodyClearance), stepClearance(stepClearance), walkPhase(0)
{
  ASSERT(stepClearance >= 0 && stepClearance < 1.0);
  minFootprintRadius = 1e10;
  double minKnee = max(0.0, model->minMaxKneeBend[0]);
  double legX = model->legs[0][0].tibiaLength*sin(minKnee);
  double legZ = model->legs[0][0].femurLength + model->legs[0][0].tibiaLength*cos(minKnee);
  double maxHipDrop = min(-model->minMaxHipLift[0], pi/2.0 - atan2(legX, legZ));
  double legLength = sqrt(sqr(legX) + sqr(legZ));
  double maximumBodyHeight = model->legs[0][0].femurLength * sin(maxHipDrop) + model->legs[0][0].tibiaLength * sin(maxHipDrop + clamped(pi/2.0 - maxHipDrop, minKnee, model->minMaxKneeBend[1]));
  ASSERT(stepClearance*maximumBodyHeight <= 2.0*model->legs[0][0].femurLength); // impossible to lift this high
  const double stepCurvatureAllowance = 0.7; // dont need full height cylinder (when 1) because the top of the step is rounded
  if (bodyClearance == -1) // if we haven't defined this then lets work out a sort of best value (to maximise circular footprint for given step clearance)
    bodyClearance = model->legs[0][0].getMinLength(model->minMaxKneeBend[1])/maximumBodyHeight + stepCurvatureAllowance*stepClearance; // in this case we assume legs have equal characteristics
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);

  for (int l = 0; l<3; l++)
  {
    // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
    Leg &leg = model->legs[l][0];
    // downward angle of leg
    double legDrop = atan2(bodyClearance*maximumBodyHeight, legLength);
    double horizontalRange = 0;
    double rad = 1e10;
    if (legDrop > -model->minMaxHipLift[0]) // leg can't be straight and touching the ground at bodyClearance
    {
      double extraHeight = bodyClearance*maximumBodyHeight - leg.femurLength * sin(-model->minMaxHipLift[0]);
      ASSERT(extraHeight <= leg.tibiaLength); // this shouldn't be possible with bodyClearance < 1
      rad = sqrt(sqr(leg.tibiaLength) - sqr(extraHeight));
      horizontalRange = leg.femurLength * cos(-model->minMaxHipLift[0]) + rad;
    }
    else
      horizontalRange = sqrt(sqr(legLength) - sqr(bodyClearance*maximumBodyHeight));
    
    double theta = model->yawLimitAroundStance[l];
    double cotanTheta = tan(0.5*pi - theta);
    rad = min(rad, solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange)));
    
    // we should also take into account the stepClearance not getting too high for the leg to reach
    double minLegLength = leg.getMinLength(model->minMaxKneeBend[1]);
    double legTipBodyClearance = max(0.0, bodyClearance - stepCurvatureAllowance*stepClearance)*maximumBodyHeight; 
    if (legTipBodyClearance < minLegLength)
      rad = min(rad, (horizontalRange - sqrt(sqr(minLegLength) - sqr(legTipBodyClearance))) / 2.0); // if footprint radius due to lift is smaller due to yaw limits, reduce this minimum radius
    
    footSpreadDistances[l] = leg.hipLength + horizontalRange - rad;
    double footprintDownscale = 0.8; // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    for (int side = 0; side<2; side++)
    {
      localStanceTipPositions[l][side] = model->legs[l][side].rootOffset + footSpreadDistances[l]*Vector3d(cos(model->stanceLegYaws[l]), sin(model->stanceLegYaws[l]), 0) + Vector3d(0,0,-bodyClearance*maximumBodyHeight);
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
  localCentreAcceleration = Vector2d(0,0);
  angularVelocity = 0;
  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*maximumBodyHeight);
}


// curvature is 0 to 1 so 1 is rotate on the spot, 0.5 rotates around leg stance pos
// bodyOffset is body pose relative to the basic stance pose, note that large offsets may prevent achievable leg positions
void TripodWalk::update(Vector2d localNormalisedVelocity, double newCurvature, const Pose *bodyOffset)
{
  targets.clear();
  Vector2d localVelocity = localNormalisedVelocity*minFootprintRadius*2.0*stepFrequency;
  double normalSpeed = localVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  Vector2d oldLocalCentreVelocity = localCentreVelocity;
  // this block assures the local velocity and curvature values don't change too quickly
  bool isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;
  if (normalSpeed > 0.0 && !isMoving) // started walking again
  {
    // reset, and we want to pick the walkPhase closest to its current phase or antiphase...
    if (walkPhase > pi*0.5 && walkPhase < pi*1.5)
      walkPhase = pi;
    else
      walkPhase = 0;
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
  // clamp angles and alert if a limit has been hit
  for (int l = 0; l<3; l++)
  {
    for (int side = 0; side<2; side++)
    {
      Leg &leg = model->legs[l][side];
      if (leg.yaw - model->stanceLegYaws[l] < -model->yawLimitAroundStance[l])
      {
        leg.yaw = -model->yawLimitAroundStance[l] + model->stanceLegYaws[l];
        cout << "leg " << l << " side " << side << " exceeded yaw limit: " << -model->yawLimitAroundStance[l] + model->stanceLegYaws[l] << endl;
      }
      else if (leg.yaw - model->stanceLegYaws[l] > model->yawLimitAroundStance[l])
      {
        leg.yaw = model->yawLimitAroundStance[l] + model->stanceLegYaws[l];
        cout << "leg " << l << " side " << side << " exceeded yaw limit: " << model->yawLimitAroundStance[l] + model->stanceLegYaws[l] << endl;
      }
      if (leg.liftAngle < model->minMaxHipLift[0])
      {
        leg.liftAngle = model->minMaxHipLift[0];
        cout << "leg " << l << " side " << side << " exceeded hip lift limit: " << model->minMaxHipLift[0] << endl;
      }
      else if (leg.liftAngle > model->minMaxHipLift[1])
      {
        leg.liftAngle = model->minMaxHipLift[1];
        cout << "leg " << l << " side " << side << " exceeded hip lift limit: " << model->minMaxHipLift[1] << endl;
      }
      if (leg.kneeAngle < model->minMaxKneeBend[0])
      {
        leg.kneeAngle = model->minMaxKneeBend[0];
        cout << "leg " << l << " side " << side << " exceeded knee limit: " << model->minMaxKneeBend[0] << endl;
      }
      else if (leg.kneeAngle > model->minMaxKneeBend[1])
      {
        leg.kneeAngle = model->minMaxKneeBend[1];
        cout << "leg " << l << " side " << side << " exceeded knee limit: " << model->minMaxKneeBend[1] << endl;
      }
    }
  }
  localCentreAcceleration = (localCentreVelocity - oldLocalCentreVelocity) / timeDelta;
  Vector2d push = localCentreVelocity*timeDelta;
  pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
  pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));
}
