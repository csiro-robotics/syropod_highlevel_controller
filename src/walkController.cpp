#pragma once
#include "../include/simple_hexapod_controller/walkController.h"

static double stancePhase = 8;          // WAVE: 8	TRIPOD: 1       RIPPLE: 2
static double swingPhase = 1;           // WAVE: 1	TRIPOD: 1       RIPPLE: 1
static double phaseOffset = 1.5;        // WAVE: 1.5    TRIPOD: 1       RIPPLE: 1
static double stanceFuncOrder = 8.0 * stancePhase / swingPhase;
static double heightRatio = 0.25; // The ratio between the positive and negative lift heights (stance/swing)

static double swingStart = stancePhase / 2.0;
static double swingEnd = stancePhase / 2.0 + swingPhase;

static int legSelectionPattern[]  = {0,1,2,0,1,2};  //WAVE: {0,1,2,0,1,2}       TRIPOD: {0,1,2,0,1,2}   RIPPLE: {2,1,0,2,1,0}
static int sideSelectionPattern[] = {0,0,0,1,1,1};  //WAVE: {0,0,0,1,1,1}       TRIPOD: {0,0,0,1,1,1}   RIPPLE: {1,0,1,0,1,0}

static double maxAcceleration = 0.1;
static double maxCurvatureSpeed = 0.4;

static bool isStarting = false;
static bool isMoving = false;
static bool isStopping = false;

static int numLegs = 6;

static double transitionPeriod = swingPhase*0.2;
static double swing0 = swingStart+transitionPeriod*0.5;
static double swing1 = swingEnd  -transitionPeriod*0.5;

// This version deccelerates on approaching the ground during a step, allowing a softer landing.
// as such, the swing phase is made from two time-symmetrical cubics and
// the stance phase is linear with a shorter cubic acceleration prior to lifting off the ground
Vector3d WalkController::LegStepper::updatePosition(double liftHeight, 
                                                    Vector2d localCentreVelocity, 
                                                    double angularVelocity)
{
  double landSpeed = 0.5*liftHeight; // 1 is linear land speed
  Vector3d strideVec(strideVector[0], strideVector[1], 0);
  Vector3d pos;
  
  // Swing Phase
  if (phase > swing0 && phase < swing1)
  {
    // X and Y components of trajectory
    Vector3d nodes[4];
    nodes[0] = currentTipPosition;
    nodes[3] = defaultTipPosition+strideVec*0.5;
    nodes[1] = nodes[0] - strideVec / 3.0; //TO BE FIXED FOR CORRECT CURVE
    nodes[2] = nodes[3] + strideVec / 3.0; //TO BE FIXED FOR CORRECT CURVE
    
    pos = cubicBezier(nodes, (phase-swing0) / (swing1 - swing0));
    
    // Z component of trajectory
    double swingMid = (swing0 + swing1)*0.5;
    double t = 1.0 - abs(phase - swingMid)/(swing1-swingMid);
    
    double a = landSpeed - 2.0*liftHeight;
    double b = liftHeight - landSpeed - a;
    pos[2] = defaultTipPosition[2] + a*t*t*t + b*t*t + landSpeed*t;
    
    ASSERT(pos.squaredNorm() < 1000.0);

    return pos;
  }
  // Stance phase
  else
  {
    double t = phase;
    if (t > (stancePhase + swingPhase)*0.5)
      t -= stancePhase + swingPhase; 
    
    pos = currentTipPosition;
    pos[2] = defaultTipPosition[2];
    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    // Z Component of trajectory         
    double liftOffSpeed = landSpeed * transitionPeriod / (swing1-swing0);
    pos[2] -= liftOffSpeed * 2.0/3.0; // cubic that ends with 0 acceleration gives twice the depth
    // pos[2] -= liftOffSpeed/3.0; // pure cubic
    
    if (abs(t) > swingStart - transitionPeriod*0.5) // transition / launch phase
    {
      t = (abs(t) - (swingStart - transitionPeriod*0.5))/transitionPeriod; // now t is 0-1 range
      // height is a bit different here, we use a linear section then a cubic section to lift off
      pos[2] += liftOffSpeed*t*t - liftOffSpeed*t*t*t/3.0;
      //pos[2] += liftOffSpeed * t*t*t / 3.0; // pure cubic, give smaller depth push
    }
    ASSERT(pos.squaredNorm() < 1000.0);

    return pos;
  }
}

/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by 
 * finding the largest footprint radius that each leg can achieve for the 
 * specified level of clearance.
 * 
 * stepFrequency:	preferred step cycles per second
 * stepClearance:	1 is full leg length, smaller values allow the leg more lateral movement 
 * 					(which is stored as minFootprintRadius)
 * bodyClearance:	0 to 1, 1 is vertical legs. Default calculates best clearance for given leg clearance
 * 
***********************************************************************************************************************/
WalkController::WalkController(Model *model, int gaitType, double stepFrequency, double stepClearance, double bodyClearance, double legSpanScale): 
    model(model), gaitType(gaitType), stepFrequency(stepFrequency), bodyClearance(bodyClearance), walkPhase(0), stepClearance(stepClearance)
{
  ASSERT(stepClearance >= 0 && stepClearance < 1.0);
  
  phaseLength = swingPhase + stancePhase;

  double minKnee = max(0.0, model->minMaxKneeBend[0]);
  double maxHipDrop = min(-model->minMaxHipLift[0], pi/2.0 - atan2(model->legs[0][0].tibiaLength*sin(minKnee), model->legs[0][0].femurLength + model->legs[0][0].tibiaLength*cos(minKnee)));
  
  maximumBodyHeight = model->legs[0][0].femurLength * sin(maxHipDrop) + model->legs[0][0].tibiaLength * sin(maxHipDrop + clamped(pi/2.0 - maxHipDrop, minKnee, model->minMaxKneeBend[1]));
  ASSERT(stepClearance*maximumBodyHeight <= 2.0*model->legs[0][0].femurLength); // impossible to lift this high
  const double stepCurvatureAllowance = 0.7; // dont need full height cylinder (when 1) because the top of the step is rounded

  // If undefined - work out a best value to maximise circular footprint for given step clearance
  if (bodyClearance == -1) 
  {
    // in this case we assume legs have equal characteristics
    bodyClearance = model->legs[0][0].minLegLength/maximumBodyHeight + stepCurvatureAllowance*stepClearance;
    cout << "auto calculating best body clearance: " << bodyClearance << endl; 
  }
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);

  minFootprintRadius = 1e10;

  for (int l = 0; l<3; l++)
  {
    // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
    Leg &leg = model->legs[l][0];
    // downward angle of leg
    double legDrop = atan2(bodyClearance*maximumBodyHeight, leg.maxLegLength);
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
    {	
      horizontalRange = sqrt(sqr(leg.maxLegLength) - sqr(bodyClearance*maximumBodyHeight));
      //horizontalRange*=0.6;
    }
    horizontalRange *= legSpanScale;

    double theta = model->yawLimitAroundStance[l];
    double cotanTheta = tan(0.5*pi - theta);
    rad = min(rad, solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange)));

    // we should also take into account the stepClearance not getting too high for the leg to reach
    double legTipBodyClearance = max(0.0, bodyClearance - stepCurvatureAllowance*stepClearance)*maximumBodyHeight; 
    if (legTipBodyClearance < leg.minLegLength)
    {
      rad = min(rad, (horizontalRange - sqrt(sqr(leg.minLegLength) - sqr(legTipBodyClearance))) / 2.0); // if footprint radius due to lift is smaller due to yaw limits, reduce this minimum radius
    }

    footSpreadDistances[l] = leg.hipLength + horizontalRange - rad;
    double footprintDownscale = 0.8; // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    
    for (int side = 0; side<2; side++)
    {
      localStanceTipPositions[l][side] = model->legs[l][side].rootOffset + footSpreadDistances[l]*Vector3d(cos(model->stanceLegYaws[l]), sin(model->stanceLegYaws[l]), 0) + Vector3d(0,0,-bodyClearance*maximumBodyHeight);
      localStanceTipPositions[l][side][0] *= model->legs[l][side].mirrorDir;
      
      legSteppers[l][side].defaultTipPosition = localStanceTipPositions[l][side];
      legSteppers[l][side].currentTipPosition = legSteppers[l][side].defaultTipPosition;
    }
  }
  // check for overlapping radii
  double minGap = 1e10;
  for (int side = 0; side<2; side++)
  {
    Vector3d posDif = localStanceTipPositions[1][side] - localStanceTipPositions[0][side];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
    posDif = localStanceTipPositions[1][side] - localStanceTipPositions[2][side];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
  }

  cout << "Gap between footprint cylinders: " << minGap << "m" << endl;
  if (minGap < 0.0)
  {
    cout << "Footprint cylinders overlap, reducing footprint radius" << endl;
    minFootprintRadius += minGap*0.5;
  }

  stanceRadius = abs(localStanceTipPositions[1][0][0]);

  for (int i = 0; i<6; i++)
  {	  
    int l = legSelectionPattern[i];
    int s = sideSelectionPattern[i];
    legSteppers[l][s].phaseOffset = fmod(phaseOffset*double(i), phaseLength);
    legSteppers[l][s].phase = 0; // Ensures that feet start stepping naturally and don't pop to up position
    legSteppers[l][s].strideVector = Vector2d(0,0);
  }

  localCentreVelocity = Vector2d(0,0);
  angularVelocity = 0;

  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*maximumBodyHeight);

  for (int l = 0; l<3; l++)
    for (int s = 0; s<2; s++)
      targets.push_back(model->legs[l][s].localTipPosition); // for use in moving to start position
}

/***********************************************************************************************************************
 * curvature:	0 to 1 - where 1 is rotate on the spot and 0.5 rotates around leg stance pos
 * bodyOffset:	Body-pose relative to the basic stance pose - Note: large offsets may prevent achievable leg positions

***********************************************************************************************************************/
void WalkController::update(Vector2d localNormalisedVelocity, double newCurvature, const Pose *bodyOffset)
{
  targets.clear();
  double onGroundRatio = (stancePhase+transitionPeriod)/(stancePhase + swingPhase);
  Vector2d localVelocity = localNormalisedVelocity*minFootprintRadius*stepFrequency/onGroundRatio;

  double normalSpeed = localVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  Vector2d oldLocalCentreVelocity = localCentreVelocity;
  
  isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;
  
  // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
  double newAngularVelocity = newCurvature * normalSpeed/stanceRadius;
  double dif = newAngularVelocity - angularVelocity;

  if (abs(dif)>0.0)
    angularVelocity += dif * min(1.0, maxCurvatureSpeed*timeDelta/abs(dif));

  Vector2d centralVelocity = localVelocity * (1 - abs(newCurvature));
  Vector2d diff = centralVelocity - localCentreVelocity;
  double diffLength = diff.norm();

  if (diffLength > 0.0)
    localCentreVelocity += diff * min(1.0, maxAcceleration*timeDelta/diffLength);
  
  //Iterate master walk phase
  if (isMoving)
    walkPhase = iteratePhase(walkPhase);
  
  //Engage States
  if (!isMoving && normalSpeed && !isStarting)
  {
    isStarting = true;
    isStopping = false;   
  }
  if (isMoving && !normalSpeed && !isStopping)
  {
    isStopping = true;
    isStarting = false;  
  }
  
  //Disengage States
  if (isStarting && targetsNotMet == 0)
  {
    isStarting = false;
    isMoving = true;
    targetsNotMet = numLegs;   
    walkPhase = 0;
  }
  if (isStopping && targetsNotMet == 0)
  {
    isStopping = false;
    isMoving = false;
    targetsNotMet = numLegs;
  }  
  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    { 
      LegStepper &legStepper = legSteppers[l][s];
      Leg &leg = model->legs[l][s];
      
      legStepper.strideVector = onGroundRatio*
          (localCentreVelocity + angularVelocity*Vector2d(leg.localTipPosition[1], -leg.localTipPosition[0]))/
          stepFrequency;
          
      double phase = fmod(walkPhase + legStepper.phaseOffset, phaseLength);
      double targetPhase = 0.0;
      
      if (isStarting)
      {        
        localCentreVelocity = Vector2d(1e-10, 1e-10);
        angularVelocity = 0.0;
        legStepper.strideVector = Vector2d(1e-10, 1e-10);
        
        if (legStepper.phaseOffset < phaseLength/2)
          targetPhase = legStepper.phaseOffset;
        else
          targetPhase = phaseLength - legStepper.phaseOffset;
        
        if (targetReached(legStepper.phase, targetPhase))
        {
          if ((2*l+s) == targetsNotMet-1)
            targetsNotMet--;
        }
        else
          legStepper.phase = iteratePhase(legStepper.phase);
        
      }
      else if (isStopping)
      {   
        if (targetReached(legStepper.phase, targetPhase))
        {
          if ((2*l+s) == targetsNotMet-1)
            targetsNotMet--;          
        }
        else
          legStepper.phase = iteratePhase(legStepper.phase);  
      }
      else if (isMoving)
        legStepper.phase = phase; // otherwise follow the step cycle exactly
      else //else stopped
        legStepper.phase = 0;
      
      double liftHeight = stepClearance*maximumBodyHeight;
      legStepper.currentTipPosition = legStepper.updatePosition(liftHeight, localCentreVelocity, angularVelocity);        
      tipPositions[l][s] = legStepper.currentTipPosition;
      
      if ((legStepper.phase < swingStart+transitionPeriod*0.5) || (legStepper.phase > swingEnd-transitionPeriod*0.5))
        targets.push_back(pose.transformVector(tipPositions[l][s]));
      
      leg.applyLocalIK(tipPositions[l][s]);
    }
  }

  if (bodyOffset != NULL)
  {
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        model->legs[l][s].applyLocalIK(bodyOffset->inverseTransformVector(model->legs[l][s].localTipPosition), false); 
            // false means we don't update local tip position, as it is needed above in step calculations
      }
    }
  }  
  
  model->clampToLimits();
  localCentreAcceleration = (localCentreVelocity - oldLocalCentreVelocity) / timeDelta;
  Vector2d push = localCentreVelocity*timeDelta;
  pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
  pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));
}

double WalkController::iteratePhase(double phase)
{
  return fmod(phase+phaseLength*stepFrequency*timeDelta, phaseLength); 
}

bool WalkController::targetReached(double phase, double targetPhase)
{
  if (phase <= targetPhase &&
      iteratePhase(phase) > targetPhase)
    return true;
  else if (targetPhase == 0 && phase > iteratePhase(phase))
    return true;
  else
    return false;
}
    

// goes from target positions to localStanceTipPositions
bool WalkController::moveToStart()
{
  if (targets.size() == 0)
    return true;
  int i = 0;
  walkPhase = min(walkPhase + stepFrequency*timeDelta/1.0, pi);
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      Leg &leg = model->legs[l][s];     
      Vector3d nodes[4];
      nodes[0] = targets[i++];
      nodes[1] = nodes[0];// + 4.0*Vector3d(0,0,stepClearance*maximumBodyHeight)/3.0;
      nodes[3] = legSteppers[l][s].updatePosition(stepClearance*maximumBodyHeight,localCentreVelocity, angularVelocity);;
      nodes[2] = nodes[3];// + Vector3d(0,0,stepClearance*maximumBodyHeight)/3.0; // ends at one quarter the speed, so it doesn't slam into the ground
      Vector3d pos = cubicBezier(nodes, walkPhase / pi);
      leg.applyLocalIK(pos);
    }
  }
  
  model->clampToLimits();
  if (walkPhase == pi)
  {
    walkPhase = 0.0;
    targets.clear();
    return true;
  }
  return false;
}
