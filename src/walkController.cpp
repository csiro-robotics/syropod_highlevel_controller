#include "../include/simple_hexapod_controller/walkController.h"

/***********************************************************************************************************************
 * Deccelerates on approaching the ground during a step, allowing a softer landing.
 * as such, the swing phase is made from dual cubic bezier curves and
 * all other phases linear.
***********************************************************************************************************************/
Vector3d WalkController::LegStepper::updatePosition(Leg leg,
                                                    double liftHeight, 
                                                    Vector2d localCentreVelocity, 
                                                    double angularVelocity,
                                                    double stepFrequency,
                                                    double timeDelta)
{
  Vector3d strideVec(strideVector[0], strideVector[1], 0.0);
  Vector3d pos = currentTipPosition;
 
  double swingStart = stancePhase*0.5 + transitionPeriod*0.5;
  double swingEnd = stancePhase*0.5 + swingPhase - transitionPeriod*0.5;
  double swingMid = (swingStart + swingEnd)/2;
   
  /*
  //OLD TIP TRAJECTORY METHOD
  // Swing Phase
  double landSpeed = liftHeight*0.5;
  if (state == SWING)
  {
    
    // X and Y components of trajectory   
    Vector3d nodes[4];
    nodes[0] = currentTipPosition;
    nodes[3] = defaultTipPosition+strideVec*0.5;
    nodes[1] = nodes[0] - strideVec / 6.0;
    nodes[2] = nodes[3] + strideVec / 3.0;
    
    pos = cubicBezier(nodes, (phase-swingStart) / (swingEnd - swingStart));
    
    // Z component of trajectory    
    double t = 1.0 - abs(phase - swingMid)/(swingEnd-swingMid);

    double a = landSpeed - 2.0*liftHeight;
    double b = liftHeight - landSpeed - a;
    double deltaZ = a*t*t*t + b*t*t + landSpeed*t;
    pos[2] = defaultTipPosition[2] + deltaZ;
    
    ASSERT(pos.squaredNorm() < 1000.0);
  */
    
  // Swing Phase
  if (state == SWING)
  {
    Vector3d controlNodesPrimary[4];
    Vector3d controlNodesSecondary[4];
    
    //Save initial tip position at beginning of swing
    if (firstIteration)
    {
      originTipPosition = currentTipPosition;
      firstIteration = false;
    }
    
    //Control nodes for dual cubic bezier curves
    //Primary Bezier curve
    controlNodesPrimary[0] = originTipPosition;         //Set as initial tip position       
    controlNodesPrimary[1] = controlNodesPrimary[0];    //Set equal to node 1 gives zero velocity at liftoff   
    controlNodesPrimary[3] = defaultTipPosition;        //Set equal to default tip position so that max liftheight and transition to 2nd bezier curve occurs at default tip position    
    controlNodesPrimary[3][2] += liftHeight;            //Set Z component of node to liftheight to make liftheight the max z value
    double retrograde = 0.1;                            //Adds retrograde component to tip trajectory
    controlNodesPrimary[2] = originTipPosition-retrograde*((defaultTipPosition + strideVec*0.5) - originTipPosition);
    //ALTERNATIVE OPTION //controlNodesPrimary[2] = (controlNodesPrimary[3] + 2*controlNodesPrimary[0])/3.0; //Set accordingly for constant acceleration
    controlNodesPrimary[2][2] = controlNodesPrimary[3][2];  //Set Z component of node to liftheight to make liftheight the max z value
     
    //Secondary Bezier curve
    controlNodesSecondary[0] = controlNodesPrimary[3];                  //Set to allow continuity between curves 
    controlNodesSecondary[3] = defaultTipPosition + strideVec*0.5;      //Set as target tip position according to stride vector
    controlNodesSecondary[2] = controlNodesSecondary[3];                //Set equal to secondary node 3 gives zero velocity at touchdown
    controlNodesSecondary[1] = 2*controlNodesSecondary[0] - controlNodesPrimary[2];  //Set accordingly so that velocity at end of primary curve equals velocity at begginning of secondary curve
    //ALTERNATIVE OPTION //controlNodesSecondary[1] = (controlNodesSecondary[0] + 2*controlNodesSecondary[2])/3.0; //Set accordingly for constant acceleration
    
    //Vector3d deltaPos;  
    Vector3d Pos;
    double deltaT = (stancePhase + swingPhase)*stepFrequency*timeDelta/(0.5*(swingEnd-swingStart));
    
    //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    double tPrimary;
    double tSecondary;
    if (phase <= swingMid)
    {
      tPrimary = (phase-swingStart)/(swingMid-swingStart)+deltaT;
      //deltaPos = deltaT*cubicBezierDot(controlNodesPrimary, tPrimary);
      Pos = cubicBezier(controlNodesPrimary, tPrimary);
    }
    else if (phase > swingMid)
    {
      tSecondary = (phase-swingMid)/(swingEnd-swingMid)+deltaT;
      //deltaPos = deltaT*cubicBezierDot(controlNodesSecondary, tSecondary);
      Pos = cubicBezier(controlNodesSecondary, tSecondary);
    }
  
    //pos += deltaPos;
    pos = Pos;
    
    //DEBUGGING
    //cout << "TIME: " << tPrimary << ":" << tSecondary << "\t\tORIGIN: " << originTipPosition[0] << ":" << originTipPosition[1] << ":" << originTipPosition[2] << "\t\tPOS: " << pos[0] << ":" << pos[1] << ":" << pos[2] << "\t\tTARGET: " << controlNodesSecondary[3][0] << ":" << controlNodesSecondary[3][1] << ":" << controlNodesSecondary[3][2] << endl; 
    //DEBUGGING
    
  }  
  // Stance phase
  else if (state == STANCE)
  {   
    firstIteration = true;
 
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
      
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    // Z component of trajectory  
    // No Z component change during Stance phase
  }
  else if (state == SWING_TRANSITION)
  {   
    firstIteration = true;
    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    // Z component of trajectory  
    //TBD
  }  
  else if (state == STANCE_TRANSITION)
  {    
    firstIteration = true;
    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    // Z component of trajectory  
    //TBD
  }  
  else if (state == TOUCHDOWN_CORRECTION)
  {    
    firstIteration = true;
    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    //Z Component of Trajectory
    //Early touchdown requires no height correction
  }
  else if (state == LIFTOFF_CORRECTION)
  {
    firstIteration = true;
    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*timeDelta;
    pos[0] += deltaPos[0];
    pos[1] += deltaPos[1];
    
    // Z Component of Trajectory
    pos[2] -= 0.001;    //TBD REFINEMENT
  }
  
  ASSERT(pos.squaredNorm() < 1000.0);
  return pos;
}

/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by 
 * finding the largest footprint radius that each leg can achieve for the 
 * specified level of clearance.
***********************************************************************************************************************/
WalkController::WalkController(Model *model, Parameters p): model(model), params(p)
{ 
  stepFrequency = params.stepFrequency;
  stepClearance = params.stepClearance;
  bodyClearance = params.bodyClearance;
  timeDelta = params.timeDelta;
  
  ASSERT(stepClearance >= 0 && stepClearance < 1.0);

  double minKnee = max(0.0, model->minMaxKneeBend[0]);
  double maxHipDrop = min(-model->minMaxHipLift[0], pi/2.0 - 
    atan2(model->legs[0][0].tibiaLength*sin(minKnee), 
          model->legs[0][0].femurLength + model->legs[0][0].tibiaLength*cos(minKnee)));
  
  maximumBodyHeight = model->legs[0][0].femurLength * sin(maxHipDrop) + model->legs[0][0].tibiaLength * 
    sin(maxHipDrop + clamped(pi/2.0 - maxHipDrop, minKnee, model->minMaxKneeBend[1]));
    
  ASSERT(stepClearance*maximumBodyHeight <= 2.0*model->legs[0][0].femurLength); // impossible to lift this high
 
  // If undefined - work out a best value to maximise circular footprint for given step clearance
  if (bodyClearance == -1) 
  {
    // in this case we assume legs have equal characteristics
    bodyClearance = model->legs[0][0].minLegLength/maximumBodyHeight + params.stepCurvatureAllowance*stepClearance;
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
    horizontalRange *= params.legSpanScale;

    double theta = model->yawLimitAroundStance[l];
    double cotanTheta = tan(0.5*pi - theta);
    rad = min(rad, solveQuadratic(sqr(cotanTheta), 2.0*horizontalRange, -sqr(horizontalRange)));
    ASSERT(rad > 0.0); // cannot have negative radius

    // we should also take into account the stepClearance not getting too high for the leg to reach
    double legTipBodyClearance = max(0.0, bodyClearance-params.stepCurvatureAllowance*stepClearance)*maximumBodyHeight; 
    
    // if footprint radius due to lift is smaller due to yaw limits, reduce this minimum radius
    if (legTipBodyClearance < leg.minLegLength)
      rad = min(rad, (horizontalRange - sqrt(sqr(leg.minLegLength) - sqr(legTipBodyClearance))) / 2.0); 
    ASSERT(rad > 0.0); // cannot have negative radius, step height is too high to allow any footprint

    footSpreadDistances[l] = leg.hipLength + horizontalRange - rad;
    
    // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    double footprintDownscale = 0.8; 
    
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    
    for (int side = 0; side<2; side++)
    {
      identityTipPositions[l][side] = model->legs[l][side].rootOffset + 
        footSpreadDistances[l]*Vector3d(cos(model->stanceLegYaws[l]), sin(model->stanceLegYaws[l]), 0) + 
          Vector3d(0,0,-bodyClearance*maximumBodyHeight);
          
      identityTipPositions[l][side][0] *= model->legs[l][side].mirrorDir;
      
      legSteppers[l][side].defaultTipPosition = identityTipPositions[l][side];
      legSteppers[l][side].currentTipPosition = identityTipPositions[l][side];
    }
  }
  // check for overlapping radii
  double minGap = 1e10;
  for (int side = 0; side<2; side++)
  {
    Vector3d posDif = identityTipPositions[1][side] - identityTipPositions[0][side];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
    posDif = identityTipPositions[1][side] - identityTipPositions[2][side];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
  }

  if (minGap < 0.0)
    minFootprintRadius += minGap*0.5;

  stanceRadius = abs(identityTipPositions[1][0][0]);

  for (int i = 0; i<6; i++)
  {	  
    int l = params.legSelectionPattern[i];
    int s = params.sideSelectionPattern[i];
    double phaseLength = params.stancePhase + params.swingPhase;
    legSteppers[l][s].phaseOffset = fmod(params.phaseOffset*double(i), phaseLength);
    legSteppers[l][s].phase = 0; // Ensures that feet start stepping naturally and don't pop to up position
    legSteppers[l][s].strideVector = Vector2d(0,0);
    legSteppers[l][s].stancePhase = params.stancePhase;
    legSteppers[l][s].swingPhase = params.swingPhase;
    legSteppers[l][s].transitionPeriod = params.transitionPeriod;
  }

  localCentreVelocity = Vector2d(0,0);
  angularVelocity = 0;

  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*maximumBodyHeight);

  //DEBUGGING
  for (int l = 0; l<3; l++)
    for (int s = 0; s<2; s++)
      targets.push_back(model->legs[l][s].localTipPosition);
  //DEBUGGING
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines 
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/
void WalkController::updateWalk(Vector2d localNormalisedVelocity, double newCurvature, double stepFrequencyMultiplier)
{
  //update stepFrequency
  stepFrequency = params.stepFrequency*stepFrequencyMultiplier;
  
  double onGroundRatio = (params.stancePhase+params.transitionPeriod)/(params.stancePhase + params.swingPhase);
  
  Vector2d localVelocity = localNormalisedVelocity*minFootprintRadius*stepFrequency/onGroundRatio;

  double normalSpeed = localVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  Vector2d oldLocalCentreVelocity = localCentreVelocity;
  
  bool isMoving = localCentreVelocity.squaredNorm() + sqr(angularVelocity) > 0.0;
  
  // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
  double newAngularVelocity = newCurvature * normalSpeed/stanceRadius;
  double dif = newAngularVelocity - angularVelocity;

  if (abs(dif)>0.0)
    angularVelocity += dif * min(1.0, params.maxCurvatureSpeed*timeDelta/abs(dif));

  Vector2d centralVelocity = localVelocity * (1 - abs(newCurvature));
  Vector2d diff = centralVelocity - localCentreVelocity;
  double diffLength = diff.norm();

  if (diffLength > 0.0)
    localCentreVelocity += diff * min(1.0, params.maxAcceleration*timeDelta/diffLength); 
  
  
  //State transitions for robot state machine.
  // State transition: STOPPED->STARTING
  if (state == STOPPED && !isMoving && normalSpeed)
  {
    state = STARTING;
    //targetsNotMet = 0;
  }
  
  // State transition: STARTING->MOVING
  if (state == STARTING && targetsNotMet == 0)
  {
    targetsNotMet = NUM_LEGS;
    for (int l = 0; l<3; l++)
      for (int s = 0; s<2; s++)
        legSteppers[l][s].phase = legSteppers[l][s].phaseOffset;
    state = MOVING;
  }
  
  // State transition: MOVING->STOPPING
  if (state == MOVING && isMoving && !normalSpeed)
    state = STOPPING;
  
  // State transition: STOPPING->STOPPED
  if (state == STOPPING && targetsNotMet == 0)
  {
    targetsNotMet = NUM_LEGS;
    state = STOPPED;
  }  
   
  //Robot State Machine
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    { 
      LegStepper &legStepper = legSteppers[l][s];
      Leg &leg = model->legs[l][s];
      
      legStepper.strideVector = onGroundRatio*
          (localCentreVelocity + angularVelocity*Vector2d(leg.localTipPosition[1], -leg.localTipPosition[0]))/
          stepFrequency;
       
      double phaseLength = params.stancePhase + params.swingPhase;
      double targetPhase = 0.0;
      
      if (state == STARTING)
      {    
        localCentreVelocity = Vector2d(1e-10, 1e-10);
        angularVelocity = 1e-10;
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
      else if (state == STOPPING)
      {   
        localCentreVelocity = Vector2d(1e-10, 1e-10);
        angularVelocity = 1e-10;
        legStepper.strideVector = Vector2d(1e-10, 1e-10);
        
        if (targetReached(legStepper.phase, targetPhase))
        {
          if ((2*l+s) == targetsNotMet-1)
            targetsNotMet--;          
        }
        else
          legStepper.phase = iteratePhase(legStepper.phase);  
      }
      else if (state == MOVING)
      {
          legStepper.phase = iteratePhase(legStepper.phase);
      }
      else if (state == STOPPED)
      {
        legStepper.phase = 0;
        localCentreVelocity = Vector2d(0, 0);
        angularVelocity = 0.0;
        targets.clear();//DEBUGGING     
        legStepper.strideVector = Vector2d(0, 0);
      } 
    }
  } 
  
  //Leg State Machine
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    { 
      LegStepper &legStepper = legSteppers[l][s];
      
      double stanceEnd = legStepper.stancePhase*0.5;
      double stanceStart = legStepper.stancePhase*0.5 + legStepper.swingPhase;
      double swingStart = stanceEnd + legStepper.transitionPeriod*0.5;
      double swingEnd = stanceStart - legStepper.transitionPeriod*0.5;
       
      if (legStepper.phase > stanceEnd && legStepper.phase < swingStart)
      {        
        legStepper.state = SWING_TRANSITION;
      }
      else if (legStepper.phase > swingStart && legStepper.phase < swingEnd)
      {
        //DEBUGGING
        if (s==0 && l==0 && legStepper.state != SWING)
          staticTargets.clear();
        //DEBUGGING
        legStepper.state = SWING; 
        if (params.legStateCorrection)
        {
          if (legStepper.tipTouchdown)
            legStepper.state = TOUCHDOWN_CORRECTION;
        }          
      }
      else if (legStepper.phase > swingEnd && legStepper.phase < stanceStart)
      {
        legStepper.state = STANCE_TRANSITION;
      }
      else
      {        
        legStepper.state = STANCE; 
        if (params.legStateCorrection)
        {
          if (!legStepper.tipTouchdown)
            legStepper.state = LIFTOFF_CORRECTION;     
        }        
      }
    }
  }
   
  //Update tip positions and apply inverse kinematics
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {  
      LegStepper &legStepper = legSteppers[l][s];
      Leg &leg = model->legs[l][s];
      
      if (leg.state == WALKING)
      {
        //Revise default and current tip positions from stanceTipPosition due to change in pose
        Vector3d tipOffset = legStepper.defaultTipPosition - legStepper.currentTipPosition;
        legStepper.defaultTipPosition = leg.stanceTipPosition;
        legStepper.currentTipPosition = legStepper.defaultTipPosition - tipOffset;
        
        double liftHeight = stepClearance*maximumBodyHeight;
        legStepper.currentTipPosition = legStepper.updatePosition(leg, liftHeight, 
                                                                  localCentreVelocity, 
                                                                  angularVelocity, 
                                                                  stepFrequency, 
                                                                  timeDelta); 
        leg.applyLocalIK(legStepper.currentTipPosition); 
      }
      
      //DEBUGGING
      //staticTargets.push_back(model->legs[0][0].localTipPosition);
      //targets.push_back(pose.transformVector(leg.localTipPosition)); 
      //DEBUGGING
    }
  }  
  
  model->clampToLimits();  
  localCentreAcceleration = (localCentreVelocity - oldLocalCentreVelocity) / timeDelta;
  
  //DEBUGGING
  Vector2d push = localCentreVelocity*timeDelta;
  pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
  pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));
  //DEBUGGING
}

/***********************************************************************************************************************
 * Iterates given phase based on timeDelta and stepFrequency
***********************************************************************************************************************/
double WalkController::iteratePhase(double phase)
{
  double phaseLength = params.stancePhase + params.swingPhase;
  return fmod(phase+phaseLength*stepFrequency*timeDelta, phaseLength); 
}

/***********************************************************************************************************************
 * Checks whether given phase is equivalent to targetPhase and thus reached target phase
 * (needed since phase == targetPhase does not work)
***********************************************************************************************************************/
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

/***********************************************************************************************************************
***********************************************************************************************************************/
