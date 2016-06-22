#include "../include/simple_hexapod_controller/walkController.h"

/***********************************************************************************************************************
 * Deccelerates on approaching the ground during a step, allowing a softer landing.
 * as such, the swing phase is made from dual cubic bezier curves and
 * all other phases linear.
***********************************************************************************************************************/
void WalkController::LegStepper::updateSwingPos(Vector3d *pos)
{  
  Vector3d controlNodesPrimary[4];
  Vector3d controlNodesSecondary[4];
  
  double liftHeight = walker->stepClearance*walker->maximumBodyHeight;
  Vector3d strideVec = Vector3d(strideVector[0], strideVector[1], 0.0);
  
  int iteration = phase-walker->swingStart+1;
  double swingLength = walker->swingEnd - walker->swingStart;
  
  //Save initial tip position at beginning of swing
  if (iteration == 1)
  {
    originTipPosition = *pos;
  }
  
  int numIterations = roundToInt((swingLength/walker->phaseLength)/(walker->stepFrequency*walker->timeDelta)/2.0)*2.0;  //Ensure compatible number of iterations 
  double deltaT = 1.0/numIterations;
       
  //Control nodes for dual cubic bezier curves
  controlNodesPrimary[0] = originTipPosition;         //Set as initial tip position       
  controlNodesPrimary[1] = controlNodesPrimary[0];    //Set equal to node 1 gives zero velocity at liftoff   
  controlNodesPrimary[3] = defaultTipPosition;        //Set equal to default tip position so that max liftheight and transition to 2nd bezier curve occurs at default tip position    
  controlNodesPrimary[3][2] = defaultTipPosition[2] + liftHeight;            //Set Z component of node to liftheight to make liftheight the max z value

  controlNodesSecondary[0] = controlNodesPrimary[3];                  //Set for position continuity between curves (C0 Smoothness)
  controlNodesSecondary[3] = defaultTipPosition + strideVec*0.5;   //Set as target tip position according to stride vector
  controlNodesSecondary[2] = controlNodesSecondary[3];                //Set equal to secondary node 3 gives zero velocity at touchdown
  
  double retrograde = 0.0;                            //Adds retrograde component to tip trajectory (TUNABLE)
  controlNodesPrimary[2] = controlNodesPrimary[0]-retrograde*(controlNodesSecondary[3] - controlNodesPrimary[0]); //Set for more control of trajectory (only C1 smooth)
  //ALTERNATIVE OPTION //controlNodesPrimary[2] = controlNodesSecondary[0] + (controlNodesPrimary[1] - controlNodesSecondary[2])/4.0; ///Set for acceleration continuity between curves (C2 Smoothness)   
  controlNodesPrimary[2][2] = defaultTipPosition[2] + liftHeight; //Set Z component of node to liftheight to make liftheight the max z value
  
  controlNodesSecondary[1] = 2*controlNodesSecondary[0] - controlNodesPrimary[2];  //Set for velocity continuity between curves (C1 Smoothness)
  controlNodesSecondary[1][2] = defaultTipPosition[2] + liftHeight; //Set Z component of node to liftheight to make liftheight the max z value
  
  Vector3d deltaPos;
  
  //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
  double t1, t2;
  int halfSwingIteration = numIterations/2;
  if (iteration <= halfSwingIteration)
  {
    t1 = iteration*deltaT*2.0;
    deltaPos = 2.0*deltaT*cubicBezierDot(controlNodesPrimary, t1);
    //*pos = cubicBezier(controlNodesPrimary, t1);
  }
  else
  {
    t2 = (iteration-halfSwingIteration)*deltaT*2.0;
    deltaPos = 2.0*deltaT*cubicBezierDot(controlNodesSecondary, t2);
    //*pos = cubicBezier(controlNodesSecondary, t2);
  }
  
  *pos += deltaPos;
  
  //DEBUGGING
  //if (t1 < deltaT) {t1=0.0;}
  //if (t2 < deltaT) {t2=0.0;}
  //cout << "ITERATION: " << iteration << "\tTIME: " << t1 << ":" << t2 << "\tORIGIN: " << originTipPosition[0] << ":" << originTipPosition[1] << ":" << originTipPosition[2] << "\tPOS: " << (*pos)[0] << ":" << (*pos)[1] << ":" << (*pos)[2] << "\tTARGET: " << controlNodesSecondary[3][0] << ":" << controlNodesSecondary[3][1] << ":" << controlNodesSecondary[3][2] << endl; 
  //DEBUGGING
}

void WalkController::LegStepper::updatePosition()
{  
  Vector2d localCentreVelocity = walker->localCentreVelocity;
  double angularVelocity = walker->angularVelocity;
  
  // Swing Phase
  if (state == SWING)
  {
    updateSwingPos(&currentTipPosition);
  }  
  // Stance phase
  else if (state == STANCE ||
           state == STANCE_TRANSITION ||
           state == SWING_TRANSITION)
  {    
    // X & Y Components of Trajectory
    Vector2d deltaPos = -(localCentreVelocity+angularVelocity*
      Vector2d(currentTipPosition[1], -currentTipPosition[0]))*walker->timeDelta;      
    currentTipPosition[0] += deltaPos[0];
    currentTipPosition[1] += deltaPos[1];
    
    // Z component of trajectory  
    /*
    int stanceLength = walker->phaseLength - (walker->swingEnd-walker->swingStart);
    int iteration;
    if (phase >= walker->swingEnd)
    {
      iteration = phase - walker->swingEnd + 1;
    }
    else
    {
      iteration = phase + stanceLength/2 + 1;
    }
    
    double deltaT = 1.0/stanceLength;
    
    Vector3d controlNodes[4];
    double stanceDepth = walker->stepClearance*walker->maximumBodyHeight*0.5;
    
    controlNodes[0] = Vector3d(0.0,0.0,defaultTipPosition[2]);
    controlNodes[1] = Vector3d(0.0,0.0,defaultTipPosition[2]-stanceDepth);
    controlNodes[2] = Vector3d(0.0,0.0,defaultTipPosition[2]-stanceDepth);
    controlNodes[3] = Vector3d(0.0,0.0,defaultTipPosition[2]);
    
    double deltaZ = (deltaT*cubicBezierDot(controlNodes, deltaT*iteration))[2];
    
    currentTipPosition[2] += deltaZ;
    */
    //DEBUGGING
    //cout << "ITERATION: " << iteration << "\tTIME: " <<  deltaT*iteration << "\tORIGIN: " << defaultTipPosition[2] << "\tPOS: " << pos[2] << "\tTARGET: " << controlNodes[2][2] << endl; 
    //DEBUGGING  
  }  
}

/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by 
 * finding the largest footprint radius that each leg can achieve for the 
 * specified level of clearance.
***********************************************************************************************************************/
WalkController::WalkController(Model *model, Parameters p)
{ 
  init(model, p);
}

void WalkController::init(Model *m, Parameters p)
{
  model = m;
  params = p;
  
  stepClearance = params.stepClearance;
  bodyClearance = params.bodyClearance;
  timeDelta = params.timeDelta;
  
  setGaitParams(p);
  
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
    double legDrop = acos((bodyClearance*maximumBodyHeight)/leg.maxLegLength);
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
    {
      rad = min(rad, (horizontalRange - sqrt(sqr(leg.minLegLength) - sqr(legTipBodyClearance))) / 2.0); 
    }
    ASSERT(rad > 0.0); // cannot have negative radius, step height is too high to allow any footprint

    footSpreadDistances[l] = leg.hipLength + horizontalRange - rad;
    
    // this is because the step cycle exceeds the ground footprint in order to maintain velocity
    double footprintDownscale = 0.8; 
    
    minFootprintRadius = min(minFootprintRadius, rad*footprintDownscale);
    
    for (int s = 0; s<2; s++)
    {
      identityTipPositions[l][s] = model->legs[l][s].rootOffset + 
        footSpreadDistances[l]*Vector3d(cos(model->stanceLegYaws[l]), sin(model->stanceLegYaws[l]), 0) + 
          Vector3d(0,0,-bodyClearance*maximumBodyHeight);
          
      identityTipPositions[l][s][0] *= model->legs[l][s].mirrorDir;
      
      legSteppers[l][s].defaultTipPosition = identityTipPositions[l][s];
      legSteppers[l][s].currentTipPosition = identityTipPositions[l][s];
      legSteppers[l][s].phase = 0; // Ensures that feet start stepping naturally and don't pop to up position
      legSteppers[l][s].strideVector = Vector2d(0,0);
      legSteppers[l][s].walker = this;
    }
  }
  // check for overlapping radii
  double minGap = 1e10;
  for (int s = 0; s<2; s++)
  {
    Vector3d posDif = identityTipPositions[1][s] - identityTipPositions[0][s];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
    posDif = identityTipPositions[1][s] - identityTipPositions[2][s];
    posDif[2] = 0.0;
    minGap = min(minGap, posDif.norm() - 2.0*minFootprintRadius);
  }

  if (minGap < 0.0)
  {
    minFootprintRadius += minGap*0.5;
  }

  stanceRadius = abs(identityTipPositions[1][0][0]);

  localCentreVelocity = Vector2d(0,0);
  angularVelocity = 0;

  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*maximumBodyHeight);
}


void WalkController::setGaitParams(Parameters p)
{
  params = p;
  stanceEnd = params.stancePhase*0.5;      
  swingStart = stanceEnd + params.transitionPeriod;
  swingEnd = swingStart + params.swingPhase;      
  stanceStart = swingEnd + params.transitionPeriod;
  
  //Normalises the step phase length to match the total number of iterations over a full step
  int basePhaseLength = params.stancePhase + params.swingPhase + params.transitionPeriod*2.0;
  double swingRatio = (params.swingPhase+params.transitionPeriod)/basePhaseLength; //Used to modify stepFreqency based on gait
  phaseLength = (roundToInt((1.0/(2.0*params.stepFrequency*timeDelta))/(basePhaseLength*swingRatio))*(basePhaseLength*swingRatio))/swingRatio;
  stepFrequency = 1/(phaseLength*timeDelta); //adjust stepFrequency to match corrected phaseLength
  ASSERT(phaseLength%basePhaseLength == 0);
  int normaliser = phaseLength/basePhaseLength;
  stanceEnd *= normaliser;   
  swingStart *= normaliser;
  swingEnd *= normaliser;     
  stanceStart *= normaliser;
  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {       
      int index = 2*l+s;
      int multiplier = params.offsetMultiplier[index];
      legSteppers[l][s].phaseOffset = (int(params.phaseOffset*normaliser)*multiplier)%phaseLength;
      legSteppers[l][s].defaultTipPosition = identityTipPositions[l][s];
      legSteppers[l][s].currentTipPosition = identityTipPositions[l][s];
    }
  }
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines 
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/
void WalkController::updateWalk(Vector2d localNormalisedVelocity, double newCurvature, double deltaZ[3][2], double velocityMultiplier)
{
  double onGroundRatio = double(phaseLength-(swingEnd-swingStart))/double(phaseLength);
  
  Vector2d localVelocity;
  if (state != STOPPING)
  {
    localVelocity = localNormalisedVelocity*minFootprintRadius*stepFrequency/onGroundRatio;
    ASSERT(velocityMultiplier <= 2.0); 
    localVelocity *= velocityMultiplier;
  }
  else
  {
    localVelocity = Vector2d(0.0,0.0);
  }
    
  double normalSpeed = localVelocity.norm();
  ASSERT(normalSpeed < 1.01); // normalised speed should not exceed 1, it can't reach this
  Vector2d oldLocalCentreVelocity = localCentreVelocity;
   
  // we make the speed argument refer to the outer leg, so turning on the spot still has a meaningful speed argument
  double newAngularVelocity = newCurvature * normalSpeed/stanceRadius;
  double dif = newAngularVelocity - angularVelocity;

  if (abs(dif)>0.0)
  {
    angularVelocity += dif * min(1.0, params.maxCurvatureSpeed*timeDelta/abs(dif));
  }

  Vector2d centralVelocity = localVelocity * (1 - abs(newCurvature));
  Vector2d diff = centralVelocity - localCentreVelocity;
  double diffLength = diff.norm();

  if (diffLength > 0.0)
  {
    localCentreVelocity += diff * min(1.0, params.maxAcceleration*timeDelta/diffLength);
  }  
  
  //State transitions for robot state machine.
  // State transition: STOPPED->STARTING
  if (state == STOPPED && normalSpeed)
  {
    state = STARTING;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        legSteppers[l][s].phase = legSteppers[l][s].phaseOffset;
      }
    }
  }  
  // State transition: STARTING->MOVING
  else if (state == STARTING && targetsMet == NUM_LEGS)
  {
    targetsMet = 0;
    state = MOVING;
  }  
  // State transition: MOVING->STOPPING
  else if (state == MOVING && !normalSpeed)
  {
    state = STOPPING;
  }  
  // State transition: STOPPING->STOPPED
  else if (state == STOPPING && targetsMet == NUM_LEGS)
  {
    targetsMet = 0;
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
      
      if (state == STARTING)
      {   
        // Force any leg state into STANCE if it starts in a mid-swing state
        if (legStepper.phaseOffset >= swingStart && legStepper.phaseOffset < swingEnd) //SWING STATE
        {
          if (legStepper.phase == swingEnd)
          {
            targetsMet++;  
            legStepper.metTarget = true;
          }
          else
          {
            legStepper.state = FORCE_STANCE;  
          }
        }
        else if (!legStepper.metTarget)
        {
          targetsMet++;  
          legStepper.metTarget = true;
        }
        
        legStepper.phase = (legStepper.phase+1)%phaseLength; //Iterate phase
      }
      else if (state == STOPPING)
      {  
        //All legs (except front_left) must make one extra step after receiving stopping signal
        if (legStepper.strideVector.norm() == 0 && legStepper.phase == swingEnd)
        {
          legStepper.state = FORCE_STOP;
          if (!(l==0 && s==0))
          {
            if (!legStepper.metTarget)
            {
              legStepper.metTarget = true;
              targetsMet++;
            }
          }
        }          
        
        if (!legStepper.metTarget)
        {
          legStepper.phase = (legStepper.phase+1)%phaseLength; //Iterate phase
          
          //Front_left leg only "meets target" after completing extra step AND returning to zero phase
          if (l==0 && s==0 && legStepper.state == FORCE_STOP && legStepper.phase == 0)
          {
            legStepper.metTarget = true;
            targetsMet++;
            legStepper.state = STANCE;
          }
        }
      }
      else if (state == MOVING)
      {
        legStepper.phase = (legStepper.phase+1)%phaseLength; //Iterate phase
        legStepper.metTarget = false;
      }
      else if (state == STOPPED)
      {        
        legStepper.metTarget = false;
        legStepper.phase = 0;
        legStepper.state = STANCE;
      } 
    }
  } 
  
  //Leg State Machine
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    { 
      LegStepper &legStepper = legSteppers[l][s];       
         
      //Force leg state as STANCE for STARTING robot state
      if (legStepper.state == FORCE_STANCE)
      {
        legStepper.state = STANCE;
      }
      //Force leg state as FORCE_STOP for STOPPING robot state
      else if (legStepper.state == FORCE_STOP)
      {
        legStepper.state = FORCE_STOP;
      }
      else if (legStepper.phase >= stanceEnd && legStepper.phase < swingStart)
      {        
        legStepper.state = SWING_TRANSITION;
      }
      else if (legStepper.phase >= swingStart && legStepper.phase < swingEnd)
      {
        legStepper.state = SWING;
      }
      else if (legStepper.phase >= swingEnd && legStepper.phase < stanceStart)
      {
        legStepper.state = STANCE_TRANSITION;
      }
      else if (legStepper.phase < stanceEnd || legStepper.phase >= stanceStart)
      {        
        legStepper.state = STANCE; 
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
        
        legStepper.updatePosition(); //updates current tip position through step cycle

        Vector3d adjustedPos = legStepper.currentTipPosition;
        adjustedPos[2] -= deltaZ[l][s]; //Impedance controller
        leg.applyLocalIK(adjustedPos); 
      }
    }
  }  
  
  model->clampToLimits();  
  localCentreAcceleration = (localCentreVelocity - oldLocalCentreVelocity) / timeDelta;
  
  //RVIZ
  Vector2d push = localCentreVelocity*timeDelta;
  pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
  pose.rotation *= Quat(Vector3d(0.0,0.0,-angularVelocity*timeDelta));
  //RVIZ
}

/***********************************************************************************************************************
***********************************************************************************************************************/
