#include "../include/simple_hexapod_controller/walkController.h"

/***********************************************************************************************************************
 * Generates control nodes for each quartic bezier curve of swing tip trajectory calculation 
***********************************************************************************************************************/
void WalkController::LegStepper::generateSwingControlNodes(Vector3d strideVector)
{
  double swingHeight = walker->stepClearance*walker->maximumBodyHeight;
  
  //Used to scale the difference between control nodes for the stance curve which has differing delta time values to swing curves
  double bezierScaler = stanceDeltaT/swingDeltaT;
  
  //Control nodes for swing quartic bezier curves - horizontal plane
  swing1ControlNodes[0] = swingOriginTipPosition;         									//Set for horizontal position continuity at transition between stance and primary swing curves (C0 Smoothness)     
  swing1ControlNodes[1] = swing1ControlNodes[0] + bezierScaler*(stanceControlNodes[4]-stanceControlNodes[3]);			//Set for horizontal velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing1ControlNodes[2] = swing1ControlNodes[1] + (swing1ControlNodes[1]-swing1ControlNodes[0]);				//Set for horizontal acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)  
  swing1ControlNodes[4] = defaultTipPosition;											//Set equal to default tip position so that max swing height and transition to 2nd swing curve always occurs at default tip position    
  swing1ControlNodes[3] = (swing1ControlNodes[2]+swing1ControlNodes[4])/2.0;							//Set for horizontal acceleration continuity at transition between primary and secondary swing curves (C2 Smoothness) (symetrical swing curves only!) 
  
  swing2ControlNodes[0] = swing1ControlNodes[4];										//Set for horizontal position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing2ControlNodes[1] = swing2ControlNodes[0] + (swing2ControlNodes[0]-swing1ControlNodes[3]);				//Set for horizontal velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing2ControlNodes[3] = stanceControlNodes[0] + bezierScaler*(stanceControlNodes[0]-stanceControlNodes[1]);			//Set for horizontal velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
  swing2ControlNodes[2] = swing2ControlNodes[3] + swing2ControlNodes[3]-stanceControlNodes[0];					//Set for horizontal acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing2ControlNodes[4] = stanceControlNodes[0];										//Set for horizontal position continuity at transition between secondary swing and stance curves (C0 Smoothness)
  
  //Control nodes for swing quartic bezier curves - vertical plane
  swing1ControlNodes[0][2] = swingOriginTipPosition[2];										//Set for vertical position continuity at transition between stance and primary swing curves (C0 Smoothness)     
  swing1ControlNodes[1][2] = swing1ControlNodes[0][2] + bezierScaler*(stanceControlNodes[4][2]-stanceControlNodes[3][2]);	//Set for vertical velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing1ControlNodes[4][2] = swing1ControlNodes[0][2] + swingHeight;								//Set equal to default tip position plus swing height so that max swing height and transition to 2nd swing curve always occurs at default tip position
  swing1ControlNodes[2][2] = swing1ControlNodes[0][2] + 2.0*bezierScaler*(stanceControlNodes[4][2]-stanceControlNodes[3][2]);	//Set for vertical acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing1ControlNodes[3][2] = swing1ControlNodes[4][2];										//Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)

  swing2ControlNodes[0][2] = swing1ControlNodes[4][2];										//Set for vertical position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing2ControlNodes[1][2] = swing2ControlNodes[0][2];										//Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing2ControlNodes[2][2] = stanceControlNodes[0][2] + 2.0*bezierScaler*(stanceControlNodes[0][2]-stanceControlNodes[1][2]);	//Set for vertical acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing2ControlNodes[3][2] = stanceControlNodes[0][2] + bezierScaler*(stanceControlNodes[0][2]-stanceControlNodes[1][2]);	//Set for vertical velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)					  
  swing2ControlNodes[4][2] = stanceControlNodes[0][2];										//Set for vertical position continuity at transition between secondary swing and stance curves (C0 Smoothness)
}

/***********************************************************************************************************************
 * Generates control nodes for quartic bezier curve of stance tip trajectory calculation 
***********************************************************************************************************************/
void WalkController::LegStepper::generateStanceControlNodes(Vector3d strideVector)
{
  double stanceDepth = walker->stepDepth*walker->maximumBodyHeight;
  
  //Control nodes for stance quartic bezier curve - horizontal plane
  stanceControlNodes[0] = stanceOriginTipPosition;							//Set as initial horizontal tip position  
  stanceControlNodes[4] = stanceOriginTipPosition - strideVector;					//Set as target horizontal tip position 
  stanceControlNodes[1] = stanceControlNodes[4] + 0.75*(stanceControlNodes[0] - stanceControlNodes[4]);	//Set for constant horizontal velocity in stance phase
  stanceControlNodes[2] = stanceControlNodes[4] + 0.5*(stanceControlNodes[0] - stanceControlNodes[4]);	//Set for constant horizontal velocity in stance phase
  stanceControlNodes[3] = stanceControlNodes[4] + 0.25*(stanceControlNodes[0] - stanceControlNodes[4]);	//Set for constant horizontal velocity in stance phase;

  //Control nodes for stance quartic bezier curve - vertical plane
  stanceControlNodes[0][2] = stanceOriginTipPosition[2];						//Set as initial vertical tip position
  stanceControlNodes[4][2] = defaultTipPosition[2];							//Set as target vertical tip position 
  stanceControlNodes[2][2] = stanceControlNodes[0][2] - stanceDepth;					//Set to control depth below ground level of stance trajectory, defined by stanceDepth
  stanceControlNodes[1][2] = (stanceControlNodes[0][2] + stanceControlNodes[2][2])/2.0;			//Set for vertical acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  stanceControlNodes[3][2] = (stanceControlNodes[4][2] + stanceControlNodes[2][2])/2.0;			//Set for vertical acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)  
}

/***********************************************************************************************************************
 * Calculates time deltas for use in quartic bezier curve tip trajectory calculations
***********************************************************************************************************************/
double WalkController::LegStepper::calculateDeltaT(StepState state, int length)
{
  int numIterations = roundToInt((double(length)/walker->phaseLength)/(walker->stepFrequency*walker->timeDelta)/2.0)*2.0;  //Ensure compatible number of iterations 
  if (state == SWING)
  {
    return 2.0/numIterations;
  }
  else
  {
    return 1.0/numIterations;
  }
}

/***********************************************************************************************************************
 * Sets step state
***********************************************************************************************************************/
void WalkController::LegStepper::setState(StepState stepState)
{
  state = stepState;
  walker->stepStates[legIndex][sideIndex] = stepState;
}

/***********************************************************************************************************************
 * Iterates the step phase and updates the progress variables
***********************************************************************************************************************/
void WalkController::LegStepper::iteratePhase()
{
  phase = (phase+1)%(walker->phaseLength);
  
  if (state == SWING)
  {
    swingProgress = double(phase-walker->swingStart+1)/double(walker->swingEnd - walker->swingStart);
    stanceProgress = -1.0;
  }
  else if (state == STANCE)
  {
    stanceProgress = double(mod(phase+(walker->phaseLength-walker->stanceStart), walker->phaseLength)+1)/
		     double(mod(walker->stanceEnd-walker->stanceStart, walker->phaseLength)); 
    swingProgress = -1.0;    
  }    
}

/***********************************************************************************************************************
 * Updates position of tip using tri-quartic bezier curve tip trajectory engine. Calculates change in tip position using
 * the derivatives of three quartic bezier curves, two for swing phase and one for stance phase. Each Bezier curve uses 
 * 5 control nodes designed specifically to give a C2 smooth trajectory for the entire step cycle.  
***********************************************************************************************************************/
void WalkController::LegStepper::updatePosition()
{      
  // Swing Phase
  if (state == SWING)
  {
    int iteration = phase-walker->swingStart+1;
    double swingLength = walker->swingEnd - walker->swingStart;
    swingDeltaT = calculateDeltaT(state, swingLength);
    int numIterations = 2.0/swingDeltaT;
    
    //Save initial tip position at beginning of swing
    if (iteration == 1)
    {
      swingOriginTipPosition = currentTipPosition;
    }
    
    //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    Vector3d deltaPos;
    double t1 = 0;
    double t2 = 0;
    Vector3d strideVec = Vector3d(strideVector[0], strideVector[1], 0.0);
    
    if (iteration <= numIterations/2)
    {
      generateSwingControlNodes(strideVec);
      t1 = iteration*swingDeltaT;
      deltaPos = swingDeltaT*quarticBezierDot(swing1ControlNodes, t1); //
    }
    else
    {
      //Update values of NEXT stance curve for use in calculation of secondary swing control nodes
      int stanceStart = walker->swingEnd;
      int stanceEnd = walker->swingStart;
      int stanceLength = mod(stanceEnd-stanceStart, walker->phaseLength);
      stanceDeltaT = calculateDeltaT(STANCE, stanceLength); 
      stanceOriginTipPosition = defaultTipPosition + 0.5*strideVec;
      generateStanceControlNodes(strideVec);
      
      generateSwingControlNodes(strideVec);
      t2 = (iteration-numIterations/2)*swingDeltaT;
      deltaPos = swingDeltaT*quarticBezierDot(swing2ControlNodes, t2);
    }    
    
    currentTipPosition += deltaPos; 
    currentTipVelocity = deltaPos/walker->timeDelta;
    
    if (t1 < swingDeltaT) {t1=0.0;}
    if (t2 < swingDeltaT) {t2=0.0;}
    if (true)//&(walker->legSteppers[0][0]) == this) //Front left leg
    {
      ROS_DEBUG_COND(params->debugSwingTrajectory, "SWING TRAJECTORY_DEBUG - ITERATION: %d\t\tTIME: %f:%f\t\tORIGIN: %f:%f:%f\t\tPOS: %f:%f:%f\t\tTARGET: %f:%f:%f\n", 
		    iteration, t1, t2,
		    swingOriginTipPosition[0], swingOriginTipPosition[1], swingOriginTipPosition[2],
		    currentTipPosition[0], currentTipPosition[1], currentTipPosition[2],
		    swing2ControlNodes[4][0], swing2ControlNodes[4][1], swing2ControlNodes[4][2]); 
    }
  }  
  // Stance phase
  else if (state == STANCE)
  {      
    int stanceStart = completedFirstStep ? walker->swingEnd : phaseOffset;
    int stanceEnd = walker->swingStart;
    int stanceLength = mod(stanceEnd-stanceStart, walker->phaseLength);
    stanceDeltaT = calculateDeltaT(STANCE, stanceLength);
    
    int iteration = mod(phase+(walker->phaseLength-stanceStart), walker->phaseLength)+1;
    
    //Save initial tip position at beginning of swing
    if (iteration == 1)
    {
      stanceOriginTipPosition = currentTipPosition;
    }    
    
    //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    Vector3d deltaPos;
    double t;
    
    
    //Scales stride vector according to stance length specifically for STARTING state of walker
    Vector3d strideVec = Vector3d(strideVector[0], strideVector[1], 0.0);
    strideVec *= double(stanceLength)/(mod(walker->swingStart-walker->swingEnd,walker->phaseLength));
    
    generateStanceControlNodes(strideVec);
    t = iteration*stanceDeltaT;
    deltaPos = stanceDeltaT*quarticBezierDot(stanceControlNodes, t);
    
    currentTipPosition += deltaPos; 
    currentTipVelocity = deltaPos/walker->timeDelta;
    
    if (t < stanceDeltaT) {t=0.0;}
    if (&(walker->legSteppers[0][0]) == this) //Front left leg
    {
      ROS_DEBUG_COND(params->debugStanceTrajectory, "STANCE TRAJECTORY_DEBUG - ITERATION: %d\t\tTIME: %f\t\tORIGIN: %f:%f:%f\t\tPOS: %f:%f:%f\t\tTARGET: %f:%f:%f\n", 
		    iteration, t,
		    stanceOriginTipPosition[0], stanceOriginTipPosition[1], stanceOriginTipPosition[2],
		    currentTipPosition[0], currentTipPosition[1], currentTipPosition[2],
		    stanceControlNodes[4][0], stanceControlNodes[4][1], stanceControlNodes[4][2]); 
    }    
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
  stepDepth = params.stepDepth;
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
    bodyClearance = (model->legs[0][0].minLegLength + stepClearance*maximumBodyHeight)/maximumBodyHeight;
  }
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);

  minFootprintRadius = 1e10;

  for (int l = 0; l<3; l++)
  {
    // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
    Leg &leg = model->legs[l][0];
    // downward angle of leg
    double legDrop = asin((bodyClearance*maximumBodyHeight)/leg.maxLegLength);
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
    //rad = horizontalRange*sin(theta)/(1+sin(theta)); //ALTERNATIVE ALGORITHM FOR RADIUS OF CIRCLE INSCRIBED BY A SECTOR
    ASSERT(rad > 0.0); // cannot have negative radius

    // we should also take into account the stepClearance not getting too high for the leg to reach
    double legTipBodyClearance = max(0.0, bodyClearance-stepClearance)*maximumBodyHeight; 
    
    // if footprint radius due to lift is smaller due to yaw limits, reduce this minimum radius
    if (legTipBodyClearance < leg.minLegLength)
    {
      rad = min(rad, (horizontalRange - sqrt(sqr(leg.minLegLength) - sqr(legTipBodyClearance))) / 2.0); 
    }
    ASSERT(rad > 0.0); // cannot have negative radius, step height is too high to allow any footprint

    footSpreadDistances[l] = leg.hipLength + horizontalRange - rad;
    
    //FootprintDownscale scales the tip footprint down as the step cycle sometimes exceeds the ground footprint in order to maintain velocity
    minFootprintRadius = min(minFootprintRadius, rad*params.footprintDownscale);
    
    for (int s = 0; s<2; s++)
    {
      identityTipPositions[l][s] = model->legs[l][s].rootOffset + 
        footSpreadDistances[l]*Vector3d(sin(model->stanceLegYaws[l]), -cos(model->stanceLegYaws[l]), 0) + 
          Vector3d(0,0,-bodyClearance*maximumBodyHeight);
          
      identityTipPositions[l][s][1] *= model->legs[l][s].mirrorDir;
      
      legSteppers[l][s].legIndex = l;
      legSteppers[l][s].sideIndex = s;
      legSteppers[l][s].walker = this;
      legSteppers[l][s].defaultTipPosition = identityTipPositions[l][s];
      legSteppers[l][s].currentTipPosition = identityTipPositions[l][s];
      legSteppers[l][s].phase = 0; // Ensures that feet start stepping naturally and don't pop to up position
      legSteppers[l][s].strideVector = Vector2d(0,0);      
      legSteppers[l][s].params = &params;
      legSteppers[l][s].setState(STANCE);
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

  stanceRadius = Vector2d(identityTipPositions[0][1][0], identityTipPositions[0][1][1]).norm();

  currentLinearVelocity = Vector2d(0,0);
  currentAngularVelocity = 0;

  pose.rotation = Quat(1,0,0,0);
  pose.position = Vector3d(0, 0, bodyClearance*maximumBodyHeight);
}


void WalkController::setGaitParams(Parameters p)
{
  params = p;
  stanceEnd = params.stancePhase*0.5;      
  swingStart = stanceEnd;
  swingEnd = swingStart + params.swingPhase;      
  stanceStart = swingEnd;
  
  //Normalises the step phase length to match the total number of iterations over a full step
  int basePhaseLength = params.stancePhase + params.swingPhase;
  double swingRatio = (params.swingPhase)/basePhaseLength; //Used to modify stepFreqency based on gait
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
    }
  }
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines 
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/
void WalkController::updateWalk(Vector2d linearVelocityInput, double angularVelocityInput, Vector3d tipVelocityInput)
{
  double onGroundRatio = double(phaseLength-(swingEnd-swingStart))/double(phaseLength);
  
  Vector2d newLinearVelocity;  
  double newAngularVelocity; 
  
  double maxLinearSpeed = 2.0*minFootprintRadius*stepFrequency/onGroundRatio; //Distance = 2.0*minFootprintRadius, time = onGroundratio*(1/stepFrequency) (stepFrequency is FULL step cycles per second)
  double maxAngularSpeed = maxLinearSpeed/stanceRadius;
  
  //Get new angular/linear velocities according to input mode
  if (state != STOPPING)
  {
    if (params.velocityInputMode == "throttle")
    {      
      newLinearVelocity = clamped(linearVelocityInput, 1.0)*maxLinearSpeed; //Forces input between -1.0 and 1.0
      newAngularVelocity = clamped(angularVelocityInput, -1.0, 1.0)*maxAngularSpeed;
      
      newLinearVelocity *= (1 - abs(angularVelocityInput)); //Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
    }
    else if (params.velocityInputMode == "real")
    {
      newLinearVelocity = clamped(linearVelocityInput, maxLinearSpeed);
      newAngularVelocity = clamped(angularVelocityInput, -maxAngularSpeed, maxAngularSpeed);
      
      newLinearVelocity *= (maxAngularSpeed != 0 ? (1 - abs(newAngularVelocity/maxAngularSpeed)) : 0.0); //Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      
      if (linearVelocityInput.norm() > maxLinearSpeed)
      {
	ROS_WARN_THROTTLE(10, "Input linear speed (%f) exceeds maximum linear speed (%f) and has been clamped.", linearVelocityInput.norm(), maxLinearSpeed);
      }
      if (abs(angularVelocityInput) > maxAngularSpeed)
      {
	ROS_WARN_THROTTLE(10, "Input angular velocity (%f) exceeds maximum angular speed (%f) and has been clamped.", abs(angularVelocityInput), maxAngularSpeed);
      }
    }
  }
  else
  {
    newLinearVelocity = Vector2d(0.0,0.0);
  }
   
  //Angular Acceleration Control
  double angularAcceleration = newAngularVelocity - currentAngularVelocity; 
  //Calculate max angular acceleration if specified by parameter
  if (params.maxAngularAcceleration == -1.0)
  {
    //Ensures tip of last leg to make first swing does not move further than footprint radius before starting first swing (s=0.5*a*(t^2))
    double theta = acos(1 - (sqr(minFootprintRadius)/(2.0*sqr(stanceRadius)))); //Max angular distance within footprint
    params.maxAngularAcceleration = 2.0*theta/sqr(((phaseLength-(swingEnd-swingStart)*0.5)*timeDelta));
  }   
  //Update angular velocity according to acceleration limits
  if (abs(angularAcceleration)>0.0)
  {
    currentAngularVelocity += angularAcceleration * min(1.0, params.maxAngularAcceleration*timeDelta/abs(angularAcceleration));
  }

  //Linear Acceleration Control
  Vector2d linearAcceleration = newLinearVelocity - currentLinearVelocity; 
  //Calculate max acceleration if specified by parameter
  if (params.maxLinearAcceleration == -1.0) 
  {
    //Ensures tip of last leg to make first swing does not move further than footprint radius before starting first swing (s=0.5*a*(t^2))
    params.maxLinearAcceleration = 2.0*minFootprintRadius/sqr(((phaseLength-(swingEnd-swingStart)*0.5)*timeDelta)); 
  }      
  //Update linear velocity according to acceleration limits
  if (linearAcceleration.norm() > 0.0)
  {
    currentLinearVelocity += linearAcceleration*min(1.0, params.maxLinearAcceleration*timeDelta/linearAcceleration.norm());
  } 
  
  bool hasVelocityCommand = linearVelocityInput.norm() || angularVelocityInput;
  
  //Check that all legs are in WALKING state
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      if (hasVelocityCommand && model->legs[l][s].state != WALKING)
      {
	hasVelocityCommand = false;
	if (angularVelocityInput == 0)
	{
	  ROS_INFO_THROTTLE(2,"Unable to walk whilst manually manipulating legs, make sure all legs are in walking state.\n");
	}
      }
    }
  }
  
  //State transitions for robot state machine.
  // State transition: STOPPED->STARTING
  if (state == STOPPED && hasVelocityCommand)
  {
    state = STARTING;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        legSteppers[l][s].phase = legSteppers[l][s].phaseOffset-1;
      }
    }
  }  
  // State transition: STARTING->MOVING
  else if (state == STARTING && legsInCorrectPhase == NUM_LEGS && legsCompletedFirstStep == NUM_LEGS)
  {
    legsInCorrectPhase = 0;
    legsCompletedFirstStep = 0;
    state = MOVING;
  }  
  // State transition: MOVING->STOPPING
  else if (state == MOVING && !hasVelocityCommand)
  {
    state = STOPPING;
  }  
  // State transition: STOPPING->STOPPED
  else if (state == STOPPING && legsInCorrectPhase == NUM_LEGS)
  {
    legsInCorrectPhase = 0;
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
          (currentLinearVelocity + currentAngularVelocity*Vector2d(leg.localTipPosition[1], -leg.localTipPosition[0]))/
          stepFrequency;
      
      if (state == STARTING)
      {  
	legStepper.iteratePhase();
	
	//Check if all legs have completed one step
        if (legsInCorrectPhase == NUM_LEGS)
	{
	  if (legStepper.phase == swingEnd && !legStepper.completedFirstStep)
          {
	    legStepper.completedFirstStep = true;
	    legsCompletedFirstStep++;
	  }
	}
	
        // Force any leg state into STANCE if it starts offset in a mid-swing state
        if (!legStepper.inCorrectPhase)
	{
	  if (legStepper.phaseOffset > swingStart && legStepper.phaseOffset < swingEnd) //SWING STATE
	  {
	    if (legStepper.phase == swingEnd)
	    {
	      legsInCorrectPhase++;  
	      legStepper.inCorrectPhase = true;
	    }
	    else
	    {
	      legStepper.state = FORCE_STANCE;  
	    }
	  }
	  else
	  {
	    legsInCorrectPhase++;  
	    legStepper.inCorrectPhase = true;
	  }      
	}
      }
      else if (state == STOPPING)
      {  
	if (!legStepper.inCorrectPhase)
        {
          legStepper.iteratePhase();
          
          //Front_left leg only "meets target" after completing extra step AND returning to zero phase
          if (l==0 && s==0 && legStepper.state == FORCE_STOP && legStepper.phase == 0)
          {
            legStepper.inCorrectPhase = true;
            legsInCorrectPhase++;
            legStepper.state = STANCE;
          }
        }
	
        //All legs (except front_left) must make one extra step after receiving stopping signal
        if (legStepper.strideVector.norm() == 0 && legStepper.phase == swingEnd)
        {
          legStepper.state = FORCE_STOP;
          if (!(l==0 && s==0))
          {
            if (!legStepper.inCorrectPhase)
            {
              legStepper.inCorrectPhase = true;
              legsInCorrectPhase++;
            }
          }
        }             
      }
      else if (state == MOVING)
      {
        legStepper.iteratePhase();
        legStepper.inCorrectPhase = false;
      }
      else if (state == STOPPED)
      {        
        legStepper.inCorrectPhase = false;
	legStepper.completedFirstStep = false;
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
        legStepper.setState(STANCE);
      }
      //Force leg state as FORCE_STOP for STOPPING robot state
      else if (legStepper.state == FORCE_STOP)
      {
        legStepper.setState(FORCE_STOP);
      }
      else if (legStepper.phase >= swingStart && legStepper.phase < swingEnd)
      {
        legStepper.setState(SWING);
      }
      else if (legStepper.phase < stanceEnd || legStepper.phase >= stanceStart)
      {        
        legStepper.setState(STANCE); 
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
	if (state != STOPPED) 
	{
	  legStepper.updatePosition(); //updates current tip position through step cycle
	}
	tipPositions[l][s] = legStepper.currentTipPosition;
      }
      else if (leg.state == MANUAL)
      {
	if (params.legManipulationMode == "joint_control")
	{
	  double coxaVel = tipVelocityInput[0]*params.maxRotationVelocity*timeDelta;
	  double femurVel = tipVelocityInput[1]*params.maxRotationVelocity*timeDelta;
	  double tibiaVel = tipVelocityInput[2]*params.maxRotationVelocity*timeDelta;
	  leg.yaw += coxaVel;
	  leg.liftAngle += femurVel;
	  leg.kneeAngle += tibiaVel;
	  leg.applyFK();
	  tipPositions[l][s] = leg.localTipPosition;
	}
	else if (params.legManipulationMode == "tip_control") 
	{
	  tipPositions[l][s] += tipVelocityInput*params.maxTranslationVelocity*timeDelta;
	}
      }
    }
  }  
  
  //RVIZ
  if (state != STOPPED)
  {
    Vector2d push = currentLinearVelocity*timeDelta;
    pose.position += pose.rotation.rotateVector(Vector3d(push[0], push[1], 0));
    pose.rotation *= Quat(Vector3d(0.0,0.0,-currentAngularVelocity*timeDelta));
  }
  //RVIZ
}

/***********************************************************************************************************************
***********************************************************************************************************************/
