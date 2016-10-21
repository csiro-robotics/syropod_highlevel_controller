
#include "../include/simple_hexapod_controller/poseController.h"

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
PoseController::PoseController(Model *model, WalkController *walker, Parameters p): 
  model(model), 
  walker(walker),
  params(p), 
  timeDelta(p.timeDelta),
  currentPose(Pose::identity()), 
  targetPose(Pose::identity()),
  originPose(Pose::identity()),
  manualPose(Pose::identity()),
  imuPose(Pose::identity()),
  inclinationPose(Pose::identity()),
  deltaZPose(Pose::identity())
{   
  sensor_msgs::Imu imuData;
  
  rotationAbsementError = Vector3d(0,0,0);
  rotationPositionError = Vector3d(0,0,0);
  rotationVelocityError = Vector3d(0,0,0);
  
  translationAbsementError = Vector3d(0,0,0);
  translationPositionError = Vector3d(0,0,0);
  translationVelocityError = Vector3d(0,0,0);
  translationAccelerationError = Vector3d(0,0,0);
} 

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
bool PoseController::updateStance(Vector3d targetTipPositions[3][2],
                                  bool excludeSwingingLegs)
{  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      if (!excludeSwingingLegs || walker->legSteppers[l][s].state != SWING)
      {
        model->legs[l][s].stanceTipPosition = currentPose.inverseTransformVector(targetTipPositions[l][s]);
        model->stanceTipPositions[l][s] = model->legs[l][s].stanceTipPosition;  
      }
      else
      {
	//Remove default autoCompensation and apply leg specific auto compensation during swing
	Pose swingAutoComp = currentPose;
	swingAutoComp.position -= (autoPoseDefault.position - autoPose[l][s].position);
	swingAutoComp.rotation[1] -= (autoPoseDefault.rotation[1] - autoPose[l][s].rotation[1]);
	swingAutoComp.rotation[2] -= (autoPoseDefault.rotation[2] - autoPose[l][s].rotation[2]);
	
	model->legs[l][s].stanceTipPosition = swingAutoComp.inverseTransformVector(targetTipPositions[l][s]);
        model->stanceTipPositions[l][s] = model->legs[l][s].stanceTipPosition; 
      }
    }
  }
  return true;       
}

/***********************************************************************************************************************
 * Steps legs (sequentially, simultaneously or tripod) into desired tip positions - (updates default stance)
***********************************************************************************************************************/
bool PoseController::stepToPosition(Vector3d targetTipPositions[3][2], 
				    double deltaZ[3][2], 
				    int mode, 
				    double liftHeight, 
				    double timeToStep)
{ 
  if (firstIteration)
  {       
    firstIteration = false;
    masterIterationCount = 0;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        originTipPositions[l][s] = model->legs[l][s].localTipPosition;
	originTipPositions[l][s][2] += deltaZ[l][s];
        midTipPositions[l][s] = 0.5*(targetTipPositions[l][s] + originTipPositions[l][s]);
      } 
    }  
  } 
  
  if (mode == NO_STEP_MODE)
  {
    liftHeight = 0.0;
  }  
  
  double timeLimit;
  switch (mode)
  {
    case NO_STEP_MODE:
    case SIMULTANEOUS_MODE:
      timeLimit = 1.0;
      break;
    case TRIPOD_MODE:
      timeLimit = 2.0;
      break;
    case SEQUENTIAL_MODE:
    default:
      timeLimit = 6.0;
      break;
  }  
 
  int numIterations = max(1,int(roundToInt((timeToStep/timeDelta)/(timeLimit*2))*(timeLimit*2))); //Ensure compatible number of iterations 
  double deltaT = timeLimit/numIterations;
    
  //Check if master count has reached final iteration and iterate if not
  if (masterIterationCount >= numIterations)
  {    
    firstIteration = true;
    return true;
  }
  else 
  {
    masterIterationCount++; 
  }
  
  //Eg: numIterations = 18 with 6 swings (sequential mode)
  //masterIteration: 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18
  //swingIteration:  1,2,3,1,2,3,1,2,3, 1, 2, 3, 1, 2, 3, 1, 2, 3
  //following code gives this pattern for any provided value of numIterations
  int swingIterationCount = (masterIterationCount+(numIterations/int(timeLimit)-1))%(numIterations/int(timeLimit))+1;
  Vector3d pos;
  
  //Iterate through legs (sequentially or simultaneously)   
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {   
      //Determine if leg is to be moved this iteration based on mode      
      int currentLegInSequence = -1;
      int currentSideInSequence = -1;
      if (mode == SIMULTANEOUS_MODE || mode == NO_STEP_MODE)    //All legs step simultaneously (one step of all legs)
      {
        currentLegInSequence  = l;
        currentSideInSequence = s;
      }
      else if (mode == TRIPOD_MODE)     //Legs step in tripod gait (two steps of 3 legs)
      {
        if (int((masterIterationCount-1)/(numIterations/2)) == (l+s)%2) //Hacky way to get the correct legs for tripod
        {
          currentLegInSequence = l;
          currentSideInSequence = s;
        }
      }
      else if (mode == SEQUENTIAL_MODE) //Legs step sequentially (six steps of single legs)
      {
        currentLegInSequence = (int((double(masterIterationCount-1)/double(numIterations))*3));
        currentSideInSequence = ((masterIterationCount-1)/int(numIterations/timeLimit))%2;
      }
     
      //Update leg tip position
      if (currentLegInSequence == l && currentSideInSequence == s)
      {                
        Vector3d controlNodesPrimary[4];
        Vector3d controlNodesSecondary[4];

        //If tip z position is required to change then set stepHeight to zero
        /*
        double stepHeight;
        if (abs(originTipPositions[l][s][2] - targetTipPositions[l][s][2]) > 1e-3)
        {
          stepHeight = 0.0;
        }
        else
        {
          stepHeight = liftHeight;
        }
        */
        double stepHeight = liftHeight;
        
        //Control nodes for dual 3d cubic bezier curves
        controlNodesPrimary[0] = originTipPositions[l][s]; 
        controlNodesPrimary[1] = controlNodesPrimary[0];    
        controlNodesPrimary[3] = midTipPositions[l][s];  
        controlNodesPrimary[3][2] += stepHeight;
        controlNodesPrimary[2] = controlNodesPrimary[0];
        controlNodesPrimary[2][2] += stepHeight;
        
        controlNodesSecondary[0] = controlNodesPrimary[3];
        controlNodesSecondary[1] = 2*controlNodesSecondary[0] - controlNodesPrimary[2];
        controlNodesSecondary[3] = targetTipPositions[l][s];        
        controlNodesSecondary[2] = controlNodesSecondary[3];
        
        //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
        int halfSwingIteration = (numIterations/int(timeLimit))/2;
        if (swingIterationCount <= halfSwingIteration)
        {
          pos = cubicBezier(controlNodesPrimary, swingIterationCount*deltaT*2.0);          
        }
        else
        {
          pos = cubicBezier(controlNodesSecondary, (swingIterationCount-halfSwingIteration)*deltaT*2.0);
        }    
        
        if (s==0 && l==0)
	{
	  ROS_DEBUG_COND(params.debugStepToPosition, "STEP_TO_POSITION DEBUG - LEG: %d:%d\t\tMASTER ITERATION: %d\t\tORIGIN: %f:%f:%f\t\tCURRENT: %f:%f:%f\t\tTARGET: %f:%f:%f\n", 
		  l, s, masterIterationCount, 
		  originTipPositions[l][s][0], originTipPositions[l][s][1], originTipPositions[l][s][2], 
		  pos[0], pos[1], pos[2],
		  targetTipPositions[l][s][0], targetTipPositions[l][s][1], targetTipPositions[l][s][2]);  
	}
	
      }
      else
      {
	//pos = model->stanceTipPositions[l][s];
	pos = model->localTipPositions[l][s];
      }
      
      //Apply inverse kinematics to localTipPositions and stanceTipPositions
      if (model->legs[l][s].state != OFF)
      {
	Vector3d adjustedPos = pos;
	adjustedPos[2] -= deltaZ[l][s]; //Impedance controller
	model->legs[l][s].applyLocalIK(adjustedPos); 
      }
    }
  }  
  model->clampToLimits();
  return false;
}

/***********************************************************************************************************************
 * Safely directly set joint angle
***********************************************************************************************************************/
bool PoseController::moveToJointPosition(Vector3d targetJointPositions[3][2], double timeToMove)
{
  //Setup origin and target joint positions for bezier curve
  if (firstIteration)
  {
    firstIteration = false;
    masterIterationCount = 0;
    for (int l=0; l<3; l++)
    {
      for (int s=0; s<2; s++)
      {
        originJointPositions[l][s] = Vector3d(model->legs[l][s].yaw,
                                              model->legs[l][s].liftAngle,
                                              model->legs[l][s].kneeAngle);
      }
    }
  }

  int numIterations = roundToInt(timeToMove/timeDelta);
  double deltaT = 1.0/numIterations;
  
  //Return true at end after all iterations (target achieved)
  if (masterIterationCount >= numIterations)
  {    
    firstIteration = true;
    return true;
  }
  else 
  {
    masterIterationCount++; 
  }
  
  Vector3d pos;  
  Vector3d controlNodes[4];
  
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      //Control nodes for linear cubic bezier curve
      controlNodes[0] = originJointPositions[l][s]; 
      controlNodes[1] = originJointPositions[l][s];  
      controlNodes[2] = targetJointPositions[l][s];
      controlNodes[3] = targetJointPositions[l][s];
      
      //Calculate change in position using bezier curve
      pos = cubicBezier(controlNodes, masterIterationCount*deltaT); 

      if (l==0 && s==0)
      {
	double time = masterIterationCount*deltaT;
	ROS_DEBUG_COND(params.debugMoveToJointPosition, "MOVE_TO_JOINT_POSITION DEBUG - MASTER ITERATION: %d\t\tTIME: %f\t\tORIGIN: %f:%f:%f\t\tCURRENT: %f:%f:%f\t\tTARGET: %f:%f:%f\n", 
		  masterIterationCount, time,
		  originJointPositions[l][s][0], originJointPositions[l][s][1], originJointPositions[l][s][2], 
		  pos[0], pos[1], pos[2],
		  targetJointPositions[l][s][0], targetJointPositions[l][s][1], targetJointPositions[l][s][2]);
      }   

      model->legs[l][s].yaw = pos[0];
      model->legs[l][s].liftAngle = pos[1];
      model->legs[l][s].kneeAngle = pos[2];
      model->legs[l][s].applyFK();
    }
  } 
  return false;
}

/***********************************************************************************************************************
 * Startup sequence
***********************************************************************************************************************/
bool PoseController::startUpSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode)
{  
  if (sequenceStep == 0)
  {
    startHeightRatio = createSequence(targetTipPositions);
    if (startHeightRatio > 0.8)
    {
      sequenceStep = 3;
    }
    else
    {
      sequenceStep = 1;
    }
  }
  
  int mode;
  double stepTime;
  if (forceSequentialMode)
  {
    mode = SEQUENTIAL_MODE;
  }
  else if (sequenceStep == 1)
  {
    mode = startHeightRatio < 0.1 ? NO_STEP_MODE:SEQUENTIAL_MODE;
  }
  else if (sequenceStep == 3)
  {
    mode = startHeightRatio > 0.8 ? TRIPOD_MODE:SEQUENTIAL_MODE;
  }
  
  if (mode == SEQUENTIAL_MODE)
  {
    stepTime = 6.0/params.stepFrequency;
  }
  else
  {
    stepTime = 2.0/params.stepFrequency;
  }
  
  bool res = false;
  double stepHeight = walker->maximumBodyHeight*walker->stepClearance;
  double deltaZ[3][2] = {{0,0},{0,0},{0,0}};
  switch (sequenceStep)
  {
    case 1:
      res = stepToPosition(phase1TipPositions, deltaZ, mode, stepHeight, stepTime);
      break;
    case 2:
      res = stepToPosition(phase2TipPositions, deltaZ, NO_STEP_MODE, 0.0, stepTime);
      break;
    case 3:
      res = stepToPosition(phase3TipPositions, deltaZ, mode, stepHeight, stepTime);
      break;
    case 4:
      res = stepToPosition(phase4TipPositions, deltaZ, NO_STEP_MODE, 0.0, stepTime);
      break;
    case 5:
      sequenceStep = 0;
      return true;
    default:
      return false;
  }
  
  if (res) 
  {
    sequenceStep++;
  }
  
  return false;
}

/***********************************************************************************************************************
 * Shutdown sequence
***********************************************************************************************************************/
bool PoseController::shutDownSequence(Vector3d targetTipPositions[3][2], bool forceSequentialMode)
{  
  bool res = false;
  double stepHeight = walker->maximumBodyHeight*walker->stepClearance;
  double deltaZ[3][2] = {{0,0},{0,0},{0,0}};
  switch (sequenceStep)
  {
    case 0:
      createSequence(targetTipPositions);
      res = true;
      break;
    case 1:
      res = stepToPosition(phase5TipPositions, deltaZ, NO_STEP_MODE, 0.0, 2.0/params.stepFrequency);
      break;
    case 2:
      res = stepToPosition(phase6TipPositions, deltaZ, SEQUENTIAL_MODE, stepHeight, 6.0/params.stepFrequency);
      break;
    case 3:
      res = stepToPosition(phase7TipPositions, deltaZ, NO_STEP_MODE, 0.0, 2.0/params.stepFrequency);
      break;
    case 4:
      sequenceStep = 0;
      return true;
    default:      
      return false;
  }
  
  if (res) 
  {
    sequenceStep++;
  }
  
  return false;
}

/***********************************************************************************************************************
 * Calculates tip positions for startup/shutdown sequences
***********************************************************************************************************************/
//TBD REFACTORING
double PoseController::createSequence(Vector3d targetTipPositions[3][2])
{  
  //Get average z position of leg tips
  double averageTipZ = 0.0;
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      averageTipZ += model->legs[l][s].localTipPosition[2];
    }
  }
  averageTipZ /= 6.0;
  
  //Ratio of average tip z position to required default tip z position 
  double startHeightRatio = averageTipZ/targetTipPositions[0][0][2];
  if (startHeightRatio < 0.0)
  {
    startHeightRatio = 0.0;
  }
  
  double liftAngle = pi/3;
  double  desKneeAngle = liftAngle+asin(model->legs[0][0].femurLength*sin(liftAngle)/model->legs[0][0].tibiaLength);
  double offset = atan2(model->legs[0][0].tipOffset[2],model->legs[0][0].tipOffset[0]);
  desKneeAngle += offset;
  Vector3d minStartTipPositions = model->legs[0][0].calculateFK(0.77,liftAngle,desKneeAngle);
  double startStanceRatio = minStartTipPositions.squaredNorm()/targetTipPositions[0][0].squaredNorm();
 
  double heightScaler = 0.85;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      double positionScaler = (-(startStanceRatio-1.0)*startHeightRatio+startStanceRatio);
      phase1TipPositions[l][s] = targetTipPositions[l][s]*positionScaler;
      phase1TipPositions[l][s][2] = -0.025; //parameterise
      
      phase2TipPositions[l][s] = phase1TipPositions[l][s];
      phase2TipPositions[l][s][2] = (heightScaler+startHeightRatio*(1-heightScaler))*targetTipPositions[0][0][2];
      
      phase3TipPositions[l][s] = targetTipPositions[l][s];
      phase3TipPositions[l][s][2] = phase2TipPositions[l][s][2];
      
      phase4TipPositions[l][s] = targetTipPositions[l][s];
      
      phase5TipPositions[l][s] = model->legs[l][s].localTipPosition;
      phase5TipPositions[l][s][2] *= heightScaler;
      
      phase6TipPositions[l][s] = targetTipPositions[l][s]*1.3;
      phase6TipPositions[l][s][2] = phase5TipPositions[l][s][2];
      
      phase7TipPositions[l][s] = phase6TipPositions[l][s];
      phase7TipPositions[l][s][2] = -0.025; //parameterise
    }
  }   
  return startHeightRatio;
}

/***********************************************************************************************************************
 * Reset sequence step
***********************************************************************************************************************/
void PoseController::resetSequence(void)
{
  sequenceStep = 1;
  firstIteration = true;
}

/***********************************************************************************************************************
 * Calculates pitch/roll for smooth auto body compensation from offset pose
***********************************************************************************************************************/
void PoseController::autoCompensation()
{    
  double swingHeightPercentage = 1.0;
  double roll[3][2];
  double pitch[3][2];
  double zTrans[3][2];
  
  //Turn on/off the calculation of roll/pitch/ztrans based on curent gait
  int calculateRoll = int(walker->params.gaitType != "amble_gait");
  int calculatePitch = int(walker->params.gaitType != "tripod_gait" && walker->params.gaitType != "amble_gait");
  int calculateZTrans = int(walker->params.gaitType == "tripod_gait");
  
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      if (walker->legSteppers[l][s].state == SWING)
      {
	swingHeightPercentage = abs(walker->legSteppers[l][s].currentTipPosition[2] - walker->legSteppers[l][s].defaultTipPosition[2])/(walker->stepClearance*walker->maximumBodyHeight);

	roll[l][s] = swingHeightPercentage*pow(-1.0, s)*params.rollAmplitude*calculateRoll; //Inverting as required
	pitch[l][s] = swingHeightPercentage*(-(l-1))*params.pitchAmplitude*calculatePitch; //Inverting as required
	zTrans[l][s] = swingHeightPercentage*params.zTransAmplitude*calculateZTrans;
      } 
      else
      {
	roll[l][s] = 0.0;
	pitch[l][s] = 0.0;
	zTrans[l][s] = 0.0;		
      }
    }
  }
  
  //Calculates how many legs are perfectly in phase.
  int legsInPhase = 6/(1+*max_element(walker->params.offsetMultiplier.begin(), walker->params.offsetMultiplier.end()));
  
  //Only adds pitch/roll/zTrans values from 'lead' legs (this ensures value from 'in phase' legs is only added once).
  autoPoseDefault = Pose::identity();
  for (int i=0; i<(6/legsInPhase); i++)
  {
    int leadLegRef = i/2;
    int leadSideRef = i%2;
    
    autoPoseDefault.rotation[2] += roll[leadLegRef][leadSideRef];
    autoPoseDefault.rotation[1] += pitch[leadLegRef][leadSideRef];
    autoPoseDefault.position[2] += zTrans[leadLegRef][leadSideRef];    
  }
  
  //Reduce pose on each leg to zero during swing by subtracting compensation added by it's lead leg.
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {       
      int index = 2*l+s;
      int inPhaseLeadIndex = distance(walker->params.offsetMultiplier.begin(), 
				      find(walker->params.offsetMultiplier.begin(),
					   walker->params.offsetMultiplier.end(), 
					   walker->params.offsetMultiplier[index]));
      int inPhaseLeadLegRef = inPhaseLeadIndex/2;
      int inPhaseLeadSideRef = inPhaseLeadIndex%2;      
      
      autoPose[l][s] = autoPoseDefault;
      autoPose[l][s].rotation[2] -= roll[inPhaseLeadLegRef][inPhaseLeadSideRef];
      autoPose[l][s].rotation[1] -= pitch[inPhaseLeadLegRef][inPhaseLeadSideRef];
      autoPose[l][s].position[2] -= zTrans[inPhaseLeadLegRef][inPhaseLeadSideRef];
    }
  }
}

/***********************************************************************************************************************
 * Calculates pitch/roll/yaw/x,y,z for smooth transition to target pose for manual body compensation
***********************************************************************************************************************/
void PoseController::manualCompensation(Pose newPosingVelocity, PoseResetMode poseResetMode)
{   
  Vector3d translationPosition = manualPose.position;
  Vector3d rotationPosition = Vector3d(manualPose.rotation[1], manualPose.rotation[2], manualPose.rotation[3]); //Quat to Euler
  
  Vector3d maxTranslation = params.maxTranslation;
  Vector3d maxRotation = params.maxRotation;
  switch (poseResetMode)
  {
    case (Z_AND_YAW_RESET):
      maxTranslation[2] = 0.0;
      maxRotation[2] = 0.0;
      break;
    case (X_AND_Y_RESET):
      maxTranslation[0] = 0.0;
      maxTranslation[1] = 0.0;
      break;
    case (PITCH_AND_ROLL_RESET):
      maxRotation[0] = 0.0;
      maxRotation[1] = 0.0;
      break;
    case (ALL_RESET):
      maxTranslation = Vector3d(0.0,0.0,0.0);
      maxRotation = Vector3d(0.0,0.0,0.0);
      break;
    case (NO_RESET):
      maxTranslation = params.maxTranslation;
      maxRotation = params.maxRotation;
      break;
  }
  
  //Override posing velocity commands depending on pose reset mode 
  for (int i=0; i<3; i++) //For each axis (x,y,z)/(roll,pitch,yaw)
  {      
    if (maxTranslation[i] == 0.0)
    {
      newPosingVelocity.position[i] = -sign(translationPosition[i])*1.0;
    }
    if (maxRotation[i] == 0.0)
    {
      newPosingVelocity.rotation[i+1] = -sign(rotationPosition[i])*1.0;
    }
  }
  
  Vector3d translationVelocity = clamped(newPosingVelocity.position, 1.0)*params.maxTranslationVelocity;
  Vector3d eulerRotationVelocity = Vector3d(newPosingVelocity.rotation[1], newPosingVelocity.rotation[2], newPosingVelocity.rotation[3]);
  Vector3d rotationVelocity = clamped(eulerRotationVelocity, 1.0)*params.maxRotationVelocity;  
  
  //Control velocity
  for (int i=0; i<3; i++) //For each axis (x,y,z)/(roll,pitch,yaw)
  {     
    //Zero velocity when translation position reaches limit
    if (sign(translationVelocity[i]) > 0)
    {
      if (translationPosition[i] + translationVelocity[i]*timeDelta > maxTranslation[i])
      {
	translationVelocity[i] = 0;
	translationPosition[i] = maxTranslation[i];
      }
    }
    else
    {
      if (translationPosition[i] + translationVelocity[i]*timeDelta < -maxTranslation[i])
      {
	translationVelocity[i] = 0;
	translationPosition[i] = -maxTranslation[i];
      }
    }
    
    //Zero velocity when rotation position reaches limit
    if (sign(rotationVelocity[i]) > 0)
    {
      if (rotationPosition[i] + rotationVelocity[i]*timeDelta > maxRotation[i])
      {
	rotationVelocity[i] = 0;
	rotationPosition[i] = maxRotation[i];
      }
    }
    else
    {
      if (rotationPosition[i] + rotationVelocity[i]*timeDelta < -maxRotation[i])
      {
	rotationVelocity[i] = 0;
	rotationPosition[i] = -maxRotation[i];
      }
    }
    
    //Update position
    manualPose.position[i] = translationPosition[i]+translationVelocity[i]*timeDelta;
    manualPose.rotation[i+1] = rotationPosition[i]+rotationVelocity[i]*timeDelta;
  }   
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::imuCompensation(sensor_msgs::Imu imuData, Quat targetRotation)
{
  //ROTATION COMPENSATION
  Quat orientation; 		//IMU Orientation
  Vector3d angularVelocity;	//IMU Angular Velocity
  
  //Get orientation data from IMU
  orientation.w = imuData.orientation.w;
  orientation.x = imuData.orientation.x;
  orientation.y = imuData.orientation.y;
  orientation.z = imuData.orientation.z;
  
  //Get angular velocity data from IMU
  angularVelocity(0) = imuData.angular_velocity.x;
  angularVelocity(1) = imuData.angular_velocity.y;
  angularVelocity(2) = imuData.angular_velocity.z;
  
  //There are two orientations per quaternion and we want the shorter/smaller difference. 
  double dot = targetRotation.dot(~orientation);
  if (dot < 0.0)
  {
    targetRotation = -targetRotation;
  }
  
  //PID gains
  double kD = params.rotationCompensationDerivativeGain;
  double kP = params.rotationCompensationProportionalGain;
  double kI = params.rotationCompensationIntegralGain;
  
  rotationPositionError = orientation.toEulerAngles() - targetRotation.toEulerAngles();
  rotationAbsementError += rotationPositionError*params.timeDelta; //Integration of angle position error (absement)
  
  //Low pass filter of IMU angular velocity data
  double smoothingFactor = 0.15;
  rotationVelocityError = smoothingFactor*angularVelocity + (1-smoothingFactor)*rotationVelocityError;
  
  Vector3d rotationCorrection = kD*rotationVelocityError + kP*rotationPositionError + kI*rotationAbsementError;
  rotationCorrection[2] = 0; //No compensation in yaw rotation
  
  double stabilityThreshold = 100;
    
  if (rotationCorrection.norm() > stabilityThreshold)
  {
    ROS_FATAL("IMU rotation compensation became unstable! Adjust PID parameters.\n");
    ASSERT(rotationCorrection.norm() < stabilityThreshold);
  }
  else
  { 
    imuPose = Pose::identity();
    imuPose.rotation[1] = -rotationCorrection[0]; //Pitch correction
    imuPose.rotation[2] = -rotationCorrection[1]; //Roll correction
  } 

  //TRANSLATION COMPENSATION
  //DOES NOT CURRENTLY WORK FULLY
  /*
  Quat orientation;	//IMU Orientation
  Vector3d linearAcceleration;	//IMU Linear Acceleration
  
  //Get orientation data from IMU
  orientation.w = imuData.orientation.w;
  orientation.x = imuData.orientation.x;
  orientation.y = imuData.orientation.y;
  orientation.z = imuData.orientation.z;
  
  //Get linear acceleration data from IMU
  linearAcceleration(0) = imuData.linear_acceleration.x;
  linearAcceleration(1) = imuData.linear_acceleration.y;
  linearAcceleration(2) = imuData.linear_acceleration.z;
  
  //PID gains
  double kD = params.translationCompensationDerivativeGain;
  double kP = params.translationCompensationProportionalGain;
  double kI = params.translationCompensationIntegralGain;
  
  //Remove gravity
  Vector3d gravity = {0.0,0.0,9.75};  
  Vector3d orientedGravity = orientation.rotateVector(gravity);
  Vector3d dynamicLinearAcceleration = linearAcceleration - orientedGravity);
  
  double decayRate = 2.3; 
  
  //Low pass filter of IMU linear acceleration data (after removing acceleration due to gravity)   
  double smoothingFactor = 0.15;
  translationAccelerationError = smoothingFactor*dynamicLinearAcceleration + (1-smoothingFactor)*translationAccelerationError;
  
  //Integrate for velocity and position and absement
  translationVelocityError += translationAccelerationError*timeDelta - decayRate*timeDelta*translationVelocityError;
  translationPositionError += translationVelocityError*timeDelta - decayRate*timeDelta*translationPositionError;
  translationAbsementError += translationPositionError*timeDelta;
  
  Vector3d translationCorrection = kD*translationVelocityError + kP*translationPositionError + kI*translationAbsementError;  
  translationCorrection[2] = 0; //No compensation in z translation (competes with impedance controller)
  */  
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::inclinationCompensation(sensor_msgs::Imu imuData)
{
  Quat orientation;
  
  //Get orientation data from IMU
  orientation.w = imuData.orientation.w;
  orientation.x = imuData.orientation.x;
  orientation.y = imuData.orientation.y;
  orientation.z = imuData.orientation.z;
  
  Vector3d eulerAngles = orientation.toEulerAngles() - manualPose.rotation.toEulerAngles() - autoPoseDefault.rotation.toEulerAngles();
  
  double longitudinalCorrection = walker->bodyClearance*walker->maximumBodyHeight*tan(eulerAngles[0]);
  double lateralCorrection = -walker->bodyClearance*walker->maximumBodyHeight*tan(eulerAngles[1]); 
  
  inclinationPose.position[0] = lateralCorrection*min(1.0, params.maxTranslation[0]/abs(lateralCorrection));
  inclinationPose.position[1] = longitudinalCorrection*min(1.0, params.maxTranslation[1]/abs(longitudinalCorrection));
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::impedanceControllerCompensation(double deltaZ[3][2])
{
  double averageDeltaZ = 0.0;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      averageDeltaZ += deltaZ[l][s];
    }
  }
  averageDeltaZ /= 6.0;
  
  deltaZPose = Pose::identity();
  deltaZPose.position[2] = abs(averageDeltaZ);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
