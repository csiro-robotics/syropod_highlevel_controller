
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
  autoPose(Pose::identity()),
  imuPose(Pose::identity()),
  inclinationPose(Pose::identity()),
  deltaZPose(Pose::identity())
{   
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
	Pose removeAutoComp = currentPose;
	removeAutoComp.position -= autoPose.position;
	removeAutoComp.rotation[1] -= autoPose.rotation[1];
	removeAutoComp.rotation[2] -= autoPose.rotation[2];
	model->legs[l][s].stanceTipPosition = removeAutoComp.inverseTransformVector(targetTipPositions[l][s]);
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
        
        ROS_DEBUG_COND(params.debugStepToPosition, "STEP_TO_POSITION DEBUG - LEG: %d:%d\t\tMASTER ITERATION: %d\t\tORIGIN: %f:%f:%f\t\tCURRENT: %f:%f:%f\t\tTARGET: %f:%f:%f\n", 
		  l, s, masterIterationCount, 
		  originTipPositions[l][s][0], originTipPositions[l][s][1], originTipPositions[l][s][2], 
		  pos[0], pos[1], pos[2],
		  targetTipPositions[l][s][0], targetTipPositions[l][s][1], targetTipPositions[l][s][2]);  
	
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
  double roll = 0.0;
  double pitch = 0.0;
  double zTrans = 0.0;
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      if (walker->legSteppers[l][s].state == SWING)
      {
	double zDiff = walker->legSteppers[l][s].currentTipPosition[2] - walker->legSteppers[l][s].defaultTipPosition[2];	  
	roll += zDiff*pow(-1.0, s)*params.rollAmplitude; //Inverting as required
	pitch += zDiff*(-(l-1))*params.pitchAmplitude; //Inverting as required
	zTrans += zDiff*params.zTransAmplitude;
      }
    }
  }
  
  autoPose = Pose::identity();
  
  if (walker->params.gaitType == "wave_gait")
  {
    autoPose.rotation[1] += pitch;
    autoPose.rotation[2] += roll;
  }
  else if (walker->params.gaitType == "tripod_gait")
  {
    autoPose.rotation[2] += roll;
    autoPose.position[2] += zTrans;
  }
}

/***********************************************************************************************************************
 * Calculates pitch/roll/yaw/x,y,z for smooth transition to target pose for manual body compensation
***********************************************************************************************************************/
bool PoseController::manualCompensation(Pose requestedTargetPose, double timeToPose)
{      
  //Reset if target changes
  if (requestedTargetPose != targetPose)
  {
    targetPose = requestedTargetPose;
    masterIterationCount = 0;
    originPose = manualPose;
  }
  
  //Check if target met
  if (targetPose == manualPose)
  {
    return true;
  }  
  
  int numIterations = max(1,roundToInt(timeToPose/timeDelta));
  double deltaT = 1.0/numIterations;
  
  if (masterIterationCount < numIterations)
  {
    masterIterationCount++; 
  }
  
  Vector3d positionNodes[4];
  positionNodes[0] = originPose.position;
  positionNodes[1] = originPose.position;
  positionNodes[3] = targetPose.position;
  positionNodes[2] = targetPose.position;
  
  Quat rotationNodes[4];
  rotationNodes[0] = originPose.rotation;
  rotationNodes[1] = originPose.rotation;
  rotationNodes[3] = targetPose.rotation;
  rotationNodes[2] = targetPose.rotation;
            
  manualPose.position = cubicBezier(positionNodes, masterIterationCount*deltaT);
  manualPose.rotation = cubicBezier(rotationNodes, masterIterationCount*deltaT);
  
  double time = masterIterationCount*deltaT;
  ROS_DEBUG_COND(params.debugManualCompensationTranslation, "MANUAL_COMPENSATION_TRANSLATION DEBUG - MASTER ITERATION: %d\t\tTIME: %f\t\tPOS ORIGIN: %f:%f:%f\t\tPOS CURRENT: %f:%f:%f\t\tPOS TARGET: %f:%f:%f\n", 
		  masterIterationCount, time,
		  originPose.position[0], originPose.position[1], originPose.position[2], 
		  manualPose.position[0], manualPose.position[1], manualPose.position[2],
		  targetPose.position[0], targetPose.position[1], targetPose.position[2]);
  ROS_DEBUG_COND(params.debugManualCompensationRotation, "MANUAL_COMPENSATION_ROTATION DEBUG - MASTER ITERATION: %d\t\tTIME: %f\t\tROT ORIGIN: %f:%f:%f\t\tROT CURRENT: %f:%f:%f\t\tROT TARGET: %f:%f:%f\n", 
		  masterIterationCount, time,
		  originPose.rotation[0], originPose.rotation[1], originPose.rotation[2], 
		  manualPose.rotation[0], manualPose.rotation[1], manualPose.rotation[2],
		  targetPose.rotation[0], targetPose.rotation[1], targetPose.rotation[2]);
  return false;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
