
#include "../include/simple_hexapod_controller/poseController.h"

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
PoseController::PoseController(Model *model, Parameters p): 
  model(model), 
  params(p), 
  timeDelta(p.timeDelta),
  currentPose(Pose::zero()), 
  targetPose(Pose::zero())
{   
} 

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose at desired speed
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
bool PoseController::updateStance(Vector3d targetTipPositions[3][2], 
                                  Pose requestedTargetPose, 
                                  double timeToPose, 
                                  bool moveLegsSequentially,
                                  int excludeLeg,
                                  int excludeSide
                                 )
{  
  //Instantaneous change in pose
  if (timeToPose == 0)
  {
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        if (l != excludeLeg || s != excludeSide)
        {
          model->legs[l][s].stanceTipPosition = requestedTargetPose.inverseTransformVector(targetTipPositions[l][s]);
          model->stanceTipPositions[l][s] = model->legs[l][s].stanceTipPosition;  
        }
      }
    }
    return true;    
  }
  
  double timeDivisor = moveLegsSequentially ? timeToPose/6.0:timeToPose; //seconds for a leg to move into position
  double timeLimit = moveLegsSequentially ? 6.0:1.0;
  
  //Local pose is already at target pose return immediately 
  if (currentPose == requestedTargetPose)
  {
    return true;
  }
    
  //Check if new target pose is requested
  if (requestedTargetPose != targetPose)
  {      
    targetPose = requestedTargetPose;
    currentPose = Pose::zero(); //unknown current pose
    moveToPoseTime = 0;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        originTipPositions[l][s] = model->legs[l][s].stanceTipPosition; 
      }
    }
  }
  
  
  
  //Increment time
  ASSERT(timeDivisor != 0);
  moveToPoseTime += timeDelta/timeDivisor; 
 
  //Iterate through legs (sequentially or simultaneously) 
  if (moveLegsSequentially)
  {
    int l = int(moveToPoseTime)/2;
    int s = int(moveToPoseTime)%2;
    
    if (l != excludeLeg || s != excludeSide)
    {    
      Leg &leg = model->legs[l][s]; 
      
      if (leg.state == WALKING)
      {    
        Vector3d targetTipPosition = targetPose.inverseTransformVector(targetTipPositions[l][s]);
        
        Vector3d nodes[4];
        nodes[0] = originTipPositions[l][s];
        nodes[1] = nodes[0];
        nodes[3] = targetTipPosition;
        nodes[2] = nodes[3];
        
        Vector3d deltaPos = (timeDelta/timeDivisor)*cubicBezierDot(nodes, fmod(moveToPoseTime, 1.0));
        leg.stanceTipPosition += deltaPos;
        model->stanceTipPositions[l][s] = leg.stanceTipPosition;
      }
    }
  }
  else
  {   
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        if (l != excludeLeg || s != excludeSide)
        { 
          Leg &leg = model->legs[l][s]; 
          
          if (leg.state == WALKING)
          {        
            Vector3d targetTipPosition = targetPose.inverseTransformVector(targetTipPositions[l][s]);
            
            Vector3d nodes[4];
            nodes[0] = originTipPositions[l][s];
            nodes[1] = nodes[0];
            nodes[3] = targetTipPosition;
            nodes[2] = nodes[3];
            
            Vector3d deltaPos = (timeDelta/timeDivisor)*cubicBezierDot(nodes, fmod(moveToPoseTime, 1.0));
            leg.stanceTipPosition += deltaPos;
            model->stanceTipPositions[l][s] = leg.stanceTipPosition;           
          }
        }
      }
    }
  }  
  
  //Reset since target posed achieved
  if (moveToPoseTime >= timeLimit || currentPose == targetPose)
  {
    moveToPoseTime = 0;
    currentPose = targetPose; 
    targetPose = Pose::zero();
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        originTipPositions[l][s] = model->legs[l][s].stanceTipPosition;
      }
    }
    return true;
  }
  
  return false;    
}

/***********************************************************************************************************************
 * Steps legs (sequentially, simultaneously or tripod) into desired tip positions - (updates default stance)
***********************************************************************************************************************/
bool PoseController::stepToPosition(Vector3d (&targetTipPositions)[3][2], int mode, double liftHeight, double timeToStep)
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
 
  int numIterations = roundToInt((timeToStep/timeDelta)/(timeLimit*2))*(timeLimit*2); //Ensure compatible number of iterations 
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
        Vector3d pos;
        
        Vector3d controlNodesPrimary[4];
        Vector3d controlNodesSecondary[4];

        //If tip z position is required to change then set stepHeight to zero
        double stepHeight;
        if (abs(originTipPositions[l][s][2] - targetTipPositions[l][s][2]) > 1e-3)
        {
          stepHeight = 0.0;
        }
        else
        {
          stepHeight = liftHeight;
        }
        
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
        
        //DEBUGGING
        /*
        if (l==0 && s==0)
        {
          double time = (swingIterationCount < halfSwingIteration) ? swingIterationCount*deltaT*2.0 : (swingIterationCount-halfSwingIteration)*deltaT*2.0;
          cout << "MASTER ITERATION: " << masterIterationCount << 
          "        SWING ITERATION: " << swingIterationCount <<
          "        TIME: " << time <<
          "        ORIGIN: " << originTipPositions[0][0][0] << ":" << originTipPositions[0][0][1] << ":" << originTipPositions[0][0][2] <<
          "        CURRENT: " << pos[0] << ":" << pos[1] << ":" << pos[2] <<
          "        TARGET: " << targetTipPositions[0][0][0] << ":" << targetTipPositions[0][0][1] << ":" << targetTipPositions[0][0][2] << endl;
        } 
        */
       
        //Apply inverse kinematics to localTipPositions and stanceTipPositions
        model->legs[l][s].applyLocalIK(pos, true); 
      }
    }
  }  
  model->clampToLimits();
  return false;
}

/***********************************************************************************************************************
 * Safely directly set joint angle
***********************************************************************************************************************/
bool PoseController::moveToJointPosition(Vector3d (&targetJointPositions)[3][2], double timeToMove)
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
      
      //DEBUGGING
      /*
      if (l==0 && s==0)
      {
        double time = masterIterationCount*deltaT;
        cout << "MASTER ITERATION: " << masterIterationCount << 
        "        TIME: " << time <<
        "        ORIGIN: " << originJointPositions[0][0][0] << ":" << originJointPositions[0][0][1] << ":" << originJointPositions[0][0][2] <<
        "        CURRENT: " << pos[0] << ":" << pos[1] << ":" << pos[2] <<
        "        TARGET: " << targetJointPositions[0][0][0] << ":" << targetJointPositions[0][0][1] << ":" << targetJointPositions[0][0][2] << endl;
      }   
      */
        
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
bool PoseController::startUpSequence(double startHeightRatio, double stepHeight, bool forceSequentialMode)
{  
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
  
  if (sequenceStep == 1 && startHeightRatio > 0.8)
  {
    sequenceStep = 3;
  }
  
  if (mode == SEQUENTIAL_MODE)
  {
    stepTime = 6.0;
  }
  else
  {
    stepTime = 2.0;
  }
  
  bool res = false;
  switch (sequenceStep)
  {
    case 1:
      res = stepToPosition(phase1TipPositions, mode, stepHeight, stepTime);
      break;
    case 2:
      res = stepToPosition(phase2TipPositions, NO_STEP_MODE, 0.0, stepTime);
      break;
    case 3:
      res = stepToPosition(phase3TipPositions, mode, stepHeight, stepTime);
      break;
    case 4:
      res = stepToPosition(phase4TipPositions, NO_STEP_MODE, 0.0, stepTime);
      break;
    case 5:
      sequenceStep = 1;
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
bool PoseController::shutDownSequence(double startHeightRatio, double stepHeight, bool forceSequentialMode)
{  
  bool res = false;
  switch (sequenceStep)
  {
    case 1:
      res = stepToPosition(phase5TipPositions, NO_STEP_MODE, 0.0, 2.0);
      break;
    case 2:
      res = stepToPosition(phase6TipPositions, SEQUENTIAL_MODE, stepHeight, 6.0);
      break;
    case 3:
      res = stepToPosition(phase7TipPositions, NO_STEP_MODE, 0.0, 2.0);
      break;
    case 4:
      sequenceStep = 1;
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
  double  desKneeAngle = liftAngle+asin(model->legs[0][0].femurLength/(model->legs[0][0].tibiaLength*sqrt(2)));
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
 * Calculates pitch for auto body compensation
***********************************************************************************************************************/
double PoseController::getPitchCompensation(int leg, int side)
{
  double zDiff = model->localTipPositions[leg][side][2] - model->stanceTipPositions[leg][side][2];
  zDiff *= -(leg-1);
  double pitch = zDiff*params.pitchAmplitude;
  return pitch; 
}

/***********************************************************************************************************************
 * Calculates roll for auto body compensation
***********************************************************************************************************************/
double PoseController::getRollCompensation(int leg, int side)//double phaseProgress)
{ 
  double zDiff = model->localTipPositions[leg][side][2] - model->stanceTipPositions[leg][side][2];
  zDiff *= pow(-1.0, side);
  double roll = zDiff*params.rollAmplitude;
  return roll;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
