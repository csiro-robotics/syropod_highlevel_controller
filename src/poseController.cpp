
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
                                  bool moveLegsSequentially)
{  
  double timeDivisor = moveLegsSequentially ? timeToPose/6.0:timeToPose; //seconds for a leg to move into position
  double timeLimit = moveLegsSequentially ? 6.0:1.0;
  
  //Local pose is already at target pose return immediately 
  if (currentPose == requestedTargetPose)
    return true;
    
  //Check if new target pose is requested
  if (requestedTargetPose != targetPose)
  {      
    targetPose = requestedTargetPose;
    currentPose = Pose::zero(); //unknown current pose
    moveToPoseTime = 0;
    for (int l = 0; l<3; l++)
      for (int s = 0; s<2; s++)
        originTipPositions[l][s] = model->legs[l][s].stanceTipPosition; 
  }
  
  //Instantaneous change in pose
  if (timeToPose == 0)
  {
    for (int l = 0; l<3; l++)
      for (int s = 0; s<2; s++)
        model->legs[l][s].stanceTipPosition = requestedTargetPose.inverseTransformVector(targetTipPositions[l][s]);
    return true;    
  }
  
  //Increment time
  ASSERT(timeDivisor != 0);
  moveToPoseTime += timeDelta/timeDivisor; 
 
  //Iterate through legs (sequentially or simultaneously) 
  if (moveLegsSequentially)
  {
    int l = int(moveToPoseTime)/2;
    int s = int(moveToPoseTime)%2;
    
    Leg &leg = model->legs[l][s]; 
    
    Vector3d currentTipPosition = originTipPositions[l][s];
    Vector3d targetTipPosition = targetPose.inverseTransformVector(targetTipPositions[l][s]);
    
    Vector3d nodes[4];
    nodes[0] = currentTipPosition;
    nodes[1] = nodes[0];
    nodes[3] = targetTipPosition;
    nodes[2] = nodes[3];
    
    Vector3d deltaPos = (timeDelta/timeDivisor)*cubicBezierDot(nodes, fmod(moveToPoseTime, 1.0));
    leg.stanceTipPosition += deltaPos;
  }
  else
  {   
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        Leg &leg = model->legs[l][s]; 
        
        Vector3d currentTipPosition = originTipPositions[l][s];
        Vector3d targetTipPosition = targetPose.inverseTransformVector(targetTipPositions[l][s]);
        
        Vector3d nodes[4];
        nodes[0] = currentTipPosition;
        nodes[1] = nodes[0];
        nodes[3] = targetTipPosition;
        nodes[2] = nodes[3];
        
        Vector3d deltaPos = (timeDelta/timeDivisor)*cubicBezierDot(nodes, fmod(moveToPoseTime, 1.0));
        leg.stanceTipPosition += deltaPos;
      }
    }
  }  
  
  //Reset since target posed achieved
  if (moveToPoseTime >= timeLimit || currentPose == targetPose)
  {
    moveToPoseTime = 0;
    currentPose = targetPose; 
    for (int l = 0; l<3; l++)
      for (int s = 0; s<2; s++)
        originTipPositions[l][s] = model->legs[l][s].stanceTipPosition;
    return true;
  }
  
  return false;    
}

/***********************************************************************************************************************
 * Steps legs (sequentially, simultaneously or tripod) into desired tip positions - (updates default stance)
***********************************************************************************************************************/
bool PoseController::stepToPosition(Vector3d (&targetTipPositions)[3][2], int mode, double liftHeight, double stepSpeed)
{ 
  if (firstIteration)
  {
    firstIteration = false;
    moveToPoseTime = 0.0;
    for (int l = 0; l<3; l++)
    {
      for (int s = 0; s<2; s++)
      {
        originTipPositions[l][s] = model->legs[l][s].localTipPosition;
        midTipPositions[l][s] = 0.5*(targetTipPositions[l][s] + originTipPositions[l][s]);
      } 
    }      
  }
  
  double deltaT = timeDelta*stepSpeed;
  
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
  
  if (abs(moveToPoseTime + deltaT - timeLimit) < 1e-3 || moveToPoseTime > timeLimit)
  {    
    firstIteration = true;
    return true;
  }
  else 
  {
    moveToPoseTime += deltaT; 
  }
  
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
        if (int(moveToPoseTime) == (l+s)%2)
        {
          currentLegInSequence = l;
          currentSideInSequence = s;
        }
      }
      else if (mode == SEQUENTIAL_MODE) //Legs step sequentially (six steps of single legs)
      {
        currentLegInSequence = (int(moveToPoseTime)/2);
        currentSideInSequence = (int(moveToPoseTime)%2);
      }
     
      //Update leg tip position
      if (currentLegInSequence == l && currentSideInSequence == s)
      {      
        Vector3d pos;
        
        Vector3d controlNodesPrimary[4];
        Vector3d controlNodesSecondary[4];
        
        //Control nodes for dual 3d cubic bezier curves
        //Set as initial tip position
        controlNodesPrimary[0] = originTipPositions[l][s]; 
        
        //Set equal to node 1 gives zero velocity at liftoff
        controlNodesPrimary[1] = controlNodesPrimary[0];    
        
        //Set equal to mid trajectory position so that max liftheight and 
        //transition to 2nd bezier curve occurs halfway along trajectory
        controlNodesPrimary[3] = midTipPositions[l][s];   
        
        //Set accordingly for constant acceleration
        controlNodesPrimary[2] = (controlNodesPrimary[3] + 2*controlNodesPrimary[0])/3.0;      
        
        //Set equal to primary control node 3 to allow continuity between curves
        controlNodesSecondary[0] = controlNodesPrimary[3];  
        
        //Set as target tip position according to stride vector
        controlNodesSecondary[3] = targetTipPositions[l][s]; 
        
        //Set equal to secondary node 3 gives zero velocity at touchdown
        controlNodesSecondary[2] = controlNodesSecondary[3];
        
        //Set accordingly so that velocity at end of primary curve equals velocity at begginning of secondary curve
        controlNodesSecondary[1] = (controlNodesSecondary[0] + 2*controlNodesSecondary[2])/3.0;
        
        //If stepping assume load bearing leg and return to starting height is required
        if (mode != NO_STEP_MODE)
        {
          controlNodesPrimary[2][2] = originTipPositions[l][s][2] + liftHeight;  
          controlNodesPrimary[3][2] = originTipPositions[l][s][2] + liftHeight; 
          controlNodesSecondary[0][2] = originTipPositions[l][s][2] + liftHeight;
          controlNodesSecondary[1][2] = originTipPositions[l][s][2] + liftHeight;
          controlNodesSecondary[2][2] = originTipPositions[l][s][2];
          controlNodesSecondary[3][2] = originTipPositions[l][s][2];
        }  
        
        
        //Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
        if (fmod(moveToPoseTime,1.0) < 0.5)
        {
          pos = cubicBezier(controlNodesPrimary, fmod(moveToPoseTime,1.0)*2.0);          
        }
        else
        {
          pos = cubicBezier(controlNodesSecondary, (fmod(moveToPoseTime,1.0)-0.5)*2.0);
        }
        
        
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
bool PoseController::moveToJointPosition(Vector3d (&targetJointPositions)[3][2], double speed)
{
  //Setup origin and target joint positions for bezier curve
  if (firstIteration)
  {
    firstIteration = false;
    moveToPoseTime = 0.0;
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
  
  double deltaT = timeDelta*speed;  
  double timeLimit = 1.0;
  
  //Return true at end of timeLimit (target achieved)
  if (abs(moveToPoseTime + deltaT - timeLimit) < 1e-3 || moveToPoseTime > timeLimit)
  {    
    firstIteration = true;
    return true;
  }
  else 
  {
    moveToPoseTime += deltaT; 
  }
  
  Vector3d pos;  
  Vector3d controlNodes[4];
  
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      //Control nodes for linear quadratic bezier curve
      controlNodes[0] = originJointPositions[l][s]; 
      controlNodes[1] = originJointPositions[l][s];  
      controlNodes[2] = targetJointPositions[l][s];
      controlNodes[3] = targetJointPositions[l][s];
      
      //Calculate change in position using bezier curve
      pos = cubicBezier(controlNodes, moveToPoseTime);   
      //cout << "LS: " << l << s << " Time: " << moveToPoseTime << " ORIGIN: " << originJointPositions[l][s] << " CURRENT: " << pos << " TARGET: " << targetJointPositions[l][s] << endl;
      
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
  if (forceSequentialMode)
    mode = SEQUENTIAL_MODE;
  else if (sequenceStep == 1)
    mode = startHeightRatio < 0.1 ? NO_STEP_MODE:SEQUENTIAL_MODE;
  else if (sequenceStep == 3)
    mode = startHeightRatio > 0.8 ? TRIPOD_MODE:SEQUENTIAL_MODE;
  
  if (sequenceStep == 1 && startHeightRatio > 0.8)
    sequenceStep = 3;    
  
  bool res = false;
  switch (sequenceStep)
  {
    case 1:
      res = stepToPosition(phase1TipPositions, mode, stepHeight, 0.5);
      break;
    case 2:
      res = stepToPosition(phase2TipPositions, NO_STEP_MODE, 0.0, 0.5);
      break;
    case 3:
      res = stepToPosition(phase3TipPositions, mode, stepHeight, 1.0);
      break;
    case 4:
      res = stepToPosition(phase4TipPositions, NO_STEP_MODE, 0.0, 1.0);
      break;
    case 5:
      sequenceStep = 1;
      return true;
    default:
      return false;
  }
  
  if (res) 
    sequenceStep++;
  
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
      res = stepToPosition(phase5TipPositions, NO_STEP_MODE, 0.0, 1.0);
      break;
    case 2:
      res = stepToPosition(phase6TipPositions, SEQUENTIAL_MODE, stepHeight, 1.0);
      break;
    case 3:
      res = stepToPosition(phase7TipPositions, NO_STEP_MODE, 0.0, 0.5);
      break;
    case 4:
      sequenceStep = 1;
      return true;
    default:      
      return false;
  }
  
  if (res) 
    sequenceStep++;
  
  return false;
}

/***********************************************************************************************************************
 * Calculates tip positions for startup/shutdown sequences
***********************************************************************************************************************/
double PoseController::createSequence(WalkController walker)
{  
  //Get average z position of leg tips
  double averageTipZ = 0.0;
  for (int l=0; l<3; l++)
    for (int s=0; s<2; s++)
      averageTipZ += model->legs[l][s].localTipPosition[2];
  averageTipZ /= 6.0;
  
  //Ratio of average tip z position to required default tip z position 
  double startHeightRatio = averageTipZ/walker.identityTipPositions[0][0][2];
  if (startHeightRatio < 0.0)
    startHeightRatio = 0.0;
  
  double liftAngle = pi/3;
  double  desKneeAngle = liftAngle+asin(model->legs[0][0].femurLength/(model->legs[0][0].tibiaLength*sqrt(2)));
  Vector3d minStartTipPositions = model->legs[0][0].calculateFK(0.77,liftAngle,desKneeAngle);
  double startStanceRatio = minStartTipPositions.squaredNorm()/walker.identityTipPositions[0][0].squaredNorm();
 
  double heightScaler = 0.85;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      double positionScaler = (-(startStanceRatio-1.0)*startHeightRatio+startStanceRatio);
      phase1TipPositions[l][s] = walker.identityTipPositions[l][s]*positionScaler;
      phase1TipPositions[l][s][2] = -0.025; //parameterise
      
      phase2TipPositions[l][s] = phase1TipPositions[l][s];
      phase2TipPositions[l][s][2] = (heightScaler+startHeightRatio*(1-heightScaler))*walker.identityTipPositions[0][0][2];
      
      phase3TipPositions[l][s] = walker.identityTipPositions[l][s];
      phase3TipPositions[l][s][2] = phase2TipPositions[l][s][2];
      
      phase4TipPositions[l][s] = walker.identityTipPositions[l][s];
      
      phase5TipPositions[l][s] = model->legs[l][s].localTipPosition;
      phase5TipPositions[l][s][2] *= heightScaler;
      
      phase6TipPositions[l][s] = walker.identityTipPositions[l][s]*1.3;
      phase6TipPositions[l][s][2] = walker.identityTipPositions[l][s][2];
      
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
 * Calculates pitch for body compensation
***********************************************************************************************************************/
double PoseController::getPitchCompensation(double phase)
{
  double pitch;
  double buffer = params.phaseOffset/2;
  double phaseOffset = params.phaseOffset;
  double p0[2] = {0*phaseOffset, -params.pitchAmplitude};
  double p1[2] = {1*phaseOffset + buffer, -params.pitchAmplitude};
  double p2[2] = {2*phaseOffset + buffer, params.pitchAmplitude};
  double p3[2] = {4*phaseOffset + buffer, params.pitchAmplitude};
  double p4[2] = {5*phaseOffset + buffer, -params.pitchAmplitude};
  double p5[2] = {6*phaseOffset, -params.pitchAmplitude};
    
  if (phase >= p0[0] && phase < p1[0])
    pitch = p0[1];
  else if (phase >= p1[0] && phase < p2[0])
  {
    double gradient = (p2[1]-p1[1])/(p2[0]-p1[0]);
    double offset = ((p2[0]-p1[0])/2 + p1[0]);
    pitch = gradient*phase - gradient*offset;   //-2*phase/3 + 4;
  }
  else if (phase >= p2[0] && phase < p3[0])
    pitch = p2[1];
  else if (phase >= p3[0] && phase < p4[0])
  {
    double gradient = (p4[1]-p3[1])/(p4[0]-p3[0]);
    double offset = ((p4[0]-p3[0])/2 + p3[0]);
    pitch = gradient*phase - gradient*offset;   //2*phase/3 - 10;
  }
  else if (phase >= p4[0] && phase < p5[0])
    pitch = p4[1];    
  
  return pitch;    
}

/***********************************************************************************************************************
 * Calculates roll for body compensation
***********************************************************************************************************************/
double PoseController::getRollCompensation(double phase)
{ 
  double roll;
  double buffer = params.swingPhase/2.25;
  double phaseOffset = params.phaseOffset;
  double p0[2] = {0, -params.rollAmplitude};           
  double p1[2] = {0, -params.rollAmplitude}; 
  double p2[2] = {0, params.rollAmplitude};
  double p3[2] = {0, params.rollAmplitude};
  double p4[2] = {0, -params.rollAmplitude};
  double p5[2] = {0, -params.rollAmplitude};
  
  if (params.gaitType == "tripod_gait")
  {
    p0[0] = 0*phaseOffset;           
    p1[0] = 0*phaseOffset + buffer; 
    p2[0] = 1*phaseOffset - buffer;
    p3[0] = 1*phaseOffset + buffer;
    p4[0] = 2*phaseOffset - buffer;
    p5[0] = 2*phaseOffset;
  }
  else if (params.gaitType == "wave_gait")
  {
    p0[0] = 0*phaseOffset;           
    p1[0] = 0*phaseOffset + buffer; 
    p2[0] = 1*phaseOffset - buffer;
    p3[0] = 3*phaseOffset + buffer;
    p4[0] = 4*phaseOffset - buffer;
    p5[0] = 6*phaseOffset;
  }
  else
    return 0.0;
    
  if (phase >= p0[0] && phase < p1[0])
    roll = p0[1];
  else if (phase >= p1[0] && phase < p2[0])
  {
    double gradient = (p2[1]-p1[1])/(p2[0]-p1[0]);
    double offset = ((p2[0]-p1[0])/2 + p1[0]);
    roll = gradient*phase - gradient*offset; //-2*phase + 3;
  }     
  else if (phase >= p2[0] && phase < p3[0])
    roll = p2[1];
  else if (phase >= p3[0] && phase < p4[0])
  {
    double gradient = (p4[1]-p3[1])/(p4[0]-p3[0]);
    double offset = ((p4[0]-p3[0])/2 + p3[0]);
    roll = gradient*phase - gradient*offset; //2*phase - 21;      
  }
  else if (phase >= p4[0] && phase < p5[0])
    roll = p4[1];
  
  return roll;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
