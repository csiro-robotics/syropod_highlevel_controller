
#include "../include/simple_hexapod_controller/controller.h"

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
bool PoseController::updatePose(Vector3d targetTipPositions[3][2], Pose requestedTargetPose, double timeToPose, bool moveLegsSequentially)
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
        originTipPosition[l][s] = model->legs[l][s].stanceTipPosition; 
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
    
    Vector3d currentTipPosition = originTipPosition[l][s];
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
        
        Vector3d currentTipPosition = originTipPosition[l][s];
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
        originTipPosition[l][s] = model->legs[l][s].stanceTipPosition;
    return true;
  }
  
  return false;    
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