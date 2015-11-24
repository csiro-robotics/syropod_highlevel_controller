
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
***********************************************************************************************************************/