
#include "../include/simple_hexapod_controller/model.h"

/***********************************************************************************************************************
 * Initialises leg by calculating leg component lengths and applying forward kinematics for tip position
***********************************************************************************************************************/
void Leg::init(double startYaw, double startLiftAngle, double startKneeAngle)
{
  yaw = startYaw;
  liftAngle = startLiftAngle;
  kneeAngle = startKneeAngle;
  hipLength = hipOffset.norm();
  femurLength = kneeOffset.norm();
  femurAngleOffset = atan2(kneeOffset[2], -kneeOffset[1]);
  tibiaLength = tipOffset.norm();
  tibiaAngleOffset = atan2(tipOffset[2], -tipOffset[1]);
  minLegLength = sqrt(sqr(tibiaLength) + sqr(femurLength) - 
    2.0*femurLength*tibiaLength*cos(max(0.0, pi-model->minMaxKneeBend[1]))); 
  maxLegLength = sqrt(sqr(tibiaLength) + sqr(femurLength) - 
    2.0*femurLength*tibiaLength*cos(pi-max(0.0, model->minMaxKneeBend[0]))); 
  applyFK();
}

/***********************************************************************************************************************
 * Applies inverse kinematics to achieve target tip position
***********************************************************************************************************************/
Vector3d Leg::applyLocalIK(Vector3d tipTarget)
{
  // application of cosine rule
  Vector3d target = tipTarget;
  target[1] *= mirrorDir;
  target -= rootOffset; // since rootOffset is fixed in root's space
  yaw = atan2(target[0], -target[1]);
  Quat quat(Vector3d(0,0,yaw));
  target = quat.inverseRotateVector(target); // localise
  
  target -= hipOffset;
  ASSERT(abs(target[0]) < 0.01);
  target[0] = 0; // any offset here cannot be reached
  double targetLength = target.norm();
  double targetAngleOffset = atan2(target[2], -target[1]);
  
  targetLength = clamped(targetLength, minLegLength + 1e-4, maxLegLength - 1e-4); // reachable range
  double lift = acos((sqr(targetLength)+sqr(femurLength)-sqr(tibiaLength))/(2.0*targetLength*femurLength));
  liftAngle = targetAngleOffset + lift;
  double kneeBend = acos(-(sqr(femurLength)+sqr(tibiaLength)-sqr(targetLength))/(2.0*femurLength*tibiaLength));
  kneeAngle = tibiaAngleOffset + kneeBend;
  ASSERT(abs(yaw) < 7.0);
  ASSERT(abs(liftAngle) < 7.0);
  ASSERT(abs(kneeAngle) < 7.0); 
  
  Vector3d resultTipPosition = applyFK();
 
  //Debugging Error Check: Any error occurs due to an imperfect vector rotation algorithm 
  Vector3d diffVec = resultTipPosition - tipTarget;  
  double errorThreshold = 1e-3;
  if (diffVec[0] > errorThreshold || diffVec[1] > errorThreshold || diffVec[2] > errorThreshold)
  {
    ROS_WARN("FORWARD KINEMATICS ERROR: %f:%f:%f\n", diffVec[0], diffVec[1], diffVec[2]);  
  }
  
  Vector3d jointPositions = {yaw, liftAngle, kneeAngle};
  return jointPositions;
}

/***********************************************************************************************************************
 * Applies forward kinematics
***********************************************************************************************************************/
Vector3d Leg::applyFK()
{
  localTipPosition = calculateFK(yaw, liftAngle, kneeAngle);
  model->localTipPositions[legIndex][sideIndex] = localTipPosition;
  return localTipPosition;
}

/***********************************************************************************************************************
 * Calculates forward kinematics
***********************************************************************************************************************/
Vector3d Leg::calculateFK(double yaw, double liftAngle, double kneeAngle)
{
  Vector3d tipPosition;
  tipPosition = tipOffset;
  tipPosition = Quat(Vector3d(kneeAngle, 0, 0)).rotateVector(tipPosition) + kneeOffset;
  tipPosition = Quat(Vector3d(-liftAngle, 0, 0)).rotateVector(tipPosition) + hipOffset;
  tipPosition = Quat(Vector3d(0, 0, yaw)).rotateVector(tipPosition) + rootOffset;
  tipPosition[1] *= mirrorDir;
  
  return tipPosition;
}

/***********************************************************************************************************************
 * Sets leg state
***********************************************************************************************************************/
void Leg::setState(LegState newLegState)
{
  legState = newLegState;
  model->legStates[legIndex][sideIndex] = legState;
}

/***********************************************************************************************************************
 * Defines hexapod model
***********************************************************************************************************************/
Model::Model(Parameters params) : 
  stanceLegYaws(params.stanceLegYaws), 
  yawLimitAroundStance(params.yawLimits), 
  minMaxKneeBend(params.kneeLimits), 
  minMaxHipLift(params.hipLimits), 
  jointMaxAngularSpeeds(params.jointMaxAngularSpeeds)
{
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      Leg &leg = legs[l][s];
      leg.model = this;
      leg.legIndex = l;
      leg.sideIndex = s;
      leg.rootOffset = params.rootOffset[l][s];
      leg.hipOffset  = params.hipOffset[l][s];
      leg.kneeOffset = params.kneeOffset[l][s];
      leg.tipOffset  = params.tipOffset[l][s];
      leg.mirrorDir = s ? 1 : -1;
      leg.init(0,max(0.0,minMaxHipLift[0]),max(0.0,minMaxKneeBend[0]));
      leg.setState(WALKING);
    }
  }
}

/***********************************************************************************************************************
 * Set individual leg joint angles
***********************************************************************************************************************/
void Model::setLegStartAngles(int side, int leg, const Vector3d &startAngles)
{
  legs[leg][side].init(startAngles[0], startAngles[1], startAngles[2]);
}

/***********************************************************************************************************************
 * Applies inverse kinematics for target tip positions of legs (not in OFF state) along with deltaZ from impedance cont.
***********************************************************************************************************************/
void Model::updateLocal(Vector3d targetTipPositions[3][2], double deltaZ[3][2])
{
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      Vector3d adjustedPos = targetTipPositions[l][s];
      if (legs[l][s].legState != MANUAL) //Don't apply delta Z to manually manipulated legs
      {
	adjustedPos[2] = targetTipPositions[l][s][2] - deltaZ[l][s];
      }
      legs[l][s].applyLocalIK(adjustedPos);      
    }
  }  
}

/***********************************************************************************************************************
 * Get locally referenced position values for location of joints from pose (root, hip, knee, tip)
***********************************************************************************************************************/
vector<Vector3d> Model::getJointPositions(const Pose &pose)
{
  vector<Vector3d> positions;
  for (int s = 0; s<2; s++)
  {
    for (int l = 0; l<3; l++)
    {
      Leg &leg = legs[l][s];
      Pose transform;
      transform = Pose(leg.rootOffset, Quat(Vector3d(0, 0, leg.yaw)));
      positions.push_back(pose.transformVector(Vector3d(transform.position[0], transform.position[1]*leg.mirrorDir, transform.position[2])));
      transform *= Pose(leg.hipOffset, Quat(Vector3d(-leg.liftAngle, 0, 0)));
      positions.push_back(pose.transformVector(Vector3d(transform.position[0], transform.position[1]*leg.mirrorDir, transform.position[2])));
      transform *= Pose(leg.kneeOffset, Quat(Vector3d(leg.kneeAngle, 0, 0)));
      positions.push_back(pose.transformVector(Vector3d(transform.position[0], transform.position[1]*leg.mirrorDir, transform.position[2])));
      transform *= Pose(leg.tipOffset, Quat(Vector3d(0, 0, 0)));
      positions.push_back(pose.transformVector(Vector3d(transform.position[0], transform.position[1]*leg.mirrorDir, transform.position[2])));
      ASSERT(positions.back().squaredNorm() < 1000.0);
    }
  }
  return positions;
}

/***********************************************************************************************************************
 * Restrict joint angles to limits
***********************************************************************************************************************/
void Model::clampToLimits(std::map<int, std::string> legNameMap)
{
  // clamp angles and alert if a limit has been hit
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      Leg &leg = legs[l][s];
      double messageTolerance = 0.017444; //1 degree over limit before warning message
      if (leg.yaw - stanceLegYaws[l] < -yawLimitAroundStance[l])
      {
	double diff = abs(leg.yaw - (-yawLimitAroundStance[l] + stanceLegYaws[l]));
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed body_coxa joint limit: %f by %f. Clamping body_coxa joint to limit.\n", legNameMap[l*2+s].c_str(), -yawLimitAroundStance[l] + stanceLegYaws[l], diff);
	}
	leg.yaw = -yawLimitAroundStance[l] + stanceLegYaws[l];
      }
      else if (leg.yaw - stanceLegYaws[l] > yawLimitAroundStance[l])
      {
	double diff = abs(leg.yaw - (yawLimitAroundStance[l] + stanceLegYaws[l]));
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed body_coxa joint limit: %f by %f. Clamping body_coxa joint to limit.\n", legNameMap[l*2+s].c_str(), yawLimitAroundStance[l] + stanceLegYaws[l], diff);	  
	}
	leg.yaw = yawLimitAroundStance[l] + stanceLegYaws[l];        
      }
      if (leg.liftAngle < minMaxHipLift[0])
      {
	double diff = abs(leg.liftAngle - minMaxHipLift[0]);
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed coxa_femur joint limit: %f by %f. Clamping coxa_femur joint to limit.\n", legNameMap[l*2+s].c_str(), minMaxHipLift[0], diff);
	}
	leg.liftAngle = minMaxHipLift[0];
      }
      else if (leg.liftAngle > minMaxHipLift[1])
      {
	double diff = abs(leg.liftAngle - minMaxHipLift[1]);
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed coxa_femur joint limit: %f by %f. Clamping coxa_femur joint to limit.\n", legNameMap[l*2+s].c_str(), minMaxHipLift[1], diff);
	}
	leg.liftAngle = minMaxHipLift[1];
      }
      if (leg.kneeAngle < minMaxKneeBend[0])
      {
	double diff = abs(leg.kneeAngle - minMaxKneeBend[0]);
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed femur_tibia joint limit: %f by %f. Clamping femur_tibia joint to limit.\n", legNameMap[l*2+s].c_str(), minMaxKneeBend[0], diff);
	}
	leg.kneeAngle = minMaxKneeBend[0];
      }
      else if (leg.kneeAngle > minMaxKneeBend[1])
      {
	double diff = abs(leg.kneeAngle - minMaxKneeBend[1]);
	if (diff > messageTolerance)
	{
	  ROS_WARN("%s leg has tried to exceed femur_tibia joint limit: %f by %f. Clamping femur_tibia joint to limit.\n", legNameMap[l*2+s].c_str(), minMaxKneeBend[1], diff);
	}
	leg.kneeAngle = minMaxKneeBend[1];
      }
    }
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/

