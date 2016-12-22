
#include "../include/simple_hexapod_controller/model.h"

/***********************************************************************************************************************
 * Defines hexapod model
***********************************************************************************************************************/
Model::Model(Parameters params)
  : joint_max_angular_speeds_(params.joint_max_angular_speeds)
  , leg_count_(params.num_legs)
{
  for (int i=0; i < params.num_legs; ++i)
  {   
    LegType leg_type = static_cast<LegType>(params.leg_DOF[i]);
    switch(leg_type) //Check leg type to add to model
    {
      case (3_DOF):
      {
	Leg3DOF* leg = new Leg3DOF(this, i, params);
	leg_container_.insert<i, Leg3DOF*>(i, leg);
	leg->init(0, max(0.0, min_max_femur_angle_[0]), max(0.0, min_max_tibia_angle_[0])); 
	break;
      }
      default: //TBD implement other DOF leg types
      {
	ROS_FATAL("Only 3 degree of freedom legs currently supported");
	break;
      }
    }
  }
  
  /*
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      Leg &leg = legs_[l][s];
      leg.model_ = this;
      leg.leg_index_ = l;
      leg.side_index_ = s;
      leg.coxa_offset_ = params.coxa_offset[l][s];
      leg.femur_offset_ = params.femur_offset[l][s];
      leg.tibia_offset_ = params.tibia_offset[l][s];
      leg.tip_offset_ = params.tip_offset[l][s];
      leg.mirror_dir_ = s ? 1 : -1;
      leg.init(0, max(0.0, min_max_femur_angle_[0]), max(0.0, min_max_tibia_angle_[0]));
      leg.setState(WALKING);
    }
  }
  */
}

/***********************************************************************************************************************
 * Applies inverse kinematics for target tip positions of legs (not in OFF state) along with deltaZ from impedance cont.
***********************************************************************************************************************/
void Model::updateLocal(Vector3d targetTipPositions[3][2], double deltaZ[3][2])
{
  typedef std::map<std::string, Leg3DOF*>::iterator it_type;
  for (it_type it = leg_container_.begin(); it != leg_container_.end(); ++it)
  {
    Leg3DOF leg = *it->second();
    Vector3d adjustedPos = targetTipPositions[l][s];
    if (leg.getLegState() != MANUAL)  // Don't apply delta Z to manually manipulated legs
    {
      adjustedPos[2] = targetTipPositions[l][s][2] - deltaZ[l][s];
    }
    leg.applyLocalIK(adjustedPos);
  }
}

/***********************************************************************************************************************
 * Get locally referenced position values for location of joints from pose (root, hip, knee, tip)
***********************************************************************************************************************/
vector<Vector3d> Model::getJointPositions(const Pose &pose)
{
  vector<Vector3d> positions;
  typedef std::map<std::string, Leg3DOF*>::iterator it_type;
  for (it_type it = leg_container_.begin(); it != leg_container_.end(); ++it)
  {
    Leg3DOF leg = *it->second();
    Pose transform;
    transform = Pose(leg.coxa_offset_, Quat(Vector3d(0, 0, leg.coxa_joint_position_)));
    positions.push_back(pose.transformVector(
	Vector3d(transform.position_[0], transform.position_[1] * leg.mirror_dir_, transform.position_[2])));
    transform *= Pose(leg.femur_offset_, Quat(Vector3d(-leg.femur_joint_position_, 0, 0)));
    positions.push_back(pose.transformVector(
	Vector3d(transform.position_[0], transform.position_[1] * leg.mirror_dir_, transform.position_[2])));
    transform *= Pose(leg.tibia_offset_, Quat(Vector3d(leg.tibia_joint_position_, 0, 0)));
    positions.push_back(pose.transformVector(
	Vector3d(transform.position_[0], transform.position_[1] * leg.mirror_dir_, transform.position_[2])));
    transform *= Pose(leg.tip_offset_, Quat(Vector3d(0, 0, 0)));
    positions.push_back(pose.transformVector(
	Vector3d(transform.position_[0], transform.position_[1] * leg.mirror_dir_, transform.position_[2])));
    ASSERT(positions.back().squaredNorm() < 1000.0);
  }
  return positions;
}

/***********************************************************************************************************************
 * Restrict joint angles to limits
***********************************************************************************************************************/
void Model::clampToLimits(std::map<int, std::string> legNameMap)
{
  // clamp angles and alert if a limit has been hit
  vector<Vector3d> positions;
  typedef std::map<std::string, Leg3DOF*>::iterator it_type;
  for (it_type it = leg_container_.begin(); it != leg_container_.end(); ++it)
  {
    Leg3DOF leg = *it->second();
    double messageTolerance = 0.017444;  // 1 degree over limit before warning message
    if (leg.getCoxaJointPosition() - leg.default_coxa_joint_position_ < leg.min_coxa_joint_limit_around_default_)
    {
      double diff = abs(leg.getCoxaJointPosition() - (-coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l]));
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed body_coxa joint limit: %f by %f. Clamping body_coxa joint to limit.\n",
		  legNameMap[l * 2 + s].c_str(), -coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l], diff);
      }
      leg.coxa_joint_position_ = -coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l];
    }
    else if (leg.coxa_joint_position_ - stance_coxa_angles_[l] > coxa_angle_limit_around_stance_[l])
    {
      double diff = abs(leg.coxa_joint_position_ - (coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l]));
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed body_coxa joint limit: %f by %f. Clamping body_coxa joint to limit.\n",
		  legNameMap[l * 2 + s].c_str(), coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l], diff);
      }
      leg.coxa_joint_position_ = coxa_angle_limit_around_stance_[l] + stance_coxa_angles_[l];
    }

    if (leg.femur_joint_position_ < min_max_femur_angle_[0])
    {
      double diff = abs(leg.femur_joint_position_ - min_max_femur_angle_[0]);
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed coxa_femur joint limit: %f by %f. Clamping coxa_femur joint to limit.\n",
		  legNameMap[l * 2 + s].c_str(), min_max_femur_angle_[0], diff);
      }
      leg.femur_joint_position_ = min_max_femur_angle_[0];
    }
    else if (leg.femur_joint_position_ > min_max_femur_angle_[1])
    {
      double diff = abs(leg.femur_joint_position_ - min_max_femur_angle_[1]);
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed coxa_femur joint limit: %f by %f. Clamping coxa_femur joint to limit.\n",
		  legNameMap[l * 2 + s].c_str(), min_max_femur_angle_[1], diff);
      }
      leg.femur_joint_position_ = min_max_femur_angle_[1];
    }
    if (leg.tibia_joint_position_ < min_max_tibia_angle_[0])
    {
      double diff = abs(leg.tibia_joint_position_ - min_max_tibia_angle_[0]);
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed femur_tibia joint limit: %f by %f. Clamping femur_tibia joint to "
		  "limit.\n",
		  legNameMap[l * 2 + s].c_str(), min_max_tibia_angle_[0], diff);
      }
      leg.tibia_joint_position_ = min_max_tibia_angle_[0];
    }
    else if (leg.tibia_joint_position_ > min_max_tibia_angle_[1])
    {
      double diff = abs(leg.tibia_joint_position_ - min_max_tibia_angle_[1]);
      if (diff > messageTolerance)
      {
	ROS_WARN("%s leg has tried to exceed femur_tibia joint limit: %f by %f. Clamping femur_tibia joint to "
		  "limit.\n",
		  legNameMap[l * 2 + s].c_str(), min_max_tibia_angle_[1], diff);
      }
      leg.tibia_joint_position_ = min_max_tibia_angle_[1];
    }
  }
}

/***********************************************************************************************************************
 * Initialises leg by calculating leg component lengths and applying forward kinematics for tip position
***********************************************************************************************************************/
Leg3DOF::Leg3DOF(Model model, int i, Parameters p) 
  : model_(model)
  , id_number_(i)
  , id_name_(p.leg_id_names[i])
  , coxa_offset_(p.coxa_offset[i])
  , femur_offset_(p.femur_offset[i])
  , tibia_offset_(p.tibia_offset[i])
  , tip_offset_(p.tip_offset[i])
  , leg_state_(WALKING)
{
  leg_type_ = static_cast<LegType>(p.leg_DOF[i]);
    
  mirror_dir_ = pow(-1.0, (id_number_%2)+1); //Id == even -> left side -> negative mirror direction
  
  tripod_group_ = (i > 1) ? abs(i%2-1):i; //(0,3,5 -> 0) && (1,2,4 -> 1)
  
  coxa_link_length_(femur_offset_.norm());
  femur_link_length_(tibia_offset_.norm());
  tibia_link_length_(tip_offset_.norm());
  
  min_leg_length_ = sqrt(sqr(tibia_link_length_) + sqr(femur_link_length_) -
                      2.0 * femur_link_length_ * tibia_link_length_ * cos(max(0.0, pi - model_->min_max_tibia_angle_[1])));
  max_leg_length_ = sqrt(sqr(tibia_link_length_) + sqr(femur_link_length_) -
                      2.0 * femur_link_length_ * tibia_link_length_ * cos(pi - max(0.0, model_->min_max_tibia_angle_[0])));
  femur_angle_offset_ = atan2(tibia_offset_[2], -tibia_offset_[1]);
  tibia_angle_offset_ = atan2(tip_offset_[2], -tip_offset_[1]);
  
  default_coxa_joint_position_ = p.stance_coxa_angles[i/2];
  min_coxa_joint_limit_around_default_ = -p.coxa_joint_limits[i/2]; //In reference to default coxa joint position
  max_coxa_joint_limit_around_default_ = p.coxa_joint_limits[i/2];
  min_femur_joint_limit_ = p.femur_joint_limits[0]; //In reference to zero femur joint position
  max_femur_joint_limit_ = p.femur_joint_limits[1];
  min_tibia_joint_limit_ = p.tibia_joint_limits[0]; //In reference to zero tibia joint position
  max_tibia_joint_limit_ = p.tibia_joint_limits[1];
}


void Leg3DOF::init(double initial_coxa_angle, double initial_femur_angle, double initial_tibia_angle)
{
  coxa_joint_position_ = initial_coxa_angle;
  femur_joint_position_ = initial_femur_angle;
  tibia_joint_position_ = initial_tibia_angle;
  applyFK();
}

/***********************************************************************************************************************
 * Applies inverse kinematics to achieve target tip position
***********************************************************************************************************************/
Vector3d Leg3DOF::applyLocalIK(Vector3d tipTarget)
{
  // application of cosine rule
  Vector3d target = tipTarget;
  target[1] *= mirror_dir_;
  target -= coxa_offset_;  // since rootOffset is fixed in root's space
  coxa_joint_position_ = atan2(target[0], -target[1]);
  Quat quat(Vector3d(0, 0, coxa_joint_position_));
  target = quat.inverseRotateVector(target);  // localise

  target -= femur_offset_;
  ASSERT(abs(target[0]) < 0.01);
  target[0] = 0;  // any offset here cannot be reached
  double targetLength = target.norm();
  double targetAngleOffset = atan2(target[2], -target[1]);

  targetLength = clamped(targetLength, min_leg_length_ + 1e-4, max_leg_length_ - 1e-4);  // reachable range
  double lift = acos((sqr(targetLength) + sqr(femur_link_length_) - sqr(tibia_link_length_)) / (2.0 * targetLength * femur_link_length_));
  femur_joint_position_ = targetAngleOffset + lift;
  double kneeBend =
      acos(-(sqr(femur_link_length_) + sqr(tibia_link_length_) - sqr(targetLength)) / (2.0 * femur_link_length_ * tibia_link_length_));
  tibia_joint_position_ = tibia_angle_offset_ + kneeBend;
  ASSERT(abs(yaw) < 7.0);
  ASSERT(abs(liftAngle) < 7.0);
  ASSERT(abs(kneeAngle) < 7.0);

  Vector3d resultTipPosition = applyFK();

  // Debugging Error Check: Any error occurs due to an imperfect vector rotation algorithm
  Vector3d diffVec = resultTipPosition - tipTarget;
  double errorThreshold = 1e-3;
  if (diffVec[0] > errorThreshold || diffVec[1] > errorThreshold || diffVec[2] > errorThreshold)
  {
    ROS_WARN("FORWARD KINEMATICS ERROR: %f:%f:%f\n", diffVec[0], diffVec[1], diffVec[2]);
  }

  Vector3d jointPositions = { coxa_joint_position_, femur_joint_position_, tibia_joint_position_ };
  return jointPositions;
}

/***********************************************************************************************************************
 * Applies forward kinematics
***********************************************************************************************************************/
Vector3d Leg3DOF::applyFK()
{
  local_tip_position_ = calculateFK(coxa_joint_position_, femur_joint_position_, tibia_joint_position_);
  return local_tip_position_;
}

/***********************************************************************************************************************
 * Calculates forward kinematics
***********************************************************************************************************************/
Vector3d Leg3DOF::calculateFK(double coxa_joint_angle, double femur_joint_angle, double tibia_joint_angle)
{
  Vector3d tipPosition;
  tipPosition = tip_offset_;
  tipPosition = Quat(Vector3d(tibia_joint_angle, 0, 0)).rotateVector(tipPosition) + tibia_offset_;
  tipPosition = Quat(Vector3d(-femur_joint_angle, 0, 0)).rotateVector(tipPosition) + femur_offset_;
  tipPosition = Quat(Vector3d(0, 0, coxa_joint_angle)).rotateVector(tipPosition) + coxa_offset_;
  tipPosition[1] *= mirror_dir_;

  return tipPosition;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
