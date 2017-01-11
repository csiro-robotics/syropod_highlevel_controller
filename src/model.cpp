
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
	leg_container_.insert<std::string, Leg3DOF*>(leg->getIDName(), leg);
	break;
      }
      default: //TBD implement other DOF leg types
      {
	ROS_FATAL("Only 3 degree of freedom legs currently supported");
	break;
      }
    }
  }
}

/***********************************************************************************************************************
 * Applies inverse kinematics for target tip positions of legs (not in OFF state) along with deltaZ from impedance cont.
***********************************************************************************************************************/
void Model::updateLocal()
{  
  typedef std::map<std::string, Leg3DOF*>::iterator it_type;
  for (it_type it = leg_container_.begin(); it != leg_container_.end(); ++it)
  {
    Leg3DOF* leg = it->second();
    LegPoser* leg_poser = leg->getLegPoser();
    Vector3d target_tip_position = leg_poser->getCurrentTipPosition();
    if (leg->getLegState() != MANUAL)  // Don't apply delta Z to manually manipulated legs
    {
      target_tip_position[2] -= leg->getDeltaZ();
    }
    leg->applyLocalIK(target_tip_position);
  }
}

/***********************************************************************************************************************
 * Get locally referenced position values for location of joints from pose (root, hip, knee, tip)
***********************************************************************************************************************/
vector<Vector3d> Model::getJointPositions(const Pose &pose)
{
  //TBD Refactor
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
void Model::clampToLimits(void)
{
  // Clamp desired joint positions and alert if a limit has been hit
  vector<Vector3d> positions;
  typedef std::map<std::string, Leg3DOF*>::iterator leg_it_type;
  for (leg_it_type leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg3DOF* leg = leg_it->second();    
    typedef std::map<std::string, Joint*> joint_container_type;
    joint_container_type::iterator joint_it_type;
    joint_container_type* joints = leg->getJointContainer();
    
    for (joint_it_type joint_it = joints->begin(); joint_it != joints->end(); ++joint_it)
    {
      Joint* joint = joint_it->second();
      double desired_position = joint->getDesiredPosition();
      double min_position = joint->getMinPosition();
      double max_position = joint->getMaxPosition();
      if (desired_position < min_position)
      {
	joint->setDesiredPosition(min_position);
	double diff = abs(desired_position - min_position);
	ROS_WARN_COND(diff > message_tolerance, 
		      "%s leg has tried to exceed %s min joint limit: %f by %f. Clamping joint to limit.\n", 
		      leg->getIDName().c_str(), joint->getName().c_str(), min_position, diff);
      }      
      else if (desired_position > max_position)
      {
	joint->setDesiredPosition(max_position);
	double diff = abs(desired_position - max_position);
	ROS_WARN_COND(diff > message_tolerance, 
		      "%s leg has tried to exceed %s max joint limit: %f by %f. Clamping joint to limit.\n", 
		      leg->getIDName().c_str(), joint->getName().c_str(), max_position, diff);
      }      
    }
  }
}

Leg::Leg(Model model, int id_number, Parameters* params) 
  : model_(model)
  , id_number_(id_number)
  , id_name_(params->leg_id_names[id_number])
  , leg_state_(WALKING)
{
  mirror_dir_ = pow(-1.0, (id_number_%2)+1);  
}


/***********************************************************************************************************************
 * Initialises leg by calculating leg component lengths and applying forward kinematics for tip position
***********************************************************************************************************************/
Leg3DOF::Leg3DOF(Model* model, int id_number, Parameters* params) 
  : coxa_offset_(params.coxa_offset[i])
  , femur_offset_(params.femur_offset[i])
  , tibia_offset_(params.tibia_offset[i])
  , tip_offset_(params.tip_offset[i])
  , Leg(model, id_number, params)
{
  leg_type_ = static_cast<LegType>(params.leg_DOF[i]);
  tripod_group_ = (id_number > 1) ? abs(id_number%2-1):id_number; //(0,3,5 -> 0) && (1,2,4 -> 1)
  num_joints_ = 3;
  for (int i = 0; i < num_joints_; ++i)
  {
    Joint* new_joint = new Joint(this, i, params);
    Link* new_link = new Link(this, i, params);
    joint_container_.insert<std::string, Joint*>(new_joint->getIDName(), new_joint);
    link_container_.insert<std::string, Link*>(new_link->getIDName(), new_link);
  }  
  
  
  
  double femur_link_length = link_container_[params->leg_id_names[1]]->getLinkLength();
  double tibia_link_length = link_container_[params->leg_id_names[2]]->getLinkLength();
  double min_femur_angle = joint_container_[params->leg_id_names[1]]->getMinPosition();
  double max_tibia_angle = joint_container_[params->leg_id_names[2]]->getMaxPosition();
  double min_tibia_angle = joint_container_[params->leg_id_names[2]]->getMinPosition();
  min_leg_length_ = sqrt(sqr(tibia_link_length) + sqr(femur_link_length) -
                      2.0 * femur_link_length * tibia_link_length * cos(max(0.0, pi - max_tibia_angle)));
  max_leg_length_ = sqrt(sqr(tibia_link_length) + sqr(femur_link_length) -
                      2.0 * femur_link_length * tibia_link_length * cos(pi - max(0.0, min_tibia_angle)));
  
  init(0.0, mirror_dir_*max(0.0, min_femur_angle), mirror_dir_*max(0.0, min_tibia_angle));
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
Vector3d Leg3DOF::applyLocalIK(Vector3d target_tip_position)
{
  // application of cosine rule
  Vector3d target = target_tip_position;
  target[1] *= mirror_dir_;
  target -= coxa_offset_;  // since rootOffset is fixed in root's space
  coxa_joint_position_ = atan2(target[0], -target[1]);
  Quat quat(Vector3d(0, 0, coxa_joint_position_));
  target = quat.inverseRotateVector(target);  // localise

  target -= femur_offset_;
  ASSERT(abs(target[0]) < 0.01);
  target[0] = 0;  // any offset here cannot be reached
  double target_length = target.norm();
  double target_angle_offset = atan2(target[2], -target[1]);

  target_length = clamped(target_length, min_leg_length_ + 1e-4, max_leg_length_ - 1e-4);  // reachable range
  double lift = acos((sqr(target_length) + sqr(femur_link_length_) - sqr(tibia_link_length_)) / (2.0 * target_length * femur_link_length_));
  femur_joint_position_ = target_angle_offset + lift;
  double knee_bend =
      acos(-(sqr(femur_link_length_) + sqr(tibia_link_length_) - sqr(target_length)) / (2.0 * femur_link_length_ * tibia_link_length_));
  tibia_joint_position_ = tibia_angle_offset_ + knee_bend;
  ASSERT(abs(yaw) < 7.0);
  ASSERT(abs(liftAngle) < 7.0);
  ASSERT(abs(kneeAngle) < 7.0);

  Vector3d result_tip_position = applyFK();

  // Debugging Error Check: Any error occurs due to an imperfect vector rotation algorithm
  Vector3d diff_vec = result_tip_position - target_tip_position;
  double error_threshold = 1e-3;
  if (diff_vec[0] > error_threshold || diff_vec[1] > error_threshold || diff_vec[2] > error_threshold)
  {
    ROS_WARN("FORWARD KINEMATICS ERROR: %f:%f:%f\n", diff_vec[0], diff_vec[1], diff_vec[2]);
  }
  
  return Vector3d(coxa_joint_position_, femur_joint_position_, tibia_joint_position_);
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

Joint::Joint(Leg* leg, int id_number, Parameters* params)
{
  
  

  femur_angle_offset_ = atan2(tibia_offset_[2], -tibia_offset_[1]);
  tibia_angle_offset_ = atan2(tip_offset_[2], -tip_offset_[1]);
  
  default_coxa_joint_position_ = params.stance_coxa_angles[i/2];
  min_coxa_joint_limit_ = -params.coxa_joint_limits[i/2]; //In reference to default coxa joint position
  max_coxa_joint_limit_ = params.coxa_joint_limits[i/2];
  min_femur_joint_limit_ = params.femur_joint_limits[0]; //In reference to zero femur joint position
  max_femur_joint_limit_ = params.femur_joint_limits[1];
  min_tibia_joint_limit_ = params.tibia_joint_limits[0]; //In reference to zero tibia joint position
  max_tibia_joint_limit_ = params.tibia_joint_limits[1];
  
}

Link::Link(Leg* leg, int id_number, Parameters* params)
{  
  coxa_link_length_(femur_offset_.norm());
  femur_link_length_(tibia_offset_.norm());
  tibia_link_length_(tip_offset_.norm());
}

/***********************************************************************************************************************
***********************************************************************************************************************/
