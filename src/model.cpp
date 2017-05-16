/*******************************************************************************************************************//**
 *  \file    model.cpp
 *  \brief   Describes the hexapod model including all legs, joints and links. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   June 2017
 *  \version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
***********************************************************************************************************************/

#include "../include/simple_hexapod_controller/model.h"

/***********************************************************************************************************************
 * Defines hexapod model
***********************************************************************************************************************/
Model::Model(Parameters* params)
  : leg_count_(params->leg_id.data.size())
  , time_delta_(params->time_delta.data)
  , current_pose_(Pose::identity())
  , linear_velocity_(Vector2d(0, 0))
{
  for (int i = 0; i < leg_count_; ++i)
  {
    Leg* leg = new Leg(this, i, params);
    leg_container_.insert(std::map<int, Leg*>::value_type(i, leg));
  }
}

/***********************************************************************************************************************
 * Initialise all legs in model
***********************************************************************************************************************/
void Model::initLegs(bool use_default_joint_positions)
{
  // Set initial leg angles
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    leg->init(use_default_joint_positions);
  }
}

/***********************************************************************************************************************
 * Estimates if the legs of the robot are bearing the load (i.e. any leg is lower than the plan of the body underside)
***********************************************************************************************************************/
bool Model::legsBearingLoad(void) //TBD Make more robust by estimating body height
{
  // Set initial leg angles
  std::map<int, Leg*>::iterator leg_it;
  double body_height_estimate = 0.0;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    body_height_estimate += leg->getLocalTipPosition()[2];
  }
  double threshold = 0.05; //TBD Parameterise as body thickness
  return -(body_height_estimate / leg_count_) > threshold;
}

/***********************************************************************************************************************
 * Get Leg by name
***********************************************************************************************************************/
Leg* Model::getLegByIDName(std::string leg_name)
{
  map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    if (leg->getIDName() == leg_name)
    {
      return leg;
    }
  }
  return NULL;
}

/***********************************************************************************************************************
 * Generic leg data object
***********************************************************************************************************************/
Leg::Leg(Model* model, int id_number, Parameters* params)
  : model_(model)
  , id_number_(id_number)
  , id_name_(params->leg_id.data[id_number])
  , num_joints_(params->leg_DOF.data[id_name_])
  , mirror_dir_(id_number_ >= 3 ? -1.0:1.0)
  , stance_leg_yaw_(params->leg_stance_yaws.data[id_name_])
  , leg_state_(WALKING)
  , impedance_state_(std::vector<double>(2))
{
  Joint* null_joint; //HACK
  Link* base_link = new Link(this, null_joint, 0, params);
  link_container_.insert(std::map<int, Link*>::value_type(0, base_link));
  ROS_DEBUG("%s successfully added to leg %s in model.", base_link->name.c_str(), id_name_.c_str());
  Link* prev_link = base_link;
  for (int i = 1; i < num_joints_ + 1; ++i)
  {
    Joint* new_joint = new Joint(this, prev_link, i, params);
    Link* new_link = new Link(this, new_joint, i, params);
    joint_container_.insert(std::map<int, Joint*>::value_type(i, new_joint));
    link_container_.insert(std::map<int, Link*>::value_type(i, new_link));
    prev_link = new_link;
    ROS_DEBUG("%s and %s successfully added to leg %s in model.",
              new_link->name.c_str(), new_joint->name.c_str(), id_name_.c_str());
  }
  tip_ = new Tip(this, prev_link);

  Vector3d unassigned_vector(UNASSIGNED_VALUE, UNASSIGNED_VALUE, UNASSIGNED_VALUE);
  local_tip_position_ = unassigned_vector;
  desired_tip_position_ = unassigned_vector;
  desired_tip_velocity_ = Vector3d(0, 0, 0);
  group_ = (id_number % 2); //Even/odd groups

  ROS_DEBUG("Leg %s has been initialised as a %d degree of freedom leg with %lu links and %lu joints.",
            id_name_.c_str(), num_joints_, link_container_.size(), joint_container_.size());
}

/***********************************************************************************************************************
 * Initialises leg by setting desired joint positions to current position from encoders and running forward kinematics
***********************************************************************************************************************/
void Leg::init(bool use_default_joint_positions)
{
  std::map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (use_default_joint_positions && joint->current_position == UNASSIGNED_VALUE)
    {
      joint->current_position = clamped(0.0, joint->min_position, joint->max_position);
      joint->current_velocity = 0.0;
      joint->current_effort = 0.0;
    }
    joint->desired_position = joint->current_position;
    joint->prev_desired_position = joint->desired_position;
  }
  applyFK();
  desired_tip_position_ = local_tip_position_;
}

/***********************************************************************************************************************
 * Search through joint container and return joint object associated with given identification name
***********************************************************************************************************************/
Joint* Leg::getJointByIDName(std::string joint_name)
{
  std::map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (joint->name == joint_name)
    {
      return joint;
    }
  }
  return NULL;
}

/***********************************************************************************************************************
 * Search through link container and return link object associated with given identification name
***********************************************************************************************************************/
Link* Leg::getLinkByIDName(std::string link_name)
{
  std::map<int, Link*>::iterator link_it;
  for (link_it = link_container_.begin(); link_it != link_container_.end(); ++link_it)
  {
    Link* link = link_it->second;
    if (link->name == link_name)
    {
      return link;
    }
  }
  return NULL;
}

/***********************************************************************************************************************
 * Applies impedance controller delta z position within velocity limits
***********************************************************************************************************************/
void Leg::setDesiredTipPosition(Vector3d tip_position, bool apply_delta_z)
{
  desired_tip_position_[0] = tip_position[0];
  desired_tip_position_[1] = tip_position[1];
  
  // Don't apply delta Z to manually manipulated legs
  bool manually_manipulated = (leg_state_ == MANUAL || leg_state_ == WALKING_TO_MANUAL);
  if (apply_delta_z && !manually_manipulated)
  {
    desired_tip_position_[2] = tip_position[2] + delta_z_;
  }
  else
  {
    desired_tip_position_[2] = tip_position[2];
  }
}

/***********************************************************************************************************************
 * Update tip force
***********************************************************************************************************************/
bool Leg::updateTipForce(bool debug) //TBD Not currently used (experiment function)
{
  vector<map<string, double>> dh_parameters;
  map<int, Joint*>::iterator joint_it;
  map<string, double> dh_map;
  //Skip first joint dh parameters since it is a fixed transformation
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    dh_map.insert(map<string, double>::value_type("d", joint->reference_link->offset));
    dh_map.insert(map<string, double>::value_type("theta", joint->reference_link->actuating_joint->desired_position));
    dh_map.insert(map<string, double>::value_type("r", joint->reference_link->length));
    dh_map.insert(map<string, double>::value_type("alpha", joint->reference_link->twist));
    dh_parameters.push_back(dh_map);
    dh_map.clear();
  }
  //Add tip dh params
  dh_map.insert(map<string, double>::value_type("d", tip_->reference_link->offset));
  dh_map.insert(map<string, double>::value_type("theta", tip_->reference_link->actuating_joint->desired_position));
  dh_map.insert(map<string, double>::value_type("r", tip_->reference_link->length));
  dh_map.insert(map<string, double>::value_type("alpha", tip_->reference_link->twist));
  dh_parameters.push_back(dh_map);

  MatrixXd j(3, num_joints_);
  //j = createJacobian(dh_parameters, num_joints_);

  VectorXd joint_torques(num_joints_);
  int index = 0;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    Joint* joint = joint_it->second;
    joint_torques[index] = joint->current_effort;
  }

  //tip_force_ = j * joint_torques; // Estimate force at the tip in frame of first joint

  //ROS_DEBUG_COND(id_number_ == 0 && debug, "Leg: %s\n\tEstimated tip force:\t%f:%f:%f\n", id_name_.c_str(), tip_force_[0], tip_force_[1], tip_force_[2]);
  
  return false;
}
/***********************************************************************************************************************
 * Applies inverse kinematics to achieve target tip position
***********************************************************************************************************************/
double Leg::applyIK(bool debug, bool ignore_warnings, bool clamp_positions, bool clamp_velocities)
{
  vector<map<string, double>> dh_parameters;
  map<int, Joint*>::iterator joint_it;
  map<string, double> dh_map;
  //Skip first joint dh parameters since it is a fixed transformation
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    dh_map.insert(map<string, double>::value_type("d", joint->reference_link->offset));
    dh_map.insert(map<string, double>::value_type("theta", joint->reference_link->actuating_joint->desired_position));
    dh_map.insert(map<string, double>::value_type("r", joint->reference_link->length));
    dh_map.insert(map<string, double>::value_type("alpha", joint->reference_link->twist));
    dh_parameters.push_back(dh_map);
    dh_map.clear();
  }
  //Add tip dh params
  dh_map.insert(map<string, double>::value_type("d", tip_->reference_link->offset));
  dh_map.insert(map<string, double>::value_type("theta", tip_->reference_link->actuating_joint->desired_position));
  dh_map.insert(map<string, double>::value_type("r", tip_->reference_link->length));
  dh_map.insert(map<string, double>::value_type("alpha", tip_->reference_link->twist));
  dh_parameters.push_back(dh_map);

  MatrixXd j(3, num_joints_);
  j = createJacobian(dh_parameters);

  double dls_cooeficient = 0.02; //TBD calculate optimal value (this value currently works sufficiently)
  MatrixXd identity = Matrix3d::Identity();
  MatrixXd ik_matrix(num_joints_, 3);
  //ik_matrix = ((j.transpose()*j).inverse())*j.transpose(); //Pseudo Inverse method
  ik_matrix = j.transpose() * ((j * j.transpose() + sqr(dls_cooeficient) * identity).inverse()); //DLS Method

  Joint* base_joint = joint_container_.begin()->second;
  Vector3d leg_frame_desired_tip_position = base_joint->getPositionJointFrame(false, desired_tip_position_);
  Vector3d leg_frame_prev_desired_tip_position = base_joint->getPositionJointFrame(false, local_tip_position_);
  Vector3d leg_frame_tip_position_delta = leg_frame_desired_tip_position - leg_frame_prev_desired_tip_position;
  VectorXd joint_delta_pos(num_joints_);

  joint_delta_pos = ik_matrix * leg_frame_tip_position_delta;

  int index = 0;
  string clamping_events;
  double limit_proximity = 1.0;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    Joint* joint = joint_it->second;
    joint->desired_velocity = joint_delta_pos[index] / model_->getTimeDelta();
    
    // Clamp joint velocities within limits
    if (clamp_velocities)
    {
      if (abs(joint->desired_velocity) > joint->max_angular_speed)
      {
        clamping_events += stringFormat("\n\tType: Velocity\tJoint: %s\tDesired: %f rad/s\tLimited to: %f rad/s",
                                        joint->name.c_str(), abs(joint->desired_velocity), joint->max_angular_speed);
        joint->desired_velocity = clamped(joint->desired_velocity, -joint->max_angular_speed, joint->max_angular_speed);
      }
    }
    joint->desired_position = joint->prev_desired_position + joint->desired_velocity*model_->getTimeDelta();

    // Clamp joint position within limits
    if (clamp_positions)
    {
      if (joint->desired_position < joint->min_position)
      {
        clamping_events += stringFormat("\n\tType: Position\tJoint: %s\tDesired: %f rad\tLimited to: %f rad",
                                        joint->name.c_str(), joint->desired_position, joint->min_position);
        joint->desired_position = joint->min_position;
      }
      else if (joint->desired_position > joint->max_position)
      {
        clamping_events += stringFormat("\n\tType: Position\tJoint: %s\tDesired: %f rad\tLimited to: %f rad",
                                        joint->name.c_str(), joint->desired_position, joint->max_position);
        joint->desired_position = joint->max_position;
      }
    }
    
    //Calculates the proximity of the joint closest to one of it's limits. Used for preventing exceeding workspace.
    // (1.0 = furthest possible from limit, 0.0 = equal to limit)
    double min_diff = abs(joint->min_position - joint->desired_position);
    double max_diff = abs(joint->max_position - joint->desired_position);
    double half_joint_range = (joint->max_position - joint->min_position)/2.0;
    limit_proximity = min(limit_proximity, min(min_diff, max_diff)/half_joint_range);
  }

  Vector3d result = applyFK();

  // Debugging message
  ROS_DEBUG_COND(id_number_ == 0 && debug,
                 "\nLeg %s:\n\tDesired tip position from trajectory engine: %f:%f:%f\n\t"
                 "Resultant tip position from inverse/forward kinematics: %f:%f:%f", id_name_.c_str(),
                 desired_tip_position_[0], desired_tip_position_[1], desired_tip_position_[2],
                 result[0], result[1], result[2]);

  // Display warning messages for clamping events and associated inverse kinematic deviations
  if (!ignore_warnings)
  {
    Vector3d IK_tolerance(0.005, 0.005, 0.005); //5mm
    std::string axis_label[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
      if (abs(result[i] - desired_tip_position_[i]) > IK_tolerance[i])
      {
        double error_percentage = abs((result[i] - desired_tip_position_[i]) / desired_tip_position_[i]) * 100;
        ROS_WARN("\nInverse kinematics deviation! Calculated tip %s position of leg %s (%s: %f)"
                " differs from desired tip position (%s: %f) by %f%%\nThis is due to clamping events:%s\n",
                axis_label[i].c_str(), id_name_.c_str(), axis_label[i].c_str(), result[i], axis_label[i].c_str(),
                desired_tip_position_[i], error_percentage, clamping_events.c_str());
      }
    }
  }
  
  return limit_proximity;
}

/***********************************************************************************************************************
 * Applies forward kinematics
***********************************************************************************************************************/
Vector3d Leg::applyFK(bool set_local)
{
  //Update joint transforms - skip first joint since it's transform is constant
  map<int, Joint*>::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    const Link* reference_link = joint->reference_link;
    joint->current_transform = createDHMatrix(reference_link->offset, reference_link->actuating_joint->desired_position, reference_link->length, reference_link->twist);
  }
  const Link* reference_link = tip_->reference_link;
  tip_->current_transform = createDHMatrix(reference_link->offset, reference_link->actuating_joint->desired_position, reference_link->length, reference_link->twist);

  //Get world frame position of tip
  Vector3d tip_position = tip_->getPositionWorldFrame();
  if (set_local)
  {
    if (local_tip_position_[0] != UNASSIGNED_VALUE)
    {
      desired_tip_velocity_ = (tip_position - local_tip_position_) / model_->getTimeDelta();
    }
    local_tip_position_ = tip_position;
  }
  return tip_position;
}

/***********************************************************************************************************************
 * Calls jocobian creation function for requested degrees of freedom
***********************************************************************************************************************/
MatrixXd Leg::createJacobian(vector<map<string, double>> dh)
{
  switch(num_joints_)
  {
    case(1):
      return createJacobian1DOF(dh);
    case(2):
      return createJacobian2DOF(dh);
    case(3):
      return createJacobian3DOF(dh);
    case(4):
      return createJacobian4DOF(dh);
    case(5):
      return createJacobian5DOF(dh); //Not implemented
    case(6):
      return createJacobian6DOF(dh); //Not implemented
    default:
      return MatrixXd::Identity(3,3);
  };
}

/***********************************************************************************************************************
 * Joint data object
***********************************************************************************************************************/
Joint::Joint(Leg* leg, Link* link, int id, Parameters* params)
  : parent_leg(leg)
  , reference_link(link)
  , id_number(id)
  , name(leg->getIDName() + "_" + params->joint_id.data[id_number - 1] + "_joint")
  , position_offset(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["offset"])
  , min_position(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["min"])
  , max_position(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["max"])
  , packed_position(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["packed"])
  , unpacked_position(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["unpacked"])
  , max_angular_speed(params->joint_parameters[leg->getIDNumber()][id_number - 1].data["max_vel"])
{
  if (params->joint_parameters[leg->getIDNumber()][id_number - 1].initialised)
  {
    identity_transform = createDHMatrix(reference_link->offset, reference_link->angle, reference_link->length, reference_link->twist);
    current_transform = identity_transform;
    ROS_DEBUG("%s has been initialised with parameters:"
              "offset: %f, min: %f, max: %f, packed: %f, unpacked: %f, max_vel: %f.",
              name.c_str(), position_offset, min_position, max_position,
              packed_position, unpacked_position, max_angular_speed);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", name.c_str());
    ros::shutdown();
  }
}

/***********************************************************************************************************************
 * Link data object
***********************************************************************************************************************/
Link::Link(Leg* leg, Joint* joint, int id, Parameters* params)
  : parent_leg(leg)
  , actuating_joint(joint)
  , id_number(id)
  , name(leg->getIDName() + "_" + params->link_id.data[id_number] + "_link")
  , length(params->link_parameters[leg->getIDNumber()][id_number].data["r"])
  , angle(params->link_parameters[leg->getIDNumber()][id_number].data["theta"])
  , offset(params->link_parameters[leg->getIDNumber()][id_number].data["d"])
  , twist(params->link_parameters[leg->getIDNumber()][id_number].data["alpha"])
{
  if (params->link_parameters[leg->getIDNumber()][id_number].initialised)
  {
    ROS_DEBUG("%s has been initialised with DH parameters: d: %f, theta: %f, r: %f, alpha: %f.",
              name.c_str(), offset, angle, length, twist);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", name.c_str());
    ros::shutdown();
  }
}

/***********************************************************************************************************************
 * Tip data object
***********************************************************************************************************************/
Tip::Tip(Leg* leg, Link* link)
  : parent_leg(leg)
  , reference_link(link)
  , name(leg->getIDName() + "_tip")
{
  identity_transform = createDHMatrix(reference_link->offset, reference_link->angle, reference_link->length, reference_link->twist);
  current_transform = identity_transform;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
