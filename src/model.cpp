/*******************************************************************************************************************//**
 *  @file    model.cpp
 *  @brief   Describes the Syropod model including all legs, joints and links.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    June 2017
 *  @version 0.5.0
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

#include "../include/syropod_highlevel_controller/model.h"

/*******************************************************************************************************************//**
 * Contructor for robot model object - initialises member variables from parameters and creates leg objects.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Model::Model(Parameters* params)
  : leg_count_(params->leg_id.data.size())
  , time_delta_(params->time_delta.data)
  , current_pose_(Pose::identity())
{
  for (int i = 0; i < leg_count_; ++i)
  {
    Leg* leg = new Leg(this, i, params);
    leg_container_.insert(map<int, Leg*>::value_type(i, leg));
  }
}

/*******************************************************************************************************************//**
 * Iterate through legs in robot model and have them run their initialisation.
 * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position values
 * for any joint with unknown current position values.
***********************************************************************************************************************/
void Model::initLegs(bool use_default_joint_positions)
{
  map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    leg->init(use_default_joint_positions);
  }
}

/*******************************************************************************************************************//**
 * Estimates if the robot is bearing its load on its legs. Estimates the average distance between body and leg tips
 * and checks if the average breaks the plane of the robot body underside, if so, assume that at least one leg is 
 * bearing load.
 * @todo Make more robust by estimating body height above the ground.
 * @todo Parameterise HALF_BODY_DEPTH
***********************************************************************************************************************/
bool Model::legsBearingLoad(void)
{
  double body_height_estimate = 0.0;
  map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    body_height_estimate += leg->getCurrentTipPosition()[2];
  }
  return -(body_height_estimate / leg_count_) > HALF_BODY_DEPTH; //TODO Parameterise this value
}

/*******************************************************************************************************************//**
 * Returns pointer to leg requsted via identification name string input.
 * @param[in] leg_name The identification name of the requested leg object pointer.
***********************************************************************************************************************/
Leg* Model::getLegByIDName(string leg_id_name)
{
  map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    if (leg->getIDName() == leg_id_name)
    {
      return leg;
    }
  }
  return NULL;
}

/*******************************************************************************************************************//**
 * Constructor for a robot model leg object. Initialises member variables from parameters and creates child 
 * joint/link/tip objects.
 * @param[in] model A pointer to the parent robot model.
 * @param[in] id_number An identification number for this leg object.
 * @param[in] params A pointer to the parameter data structure.
 * @todo Refactor use of null pointer.
***********************************************************************************************************************/
Leg::Leg(Model* model, int id_number, Parameters* params)
  : model_(model)
  , id_number_(id_number)
  , id_name_(params->leg_id.data[id_number])
  , joint_count_(params->leg_DOF.data[id_name_])
  , stance_leg_yaw_(params->leg_stance_yaws.data[id_name_])
  , leg_state_(WALKING)
  , impedance_state_(vector<double>(2))
{
  Joint* null_joint; //TODO
  Link* base_link = new Link(this, null_joint, 0, params);
  link_container_.insert(map<int, Link*>::value_type(0, base_link));
  Link* prev_link = base_link;
  for (int i = 1; i < joint_count_ + 1; ++i)
  {
    Joint* new_joint = new Joint(this, prev_link, i, params);
    Link* new_link = new Link(this, new_joint, i, params);
    joint_container_.insert(map<int, Joint*>::value_type(i, new_joint));
    link_container_.insert(map<int, Link*>::value_type(i, new_link));
    prev_link = new_link;
  }
  tip_ = new Tip(this, prev_link);

  desired_tip_position_ = Vector3d(UNASSIGNED_VALUE, UNASSIGNED_VALUE, UNASSIGNED_VALUE);
  current_tip_position_ = Vector3d(UNASSIGNED_VALUE, UNASSIGNED_VALUE, UNASSIGNED_VALUE);
  current_tip_velocity_ = Vector3d(0, 0, 0);
  group_ = (id_number % 2); // Even/odd groups
}

/*******************************************************************************************************************//**
 * Initialises leg object by setting desired joint state to default values or to current position (from encoders) and
 * running forward kinematics for tip position.
 * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position values
 * for any joint with unknown current position values.
***********************************************************************************************************************/
void Leg::init(bool use_default_joint_positions)
{
  map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (use_default_joint_positions && joint->current_position_ == UNASSIGNED_VALUE)
    {
      joint->current_position_ = clamped(0.0, joint->min_position_, joint->max_position_);
      joint->current_velocity_ = 0.0;
      joint->current_effort_ = 0.0;
    }
    joint->desired_position_ = joint->current_position_;
    joint->prev_desired_position_ = joint->desired_position_;
  }
  applyFK();
  desired_tip_position_ = current_tip_position_;
}

/*******************************************************************************************************************//**
 * Returns pointer to joint requested via identification name string input.
 * @param[in] joint_id_name The identification name of the requested joint object pointer.
***********************************************************************************************************************/
Joint* Leg::getJointByIDName(string joint_id_name)
{
  map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (joint->id_name_ == joint_id_name)
    {
      return joint;
    }
  }
  return NULL;
}

/*******************************************************************************************************************//**
 * Returns pointer to link requested via identification name string input.
 * @param[in] link_id_name The identification name of the requested link object pointer.
***********************************************************************************************************************/
Link* Leg::getLinkByIDName(string link_id_name)
{
  map<int, Link*>::iterator link_it;
  for (link_it = link_container_.begin(); link_it != link_container_.end(); ++link_it)
  {
    Link* link = link_it->second;
    if (link->id_name_ == link_id_name)
    {
      return link;
    }
  }
  return NULL;
}

/*******************************************************************************************************************//**
 * Sets desired tip position to the input, applying impedance controller vertical offset (delta z) if requested.
 * @param[in] tip_position The input desired tip position 
 * @param[in] apply_delta_z Flag denoting if 'delta_z' should be applied to desired tip position.
***********************************************************************************************************************/
void Leg::setDesiredTipPosition(Vector3d tip_position, bool apply_delta_z)
{
  // Don't apply delta_z to manually manipulated legs
  apply_delta_z = apply_delta_z && !(leg_state_ == MANUAL || leg_state_ == WALKING_TO_MANUAL);

  desired_tip_position_[0] = tip_position[0];
  desired_tip_position_[1] = tip_position[1];
  desired_tip_position_[2] = tip_position[2] + (apply_delta_z ? delta_z_ : 0.0);
}

/*******************************************************************************************************************//**
 * Applies inverse kinematics to calculate required joint positions to achieve desired tip position. Inverse kinematics
 * is generated via the calculation of a jacobian for the current state of the leg, which is used as per the Damped 
 * Least Squares method to generate a change in joint position for each joint. Returns a ratio of the joint closest to
 * position limits.
 * @param[in] debug Flag denoting if debug information should be sent to rosconsole.
 * @param[in] ignore_warnings Flag denoting if all warning messages should be prevented from being sent to rosconsole.
 * @param[in] clamp_positions Flag denoting if joint positions should adhere to limits.
 * @param[in] clamp_velocities Flag denoting if joint velocities should adhere to limits.
 * @todo Calculate optimal DLS coefficient (this value currently works sufficiently)
 * @todo Parameterise clamp positions flag
 * @todo Parameterise clamp velocities flag
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
    const Link* reference_link = joint->reference_link_;
    double joint_angle = reference_link->actuating_joint_->desired_position_;
    dh_map.insert(map<string, double>::value_type("d", reference_link->dh_parameter_d_));
    dh_map.insert(map<string, double>::value_type("theta", reference_link->dh_parameter_theta_ + joint_angle));
    dh_map.insert(map<string, double>::value_type("r", reference_link->dh_parameter_r_));
    dh_map.insert(map<string, double>::value_type("alpha", reference_link->dh_parameter_alpha_));
    dh_parameters.push_back(dh_map);
    dh_map.clear();
  }
  //Add tip dh params
  double joint_angle = tip_->reference_link_->actuating_joint_->desired_position_;
  dh_map.insert(map<string, double>::value_type("d", tip_->reference_link_->dh_parameter_d_));
  dh_map.insert(map<string, double>::value_type("theta", tip_->reference_link_->dh_parameter_theta_ + joint_angle));
  dh_map.insert(map<string, double>::value_type("r", tip_->reference_link_->dh_parameter_r_));
  dh_map.insert(map<string, double>::value_type("alpha", tip_->reference_link_->dh_parameter_alpha_));
  dh_parameters.push_back(dh_map);

  MatrixXd j(3, joint_count_);
  j = createJacobian(dh_parameters);

  MatrixXd identity = Matrix3d::Identity();
  MatrixXd ik_matrix(joint_count_, 3);
  //ik_matrix = ((j.transpose()*j).inverse())*j.transpose(); //Pseudo Inverse method
  ik_matrix = j.transpose() * ((j * j.transpose() + sqr(DLS_COEFFICIENT) * identity).inverse()); //DLS Method

  Joint* base_joint = joint_container_.begin()->second;
  Vector3d leg_frame_desired_tip_position = base_joint->getPositionJointFrame(false, desired_tip_position_);
  Vector3d leg_frame_prev_desired_tip_position = base_joint->getPositionJointFrame(false, current_tip_position_);
  Vector3d leg_frame_tip_position_delta = leg_frame_desired_tip_position - leg_frame_prev_desired_tip_position;
  VectorXd joint_delta_pos(joint_count_);

  joint_delta_pos = ik_matrix * leg_frame_tip_position_delta;

  int index = 0;
  string clamping_events;
  double limit_proximity = 1.0;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    Joint* joint = joint_it->second;
    joint->desired_velocity_ = joint_delta_pos[index] / model_->getTimeDelta();

    // Clamp joint velocities within limits
    if (clamp_velocities)  // TODO
    {
      if (abs(joint->desired_velocity_) > joint->max_angular_speed_)
      {
        double max_velocity = joint->max_angular_speed_;
        clamping_events += stringFormat("\n\tType: Velocity\tJoint: %s\tDesired: %f rad/s\tLimited to: %f rad/s",
                                        joint->id_name_.c_str(), abs(joint->desired_velocity_), max_velocity);
        joint->desired_velocity_ = clamped(joint->desired_velocity_, -max_velocity, max_velocity);
      }
    }
    joint->desired_position_ = joint->prev_desired_position_ + joint->desired_velocity_ * model_->getTimeDelta();

    // Clamp joint position within limits
    if (clamp_positions) // TODO
    {
      if (joint->desired_position_ < joint->min_position_)
      {
        clamping_events += stringFormat("\n\tType: Position\tJoint: %s\tDesired: %f rad\tLimited to: %f rad",
                                        joint->id_name_.c_str(), joint->desired_position_, joint->min_position_);
        joint->desired_position_ = joint->min_position_;
      }
      else if (joint->desired_position_ > joint->max_position_)
      {
        clamping_events += stringFormat("\n\tType: Position\tJoint: %s\tDesired: %f rad\tLimited to: %f rad",
                                        joint->id_name_.c_str(), joint->desired_position_, joint->max_position_);
        joint->desired_position_ = joint->max_position_;
      }
    }

    //Calculates the proximity of the joint closest to one of it's limits. Used for preventing exceeding workspace.
    // (1.0 = furthest possible from limit, 0.0 = equal to limit)
    double min_diff = abs(joint->min_position_ - joint->desired_position_);
    double max_diff = abs(joint->max_position_ - joint->desired_position_);
    double half_joint_range = (joint->max_position_ - joint->min_position_) / 2.0;
    limit_proximity = min(limit_proximity, min(min_diff, max_diff) / half_joint_range);
  }

  Vector3d result = applyFK(); // Apply forward kinematics to get new current tip position

  // Debugging message
  ROS_DEBUG_COND(id_number_ == 0 && debug,
                 "\nLeg %s:\n\tDesired tip position from trajectory engine: %f:%f:%f\n\t"
                 "Resultant tip position from inverse/forward kinematics: %f:%f:%f", id_name_.c_str(),
                 desired_tip_position_[0], desired_tip_position_[1], desired_tip_position_[2],
                 result[0], result[1], result[2]);

  // Display warning messages for clamping events and associated inverse kinematic deviations
  if (!ignore_warnings)
  {
    string axis_label[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
      if (abs(result[i] - desired_tip_position_[i]) > IK_TOLERANCE)
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

/*******************************************************************************************************************//**
 * Updates joint transforms and applies forward kinematics to calculate a new tip position. Sets leg current tip 
 * position to new position if requested.
 * @param[in] set_current Flag denoting of the calculated tip position should be set as the current tip position.
***********************************************************************************************************************/
Vector3d Leg::applyFK(bool set_current)
{
  //Update joint transforms - skip first joint since it's transform is constant
  map<int, Joint*>::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    const Link* reference_link = joint->reference_link_;
    double joint_angle = reference_link->actuating_joint_->desired_position_;
    joint->current_transform_ = createDHMatrix(reference_link->dh_parameter_d_,
                                               reference_link->dh_parameter_theta_ + joint_angle,
                                               reference_link->dh_parameter_r_,
                                               reference_link->dh_parameter_alpha_);
  }
  const Link* reference_link = tip_->reference_link_;
  double joint_angle = reference_link->actuating_joint_->desired_position_;
  tip_->current_transform_ = createDHMatrix(reference_link->dh_parameter_d_,
                                            reference_link->dh_parameter_theta_ + joint_angle,
                                            reference_link->dh_parameter_r_,
                                            reference_link->dh_parameter_alpha_);

  //Get world frame position of tip
  Vector3d tip_position = tip_->getPositionRobotFrame();
  if (set_current)
  {
    if (current_tip_position_[0] != UNASSIGNED_VALUE)
    {
      current_tip_velocity_ = (tip_position - current_tip_position_) / model_->getTimeDelta();
    }
    current_tip_position_ = tip_position;
  }
  return tip_position;
}

/*******************************************************************************************************************//**
 * Calls jocobian creation function for requested degrees of freedom.
 * @param[in] dh A vector containing a map of DH parameter strings and values for each degree of freedom.
***********************************************************************************************************************/
MatrixXd Leg::createJacobian(vector<map<string, double>> dh)
{
  switch (joint_count_)
  {
    case (1):
      return createJacobian1DOF(dh);
    case (2):
      return createJacobian2DOF(dh);
    case (3):
      return createJacobian3DOF(dh);
    case (4):
      return createJacobian4DOF(dh);
    case (5):
      return createJacobian5DOF(dh);
    case (6):
      return createJacobian6DOF(dh); //Not implemented
    default:
      return MatrixXd::Identity(3, 3);
  };
}

/*******************************************************************************************************************//**
 * Constructor for Link object. Initialises member variables from parameters.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] actuating_joint A pointer to the actuating joint object, from which this link is moved.
 * @param[in] id_number The identification number for this link.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Link::Link(Leg* leg, Joint* actuating_joint, int id_number, Parameters* params)
  : parent_leg_(leg)
  , actuating_joint_(actuating_joint)
  , id_number_(id_number)
  , id_name_(leg->getIDName() + "_" + params->link_id.data[id_number_] + "_link")
  , dh_parameter_r_(params->link_parameters[leg->getIDNumber()][id_number_].data["r"])
  , dh_parameter_theta_(params->link_parameters[leg->getIDNumber()][id_number_].data["theta"])
  , dh_parameter_d_(params->link_parameters[leg->getIDNumber()][id_number_].data["d"])
  , dh_parameter_alpha_(params->link_parameters[leg->getIDNumber()][id_number_].data["alpha"])
{
  if (params->link_parameters[leg->getIDNumber()][id_number_].initialised)
  {
    ROS_DEBUG("%s has been initialised with DH parameters: d: %f, theta: %f, r: %f, alpha: %f.",
              id_name_.c_str(), dh_parameter_d_, dh_parameter_theta_, dh_parameter_r_, dh_parameter_alpha_);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", id_name_.c_str());
    ros::shutdown();
  }
}

/*******************************************************************************************************************//**
 * Constructor for Joint object. Initialises member variables from parameters and generates initial transform.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] reference_link A pointer to the reference link object, from which this joint actuates.
 * @param[in] id_number The identification number for this joint
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Joint::Joint(Leg* leg, Link* reference_link, int id_number, Parameters* params)
  : parent_leg_(leg)
  , reference_link_(reference_link)
  , id_number_(id_number)
  , id_name_(leg->getIDName() + "_" + params->joint_id.data[id_number_ - 1] + "_joint")
  , position_offset_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["offset"])
  , min_position_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["min"])
  , max_position_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["max"])
  , packed_position_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["packed"])
  , unpacked_position_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["unpacked"])
  , max_angular_speed_(params->joint_parameters[leg->getIDNumber()][id_number_ - 1].data["max_vel"])
{
  if (params->joint_parameters[leg->getIDNumber()][id_number_ - 1].initialised)
  {
    identity_transform_ = createDHMatrix(reference_link_->dh_parameter_d_,
                                         reference_link_->dh_parameter_theta_,
                                         reference_link_->dh_parameter_r_,
                                         reference_link_->dh_parameter_alpha_);
    current_transform_ = identity_transform_;
    ROS_DEBUG("%s has been initialised with parameters:"
              "offset: %f, min: %f, max: %f, packed: %f, unpacked: %f, max_vel: %f.",
              id_name_.c_str(), position_offset_, min_position_, max_position_,
              packed_position_, unpacked_position_, max_angular_speed_);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", id_name_.c_str());
    ros::shutdown();
  }
}

/*******************************************************************************************************************//**
 * Constructor for Tip object. Initialises member variables from parameters and generates initial transform.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] reference_link A pointer to the reference link object, which is attached to this tip object.
***********************************************************************************************************************/
Tip::Tip(Leg* leg, Link* reference_link)
  : parent_leg_(leg)
  , reference_link_(reference_link)
  , id_name_(leg->getIDName() + "_tip")
{
  identity_transform_ = createDHMatrix(reference_link_->dh_parameter_d_,
                                       reference_link_->dh_parameter_theta_,
                                       reference_link_->dh_parameter_r_,
                                       reference_link_->dh_parameter_alpha_);
  current_transform_ = identity_transform_;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
