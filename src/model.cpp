/*******************************************************************************************************************//**
 *  @file    model.cpp
 *  @brief   Describes the Syropod model including all legs, joints and links.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    August 2017
 *  @version 0.5.2
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
 * Contructor for robot model object - initialises member variables from parameters.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Model::Model(const Parameters& params)
  : params_(params)
  , leg_count_(params_.leg_id.data.size())
  , time_delta_(params_.time_delta.data)
  , current_pose_(Pose::identity())
{
}

/*******************************************************************************************************************//**
 * Generates child leg objects. Separated from constructor due to shared_from_this() constraints.
***********************************************************************************************************************/
void Model::generate(void)
{
  for (int i = 0; i < leg_count_; ++i)
  {
    shared_ptr<Leg> leg = make_shared<Leg>(shared_from_this(), i, params_);
    leg->generate();
    leg_container_.insert(LegContainer::value_type(i, leg));
  }
}


/*******************************************************************************************************************//**
 * Iterate through legs in robot model and have them run their initialisation.
 * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position values
 * for any joint with unknown current position values.
***********************************************************************************************************************/
void Model::initLegs(const bool& use_default_joint_positions)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
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
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    body_height_estimate += leg->getCurrentTipPosition()[2];
  }
  return -(body_height_estimate / leg_count_) > HALF_BODY_DEPTH; //TODO Parameterise this value
}

/*******************************************************************************************************************//**
 * Returns pointer to leg requsted via identification name string input.
 * @param[in] leg_id_name The identification name of the requested leg object pointer.
***********************************************************************************************************************/
shared_ptr<Leg> Model::getLegByIDName(const string& leg_id_name)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    if (leg->getIDName() == leg_id_name)
    {
      return leg;
    }
  }
  return NULL;
}

/*******************************************************************************************************************//**
 * Constructor for a robot model leg object. Initialises member variables from parameters.
 * @param[in] model A pointer to the parent robot model.
 * @param[in] id_number An identification number for this leg object.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Leg::Leg(shared_ptr<Model> model, const int& id_number, const Parameters& params)
  : model_(model)
  , params_(params)
  , id_number_(id_number)
  , id_name_(params_.leg_id.data.at(id_number))
  , joint_count_(params_.leg_DOF.data.at(id_name_))
  , leg_state_(WALKING)
  , impedance_state_(vector<double>(2))
{
  desired_tip_position_ = Vector3d(UNASSIGNED_VALUE, UNASSIGNED_VALUE, UNASSIGNED_VALUE);
  desired_tip_velocity_ = Vector3d(0, 0, 0);
  current_tip_position_ = Vector3d(UNASSIGNED_VALUE, UNASSIGNED_VALUE, UNASSIGNED_VALUE);
  current_tip_velocity_ = Vector3d(0, 0, 0);
  tip_force_ = Vector3d(0.0, 0.0, 0.0);
  group_ = (id_number % 2); // Even/odd groups
}

/*******************************************************************************************************************//**
 * Generates child joint/link/tip objects. Separated from constructor due to shared_from_this() constraints.
 * @todo Refactor use of null pointer.
***********************************************************************************************************************/
void Leg::generate(void)
{
  shared_ptr<Joint> null_joint; //TODO
  shared_ptr<Link> base_link = make_shared<Link>(shared_from_this(), null_joint, 0, params_);
  link_container_.insert(LinkContainer::value_type(0, base_link));
  shared_ptr<Link> prev_link = base_link;
  for (int i = 1; i < joint_count_ + 1; ++i)
  {
    shared_ptr<Joint> new_joint = make_shared<Joint>(shared_from_this(), prev_link, i, params_);
    shared_ptr<Link> new_link = make_shared<Link>(shared_from_this(), new_joint, i, params_);
    joint_container_.insert(JointContainer::value_type(i, new_joint));
    link_container_.insert(LinkContainer::value_type(i, new_link));
    prev_link = new_link;
  }
  tip_ = make_shared<Tip>(shared_from_this(), prev_link);
}

/*******************************************************************************************************************//**
 * Initialises leg object by setting desired joint state to default values or to current position (from encoders) and
 * running forward kinematics for tip position.
 * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position values
 * for any joint with unknown current position values.
***********************************************************************************************************************/
void Leg::init(const bool& use_default_joint_positions)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
    if (use_default_joint_positions && joint->current_position_ == UNASSIGNED_VALUE)
    {
      joint->current_position_ = clamped(0.0, joint->min_position_, joint->max_position_);
      joint->current_velocity_ = 0.0;
      joint->current_effort_ = 0.0;
    }
    joint->desired_position_ = joint->current_position_;
    joint->desired_velocity_ = joint->current_velocity_;
    joint->desired_effort_ = joint->current_effort_;
    joint->prev_desired_position_ = joint->desired_position_;
  }
  applyFK();
  desired_tip_position_ = current_tip_position_;
}

/*******************************************************************************************************************//**
  * Re-Initialises leg object by setting desired joint state to values from JointState message input and running
  * forward kinematics for tip position.
  * @param[in] desired_joint_states JointState message containing desired joint states values for leg object.
***********************************************************************************************************************/
void Leg::reInit(const sensor_msgs::JointState& desired_joint_states)
{
  bool get_effort_values = (desired_joint_states.effort.size() != 0);
  bool get_velocity_values = (desired_joint_states.velocity.size() != 0);
  for (uint i = 0; i < desired_joint_states.name.size(); ++i)
  {
    string joint_name(desired_joint_states.name[i]);
    shared_ptr<Joint> joint = getJointByIDName(joint_name);
    if (joint != NULL)
    {
      joint->current_position_ = desired_joint_states.position[i];
      if (get_velocity_values)
      {
        joint->current_velocity_ = desired_joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        joint->current_effort_ = desired_joint_states.effort[i];
      }
      joint->desired_position_ = joint->current_position_;
      joint->desired_velocity_ = joint->current_velocity_;
      joint->desired_effort_ = joint->current_effort_;
      joint->prev_desired_position_ = joint->desired_position_;
    }
  }
  applyFK();
  desired_tip_position_ = current_tip_position_;
}

/*******************************************************************************************************************//**
  * Generates a JointState message from the desired state of the joints of the leg object.
  * @param[out] joint_state_msg The output JointState mesage to fill with the state of joints within this leg object.
***********************************************************************************************************************/
void Leg::generateDesiredJointStateMsg(sensor_msgs::JointState* joint_state_msg)
{
  joint_state_msg->header.stamp = ros::Time::now();
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
    joint_state_msg->name.push_back(joint->id_name_);
    joint_state_msg->position.push_back(joint->desired_position_);
    joint_state_msg->velocity.push_back(joint->desired_velocity_);
    joint_state_msg->effort.push_back(joint->desired_effort_);
  }
}

/*******************************************************************************************************************//**
 * Returns pointer to joint requested via identification name string input.
 * @param[in] joint_id_name The identification name of the requested joint object pointer.
***********************************************************************************************************************/
shared_ptr<Joint> Leg::getJointByIDName(const string& joint_id_name)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
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
shared_ptr<Link> Leg::getLinkByIDName(const string& link_id_name)
{
  LinkContainer::iterator link_it;
  for (link_it = link_container_.begin(); link_it != link_container_.end(); ++link_it)
  {
    shared_ptr<Link> link = link_it->second;
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
void Leg::setDesiredTipPosition(const Vector3d& tip_position, bool apply_delta_z)
{
  // Don't apply delta_z to manually manipulated legs
  apply_delta_z = apply_delta_z && !(leg_state_ == MANUAL || leg_state_ == WALKING_TO_MANUAL);

  desired_tip_position_[0] = tip_position[0];
  desired_tip_position_[1] = tip_position[1];
  desired_tip_position_[2] = tip_position[2] + (apply_delta_z ? delta_z_ : 0.0);
}

/*******************************************************************************************************************//**
  * Calculates an estimate for the tip force vector acting on this leg, using the calculated state jacobian and 
  * values for the torque on each joint in the leg.
  * @todo Implement rotation to tip frame
***********************************************************************************************************************/
void Leg::calculateTipForce(void)
{
  bool ignore_tip_orientation = true; //TODO Implement rotation to tip frame
  shared_ptr<Joint> first_joint = joint_container_.begin()->second;

  Vector3d pe = tip_->getTransformFrom(first_joint).block<3, 1>(0, 3);
  Vector3d z0(0, 0, 1);
  Vector3d p0(0, 0, 0);

  MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); //Linear velocity
  jacobian.block<3, 1>(3, 0) = ignore_tip_orientation ? Vector3d(0, 0, 0) : z0; //Angular velocity

  VectorXd joint_torques(joint_count_);
  joint_torques[0] = first_joint->current_effort_;
  
  //Skip first joint dh parameters since it is a fixed transformation
  int i = 1;
  JointContainer::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    shared_ptr<Joint> joint = joint_it->second;
    Matrix4d t = joint->getTransformFrom(first_joint);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); //Linear velocity
    jacobian.block<3, 1>(3, i) = ignore_tip_orientation ? Vector3d(0, 0, 0) : t.block<3, 1>(0, 2); //Angular velocity
    joint_torques[i] = joint->current_effort_;
  }

  // Transpose and invert jacobian
  MatrixXd identity = Matrix<double, 5, 5>::Identity();
  MatrixXd jacobian_inverse = jacobian * ((jacobian.transpose()*jacobian + sqr(DLS_COEFFICIENT) * identity).inverse());
  
  VectorXd raw_tip_force = jacobian_inverse * joint_torques;
  
  // Low pass filter and force gain applied to calculated raw tip force
  double s = 0.15; // Smoothing Factor
  tip_force_[0] = s * raw_tip_force[0] * params_.force_gain.current_value + (1 - s) * tip_force_[0];
  tip_force_[1] = s * raw_tip_force[1] * params_.force_gain.current_value + (1 - s) * tip_force_[1];
  tip_force_[2] = s * raw_tip_force[2] * params_.force_gain.current_value + (1 - s) * tip_force_[2];
}

/*******************************************************************************************************************//**
 * Applies inverse kinematics to calculate required joint positions to achieve desired tip position. Inverse kinematics
 * is generated via the calculation of a jacobian for the current state of the leg, which is used as per the Damped
 * Least Squares method to generate a change in joint position for each joint. Returns a ratio of the joint closest to
 * position limits.
 * @param[in] simulation_run Flag denoting if this execution is for simulation purposes rather than normal use.
 * @param[in] ignore_tip_orientation Flag denoting if specific orientation of tip is desired or can be ignored
 * @todo Calculate optimal DLS coefficient (this value currently works sufficiently)
 * @todo Remove failsafe for uninitialised clamping flags
***********************************************************************************************************************/
double Leg::applyIK(const bool& simulation_run, const bool& ignore_tip_orientation)
{
  shared_ptr<Joint> first_joint = joint_container_.begin()->second;
  Vector3d pe = tip_->getTransformFrom(first_joint).block<3, 1>(0, 3);
  Vector3d z0(0, 0, 1);
  Vector3d p0(0, 0, 0);

  MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); //Linear velocity
  jacobian.block<3, 1>(3, 0) = ignore_tip_orientation ? Vector3d(0, 0, 0) : z0; //Angular velocity

  //Skip first joint dh parameters since it is a fixed transformation
  JointContainer::iterator joint_it;
  int i = 1;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    shared_ptr<Joint> joint = joint_it->second;
    Matrix4d t = joint->getTransformFrom(first_joint);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); //Linear velocity
    jacobian.block<3, 1>(3, i) = ignore_tip_orientation ? Vector3d(0, 0, 0) : t.block<3, 1>(0, 2); //Angular velocity
  }

  MatrixXd identity = Matrix<double, 6, 6>::Identity();
  MatrixXd ik_matrix(joint_count_, 6);
  MatrixXd j = jacobian;

  //ik_matrix = ((j.transpose()*j).inverse())*j.transpose(); //Pseudo Inverse method
  ik_matrix = j.transpose() * ((j * j.transpose() + sqr(DLS_COEFFICIENT) * identity).inverse()); //DLS Method

  if (desired_tip_velocity_.norm() != 0.0)
  {
    desired_tip_position_ = current_tip_position_ + desired_tip_velocity_ * model_->getTimeDelta();
  }

  shared_ptr<Joint> base_joint = joint_container_.begin()->second;
  Vector3d leg_frame_desired_tip_position = base_joint->getPositionJointFrame(false, desired_tip_position_);
  Vector3d leg_frame_prev_desired_tip_position = base_joint->getPositionJointFrame(false, current_tip_position_);
  Vector3d leg_frame_tip_position_delta = leg_frame_desired_tip_position - leg_frame_prev_desired_tip_position;
  MatrixXd delta = Matrix<double, 6, 1>::Zero();
  delta(0) = leg_frame_tip_position_delta(0);
  delta(1) = leg_frame_tip_position_delta(1);
  delta(2) = leg_frame_tip_position_delta(2);

  VectorXd joint_delta_pos(joint_count_);
  joint_delta_pos = ik_matrix * delta;

  int index = 0;
  string clamping_events;
  double min_limit_proximity = 1.0;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    shared_ptr<Joint> joint = joint_it->second;
    joint->desired_velocity_ = joint_delta_pos[index] / model_->getTimeDelta();

    // Clamp joint velocities within limits
    if (params_.clamp_joint_velocities.data && !simulation_run)
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
    joint->prev_desired_position_ = joint->desired_position_;

    // Clamp joint position within limits
    if (!params_.clamp_joint_positions.initialised || params_.clamp_joint_positions.data) //TODO - remove failsafe
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
    double limit_proximity = half_joint_range != 0 ? min(min_diff, max_diff) / half_joint_range : 1.0;
    min_limit_proximity = min(limit_proximity, min_limit_proximity);
    ROS_DEBUG_COND(params_.debug_IK.data && limit_proximity == 0, "\n%s at limit.\n", joint->id_name_.c_str());
  }

  Vector3d result = applyFK(); // Apply forward kinematics to get new current tip position

  // Debugging message
  ROS_DEBUG_COND(id_number_ == 0 && params_.debug_IK.data,
                 "\nLeg %s:\n\tDesired tip position from trajectory engine: %f:%f:%f\n\t"
                 "Resultant tip position from inverse/forward kinematics: %f:%f:%f", id_name_.c_str(),
                 desired_tip_position_[0], desired_tip_position_[1], desired_tip_position_[2],
                 result[0], result[1], result[2]);

  // Display warning messages for clamping events and associated inverse kinematic deviations
  if (params_.debug_workspace_calc.data && !simulation_run)
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
  return min_limit_proximity;
}

/*******************************************************************************************************************//**
 * Updates joint transforms and applies forward kinematics to calculate a new tip position. Sets leg current tip
 * position to new position if requested.
 * @param[in] set_current Flag denoting of the calculated tip position should be set as the current tip position.
***********************************************************************************************************************/
Vector3d Leg::applyFK(const bool& set_current)
{
  //Update joint transforms - skip first joint since it's transform is constant
  JointContainer::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
    const shared_ptr<Link> reference_link = joint->reference_link_;
    double joint_angle = reference_link->actuating_joint_->desired_position_;
    joint->current_transform_ = createDHMatrix(reference_link->dh_parameter_d_,
                                               reference_link->dh_parameter_theta_ + joint_angle,
                                               reference_link->dh_parameter_r_,
                                               reference_link->dh_parameter_alpha_);
  }
  const shared_ptr<Link> reference_link = tip_->reference_link_;
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
 * Constructor for Link object. Initialises member variables from parameters.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] actuating_joint A pointer to the actuating joint object, from which this link is moved.
 * @param[in] id_number The identification number for this link.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Link::Link(shared_ptr<Leg> leg, shared_ptr<Joint> actuating_joint, const int& id_number, const Parameters& params)
  : parent_leg_(leg)
  , actuating_joint_(actuating_joint)
  , id_number_(id_number)
  , id_name_(leg->getIDName() + "_" + params.link_id.data[id_number_] + "_link")
  , dh_parameter_r_(params.link_parameters[leg->getIDNumber()][id_number_].data.at("r"))
  , dh_parameter_theta_(params.link_parameters[leg->getIDNumber()][id_number_].data.at("theta"))
  , dh_parameter_d_(params.link_parameters[leg->getIDNumber()][id_number_].data.at("d"))
  , dh_parameter_alpha_(params.link_parameters[leg->getIDNumber()][id_number_].data.at("alpha"))
{
  if (params.link_parameters[leg->getIDNumber()][id_number_].initialised)
  {
    ROS_DEBUG("\n%s has been initialised with DH parameters: d: %f, theta: %f, r: %f, alpha: %f.\n",
              id_name_.c_str(), dh_parameter_d_, dh_parameter_theta_, dh_parameter_r_, dh_parameter_alpha_);
  }
  else
  {
    ROS_FATAL("\nModel initialisation error for %s\n", id_name_.c_str());
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
Joint::Joint(shared_ptr<Leg> leg, shared_ptr<Link> reference_link, const int& id_number, const Parameters& params)
  : parent_leg_(leg)
  , reference_link_(reference_link)
  , id_number_(id_number)
  , id_name_(leg->getIDName() + "_" + params.joint_id.data[id_number_ - 1] + "_joint")
  , min_position_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("min"))
  , max_position_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("max"))
  , packed_position_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("packed"))
  , unpacked_position_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("unpacked"))
  , max_angular_speed_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("max_vel"))
{
  if (params.joint_parameters[leg->getIDNumber()][id_number_ - 1].initialised)
  {
    identity_transform_ = createDHMatrix(reference_link_->dh_parameter_d_,
                                         reference_link_->dh_parameter_theta_,
                                         reference_link_->dh_parameter_r_,
                                         reference_link_->dh_parameter_alpha_);
    current_transform_ = identity_transform_;
    ROS_DEBUG("\n%s has been initialised with parameters:"
              "min: %f, max: %f, packed: %f, unpacked: %f, max_vel: %f.\n",
              id_name_.c_str(), min_position_, max_position_,
              packed_position_, unpacked_position_, max_angular_speed_);
  }
  else
  {
    ROS_FATAL("\nModel initialisation error for %s\n", id_name_.c_str());
    ros::shutdown();
  }
}

/*******************************************************************************************************************//**
 * Constructor for Tip object. Initialises member variables from parameters and generates initial transform.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] reference_link A pointer to the reference link object, which is attached to this tip object.
***********************************************************************************************************************/
Tip::Tip(shared_ptr<Leg> leg, shared_ptr<Link> reference_link)
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
