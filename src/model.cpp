/*******************************************************************************************************************//**
 *  @file    model.cpp
 *  @brief   Describes the Syropod model including all legs, joints and links.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    January 2018
 *  @version 0.5.9
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
#include "../include/syropod_highlevel_controller/walk_controller.h"
#include "../include/syropod_highlevel_controller/pose_controller.h"

/*******************************************************************************************************************//**
 * Contructor for robot model object - initialises member variables from parameters.
 * @param[in] params A pointer to the parameter data structure.
***********************************************************************************************************************/
Model::Model(const Parameters& params)
  : params_(params)
  , leg_count_(params_.leg_id.data.size())
  , time_delta_(params_.time_delta.data)
  , current_pose_(Pose::Identity())
  , default_pose_(Pose::Identity())
{
  imu_data_.orientation = UNDEFINED_ROTATION;
  imu_data_.linear_acceleration = Vector3d::Zero();
  imu_data_.angular_velocity = Vector3d::Zero();
}

/*******************************************************************************************************************//**
 * Copy Constructor for a robot model object. Initialises member variables from existing Model object.
 * @param[in] model A pointer to a existing reference robot model object
***********************************************************************************************************************/
Model::Model(shared_ptr<Model> model)
  : params_(model->params_)
  , leg_count_(model->leg_count_)
  , time_delta_(model->time_delta_)
  , current_pose_(model->current_pose_)
  , default_pose_(model->default_pose_)
  , imu_data_(model->imu_data_)
{
}

/*******************************************************************************************************************//**
 * Generates child leg objects and copies state from reference model if provided.
 * Separated from constructor due to shared_from_this() constraints.
 * @param[in] model A pointer to a existing reference robot model object
***********************************************************************************************************************/
void Model::generate(shared_ptr<Model> model)
{
  for (int i = 0; i < leg_count_; ++i)
  {
    shared_ptr<Leg> leg;
    if (model != NULL)
    {
      shared_ptr<Leg> reference_leg = model->leg_container_.find(i)->second;
      leg = allocate_shared<Leg>(aligned_allocator<Leg>(), reference_leg);
      leg->generate(reference_leg);
    }
    else
    {
      leg = allocate_shared<Leg>(aligned_allocator<Leg>(), shared_from_this(), i, params_);
      leg->generate();
    }
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
    body_height_estimate += leg->getCurrentTipPose().position_[2];
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
 * Updates joint default positions for each leg according to current joint positions of each leg.
***********************************************************************************************************************/
void Model::updateDefaultConfiguration(void)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    leg->updateDefaultConfiguration();
  }
}

/*******************************************************************************************************************//**
 * Updates model configuration by applying inverse kinematics to solve desired tip poses generated from walk/pose
 * controllers.
***********************************************************************************************************************/
void Model::updateModel(void)
{
  // Model uses posed tip positions, adds deltaZ from admittance controller and applies inverse kinematics on each leg
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    if (params_.use_joint_effort.data)
    {
      leg->calculateTipForce();
    }
    leg->setDesiredTipPose();
    leg->applyIK();
  }
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
  , admittance_delta_(Vector3d::Zero())
  , admittance_state_(vector<double>(2))
{
  desired_tip_pose_ = Pose::Undefined();
  desired_tip_velocity_ = Vector3d::Zero();
  current_tip_pose_ = Pose::Undefined();
  current_tip_velocity_ = Vector3d::Zero();
  tip_force_ = Vector3d::Zero();
  tip_torque_ = Vector3d::Zero();
  step_plane_pose_ = Pose::Undefined();
  group_ = (id_number % 2); // Even/odd groups
}

/*******************************************************************************************************************//**
 * Copy Constructor for a robot model leg object. Initialises member variables from existing Leg object.
 * @param[in] leg A pointer to the existing robot model leg object.
***********************************************************************************************************************/
Leg::Leg(shared_ptr<Leg> leg)
  : model_(leg->model_)
  , params_(leg->params_)
  , id_number_(leg->id_number_)
  , id_name_(leg->id_name_)
  , joint_count_(leg->joint_count_)
  , leg_state_(leg->leg_state_)
  , admittance_state_(leg->admittance_state_)
{
  leg_state_publisher_ = leg->leg_state_publisher_;
  asc_leg_state_publisher_ = leg->asc_leg_state_publisher_;
  admittance_delta_ = leg->admittance_delta_;
  virtual_mass_ = leg->virtual_mass_;
  virtual_stiffness_ = leg->virtual_stiffness_;
  virtual_damping_ratio_ = leg->virtual_damping_ratio_;
  desired_tip_pose_ = leg->desired_tip_pose_;
  current_tip_pose_ = leg->current_tip_pose_;  
  desired_tip_velocity_ = leg->desired_tip_velocity_;
  current_tip_velocity_ = leg->current_tip_velocity_;
  group_ = leg->group_;
  tip_force_ = leg->tip_force_;
  tip_torque_ = leg->tip_torque_;
  step_plane_pose_ = leg->step_plane_pose_;
}

/*******************************************************************************************************************//**
 * Generates child joint/link/tip objects and copies state from reference leg if provided.
 * Separated from constructor due to shared_from_this() constraints.
 * @param[in] leg A pointer to a existing reference robot model leg object
***********************************************************************************************************************/
void Leg::generate(shared_ptr<Leg> leg)
{
  shared_ptr<Joint> null_joint = allocate_shared<Joint>(aligned_allocator<Joint>()); //Null joint acts as origin
  shared_ptr<Link> base_link = 
    allocate_shared<Link>(aligned_allocator<Link>(), shared_from_this(), null_joint, 0, params_);
  link_container_.insert(LinkContainer::value_type(0, base_link));
  shared_ptr<Link> prev_link = base_link;
  for (int i = 1; i < joint_count_ + 1; ++i)
  {
    shared_ptr<Joint> new_joint = 
      allocate_shared<Joint>(aligned_allocator<Joint>(), shared_from_this(), prev_link, i, params_);
    shared_ptr<Link> new_link = 
      allocate_shared<Link>(aligned_allocator<Link>(), shared_from_this(), new_joint, i, params_);
    joint_container_.insert(JointContainer::value_type(i, new_joint));
    link_container_.insert(LinkContainer::value_type(i, new_link));
    prev_link = new_link;
  }
  tip_ = allocate_shared<Tip>(aligned_allocator<Tip>(), shared_from_this(), prev_link);

  // If given reference leg, copy member element variables to this leg object
  if (leg != NULL)
  {
    // Copy Joint variables
    JointContainer::iterator joint_it;
    for (joint_it = leg->joint_container_.begin(); joint_it != leg->joint_container_.end(); ++joint_it)
    {
      shared_ptr<Joint> old_joint = joint_it->second;
      shared_ptr<Joint> new_joint = joint_container_.find(old_joint->id_number_)->second;
      new_joint->current_transform_ = old_joint->current_transform_;
      new_joint->identity_transform_ = old_joint->identity_transform_;
      new_joint->desired_position_publisher_ = old_joint->desired_position_publisher_;
      new_joint->desired_position_ = old_joint->desired_position_;
      new_joint->desired_velocity_ = old_joint->desired_velocity_;
      new_joint->desired_effort_ = old_joint->desired_effort_;
      new_joint->prev_desired_position_ = old_joint->prev_desired_position_;
      new_joint->prev_desired_velocity_ = old_joint->prev_desired_velocity_;
      new_joint->prev_desired_effort_ = old_joint->prev_desired_effort_;
      new_joint->current_position_ = old_joint->current_position_;
      new_joint->current_velocity_ = old_joint->current_velocity_;
      new_joint->current_effort_ = old_joint->current_effort_;
      new_joint->default_position_ = old_joint->default_position_;
      new_joint->default_velocity_ = old_joint->default_velocity_;
      new_joint->default_effort_ = old_joint->default_effort_;
    }

    // Copy Link variables 
    // NOTE: Link objects have zero non-constant variables

    // Copy tip variables
    tip_->identity_transform_ = leg->tip_->identity_transform_;
    tip_->current_transform_ = leg->tip_->current_transform_;
    
    // Copy LegStepper
    leg_stepper_ = allocate_shared<LegStepper>(aligned_allocator<LegStepper>(), leg->getLegStepper());
    leg_stepper_->setParentLeg(shared_from_this());
    
    // Copy LegPoser
    leg_poser_ = allocate_shared<LegPoser>(aligned_allocator<LegPoser>(), leg->getLegPoser());
    leg_poser_->setParentLeg(shared_from_this());
  }
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
    if (use_default_joint_positions)
    {
      joint->current_position_ = joint->default_position_;
      joint->current_velocity_ = joint->default_velocity_;
      joint->current_effort_ = joint->default_effort_;
    }
    joint->desired_position_ = joint->current_position_;
    joint->desired_velocity_ = joint->current_velocity_;
    joint->desired_effort_ = joint->current_effort_;
    joint->prev_desired_position_ = joint->desired_position_;
  }
  applyFK();
  desired_tip_pose_ = current_tip_pose_;
}

/*******************************************************************************************************************//**
 * Updates joint default positions according to current joint positions.
***********************************************************************************************************************/
void Leg::updateDefaultConfiguration(void)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
    joint->default_position_ = joint->desired_position_;
  }
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
 * Sets desired tip pose to the input, applying admittance controller vertical offset (delta z) if requested.
 * @param[in] tip_pose The input desired tip pose
 * @param[in] apply_delta Flag denoting if admittance control position offset should be applied to desired tip position.
***********************************************************************************************************************/
void Leg::setDesiredTipPose(const Pose& tip_pose, bool apply_delta)
{
  // Don't apply delta to manually manipulated legs
  apply_delta = apply_delta && !(leg_state_ == MANUAL || leg_state_ == WALKING_TO_MANUAL);

  bool use_poser_tip_pose = (Pose::Undefined() == tip_pose);

  desired_tip_pose_ = use_poser_tip_pose ? leg_poser_->getCurrentTipPose() : tip_pose;
  desired_tip_pose_.position_ += (apply_delta ? admittance_delta_ : Vector3d::Zero());
  ROS_ASSERT(desired_tip_pose_.isValid());
}

/*******************************************************************************************************************//**
  * Calculates an estimate for the tip force vector acting on this leg, using the calculated state jacobian and 
  * values for the torque on each joint in the leg.
  * @todo Implement rotation to tip frame
***********************************************************************************************************************/
void Leg::calculateTipForce(void)
{
  shared_ptr<Joint> first_joint = joint_container_.begin()->second;

  Vector3d pe = tip_->getTransformFromJoint(first_joint->id_number_).block<3, 1>(0, 3);
  Vector3d z0(0, 0, 1);
  Vector3d p0(0, 0, 0);

  MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); //Linear velocity
  jacobian.block<3, 1>(3, 0) = z0; //Angular velocity

  VectorXd joint_torques(joint_count_);
  joint_torques[0] = first_joint->current_effort_;
  
  //Skip first joint dh parameters since it is a fixed transformation
  int i = 1;
  JointContainer::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    shared_ptr<Joint> joint = joint_it->second;
    Matrix4d t = joint->getTransformFromJoint(first_joint->id_number_);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); //Linear velocity
    jacobian.block<3, 1>(3, i) = t.block<3, 1>(0, 2); //Angular velocity
    joint_torques[i] = joint->current_effort_;
  }

  // Transpose and invert jacobian
  MatrixXd identity = MatrixXd::Identity(joint_count_, joint_count_);
  MatrixXd transformation = jacobian * ((jacobian.transpose()*jacobian + sqr(DLS_COEFFICIENT) * identity).inverse());
  
  VectorXd raw_tip_force_leg_frame = transformation * joint_torques;
  Quaterniond rotation = (first_joint->getPoseJointFrame()).rotation_;
  VectorXd raw_tip_force = rotation._transformVector(raw_tip_force_leg_frame.block<3, 1>(0, 0));
  
  // Low pass filter and force gain applied to calculated raw tip force
  double s = 0.15; // Smoothing Factor
  tip_force_[0] = s * raw_tip_force[0] * params_.force_gain.current_value + (1 - s) * tip_force_[0];
  tip_force_[1] = s * raw_tip_force[1] * params_.force_gain.current_value + (1 - s) * tip_force_[1];
  tip_force_[2] = s * raw_tip_force[2] * params_.force_gain.current_value + (1 - s) * tip_force_[2];
  
  // Use newly calculated tip force for touchdown detection
  leg_stepper_->setTouchdownDetection(true);
  touchdownDetection();
}

/*******************************************************************************************************************//**
  * Checks tip force magnitude against touchdown/liftoff thresholds to instantaneously define the location of the 
  * step plane.
***********************************************************************************************************************/
void Leg::touchdownDetection(void)
{
  if (tip_force_.norm() > params_.touchdown_threshold.data && step_plane_pose_ == Pose::Undefined())
  {
    step_plane_pose_ = current_tip_pose_;
  }
  else if (tip_force_.norm() < params_.liftoff_threshold.data)
  {
    step_plane_pose_ = Pose::Undefined();
  }
}

/*******************************************************************************************************************//**
 * Applies inverse kinematics to calculate required change in joint positions to achieve desired tip pose. Inverse
 * kinematics is generated via the calculation of a jacobian for the current state of the leg, which is used as per
 * the Damped Least Squares method to generate a change in joint position for each joint.
 * @param[in] delta The iterative change in tip position and rotation
 * @param[in] solve_rotation Flag denoting if IK should solve for rotation as well rather than just position.
 * @todo Calculate optimal DLS coefficient (this value currently works sufficiently).
***********************************************************************************************************************/
VectorXd Leg::solveIK(const MatrixXd& delta, const bool& solve_rotation)
{
  // Calculate Jacobian from DH matrices along kinematic chain
  // ref: 
  shared_ptr<Joint> first_joint = joint_container_.begin()->second;
  Vector3d pe = tip_->getTransformFromJoint(first_joint->id_number_).block<3, 1>(0, 3);
  Vector3d z0(0, 0, 1);
  Vector3d p0(0, 0, 0);

  MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); //Linear velocity
  jacobian.block<3, 1>(3, 0) = solve_rotation ? z0 : Vector3d::Zero(); //Angular velocity

  JointContainer::iterator joint_it;
  int i = 1; //Skip first joint dh parameters since it is a fixed transformation
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    shared_ptr<Joint> joint = joint_it->second;
    Matrix4d t = joint->getTransformFromJoint(first_joint->id_number_);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); //Linear velocity
    jacobian.block<3, 1>(3, i) = solve_rotation ? t.block<3, 1>(0, 2) : Vector3d(0,0,0); //Angular velocity
  }

  MatrixXd identity = MatrixXd::Identity(6, 6);
  MatrixXd jacobian_inverse(joint_count_, 6);
  MatrixXd j = jacobian;
  
  // Calculate jacobian inverse using damped least squares method
  // ref: Chapter 5 of Introduction to Inverse Kinematics... , Samuel R. Buss 2009
  jacobian_inverse = j.transpose() * ((j * j.transpose() + sqr(DLS_COEFFICIENT) * identity).inverse()); //DLS Method
  
  // Generate joint limit cost function and gradient
  // ref: Chapter 2.4 of Autonomous Robots - Kinematics, Path Planning and Control, Farbod. Fahimi 2008
  i = 0;
  double cost = 0.0;
  VectorXd cost_gradient = VectorXd::Zero(joint_count_);
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {    
    shared_ptr<Joint> joint = joint_it->second;
    double joint_range = joint->max_position_ - joint->min_position_;
    double range_centre = joint->min_position_ + joint_range/2.0;
    if (joint_range != 0.0)
    {
      cost += sqr(abs(JOINT_LIMIT_COST_WEIGHT * (joint->desired_position_ - range_centre) / joint_range));
      cost_gradient[i] = -sqr(JOINT_LIMIT_COST_WEIGHT) * (joint->desired_position_ - range_centre) / sqr(joint_range);
    }
  }
  cost_gradient *= (cost == 0.0 ? 0.0 : 1.0 / sqrt(cost));

  // Calculate joint position change
  identity = MatrixXd::Identity(joint_count_, joint_count_);
  return jacobian_inverse * delta + (identity - jacobian_inverse * j) * cost_gradient;
}

/*******************************************************************************************************************//**
 * Updates the joint positions of each joint in this leg based on the input vector. Clamps joint velocities and
 * positions based on limits and calculates a ratio of proximity of joint position to limits.
 * @param[in] delta The iterative change in joint position for each joint.
 * @param[in] simulation Flag denoting if this execution is for simulation purposes rather than normal use.
 * @return The ratio of the proximity of the joint position to it's limits (i.e. 0.0 = at limit, 1.0 = furthest away)
***********************************************************************************************************************/
double Leg::updateJointPositions(const VectorXd& delta, const bool& simulation)
{
  int index = 0;
  string clamping_events;
  double min_limit_proximity = 1.0;
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    shared_ptr<Joint> joint = joint_it->second;
    joint->desired_velocity_ = delta[index] / model_->getTimeDelta();
    ROS_ASSERT(joint->desired_velocity_ < UNASSIGNED_VALUE);

    // Clamp joint velocities within limits
    if (params_.clamp_joint_velocities.data && !simulation)
    {
      if (abs(joint->desired_velocity_) > joint->max_angular_speed_)
      {
        double max_velocity = joint->max_angular_speed_;
        clamping_events += stringFormat("\n\tType: Velocity\tJoint: %s\tDesired: %f rad/s\tLimited to: %f rad/s",
                                        joint->id_name_.c_str(), abs(joint->desired_velocity_), max_velocity);
        joint->desired_velocity_ = clamped(joint->desired_velocity_, -max_velocity, max_velocity);
      }
    }
    
    joint->prev_desired_position_ = joint->desired_position_;
    joint->desired_position_ = joint->prev_desired_position_ + joint->desired_velocity_ * model_->getTimeDelta();

    // Clamp joint position within limits
    if (params_.clamp_joint_positions.data && !simulation)
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
    
    // Report clamping events
    ROS_WARN_COND(!clamping_events.empty() && !params_.ignore_IK_warnings.data,
                  "\nIK Clamping Event/s:%s\n", clamping_events.c_str());
  }
  
  return min_limit_proximity;
}

/*******************************************************************************************************************//**
 * Applies inverse kinematics solution to achieve desired tip position. Clamps joint positions and velocities
 * within limits and applies forward kinematics to update tip position. Returns an estimate of the chance of solving
 * IK within thresholds on the next iteration. 0.0 denotes failure on THIS iteration.
 * @param[in] simulation Flag denoting if this execution is for simulation purposes rather than normal use.
 * @return A double between 0.0 and 1.0 which estimates the chance of solving IK within thresholds on the next 
 * iteration. 0.0 denotes failure on THIS iteration.
***********************************************************************************************************************/
double Leg::applyIK(const bool& simulation)
{
  // Generate position delta vector in reference to the base of the leg
  shared_ptr<Joint> base_joint = joint_container_.begin()->second;
  Pose leg_frame_desired_tip_pose = base_joint->getPoseJointFrame(desired_tip_pose_);
  Pose leg_frame_current_tip_pose = base_joint->getPoseJointFrame(current_tip_pose_);
  Vector3d position_delta = leg_frame_desired_tip_pose.position_ - leg_frame_current_tip_pose.position_;
  ROS_ASSERT(position_delta.norm() < UNASSIGNED_VALUE);
  
  // EXPERIMENTAL
  /*
  Vector3d limit = 0.05*Vector3d(IK_TOLERANCE, IK_TOLERANCE, IK_TOLERANCE);
  position_delta = clamped(position_delta, limit);
  
  // Check for force constaints
  Vector3d tip_direction = current_tip_pose_.rotation_._transformVector(Vector3d::UnitX());
  Vector3d aligned_tip_force = getProjection(tip_force_, tip_direction);
  if (aligned_tip_force.norm() > 0.75)
  {
    Vector3d position_delta_projection = getProjection(position_delta, tip_direction);
    if (position_delta_projection.dot(tip_direction) > 0.0) // Same direction
    {
      position_delta = Vector3d::Zero();
    }
  }
  */
  
  MatrixXd delta = Matrix<double, 6, 1>::Zero();
  delta(0) = position_delta[0];
  delta(1) = position_delta[1];
  delta(2) = position_delta[2];
  
  // Calculate change in joint positions for change in tip position
  VectorXd joint_position_delta(joint_count_);
  joint_position_delta = solveIK(delta, false);
  
  // Update change in joint positions for change in tip rotation to desired tip rotation if defined
  bool rotation_constrained = !desired_tip_pose_.rotation_.isApprox(UNDEFINED_ROTATION);
  if (rotation_constrained)
  {
    // Update model
    updateJointPositions(joint_position_delta, true);
    applyFK();
    
    // Generate rotation delta vector in reference to the base of the leg
    Vector3d desired_tip_direction = leg_frame_desired_tip_pose.rotation_._transformVector(Vector3d::UnitX());
    Vector3d current_tip_direction = leg_frame_current_tip_pose.rotation_._transformVector(Vector3d::UnitX());
    Quaterniond difference = Quaterniond::FromTwoVectors(current_tip_direction, desired_tip_direction);
    AngleAxisd axis_rotation(difference.normalized());
    Vector3d rotation_delta = axis_rotation.axis() * axis_rotation.angle();
    delta = Matrix<double, 6, 1>::Zero();
    delta(3) = rotation_delta[0];
    delta(4) = rotation_delta[1];
    delta(5) = rotation_delta[2];
    joint_position_delta = solveIK(delta, true);
  }

  // Update Model
  double ik_success = updateJointPositions(joint_position_delta, simulation);
  applyFK();

  // Debugging message
  ROS_DEBUG_COND(id_number_ == 0 && params_.debug_IK.data,
                 "\nLeg %s:\n\tDesired tip position from trajectory engine: %f:%f:%f\n\t"
                 "Resultant tip position from inverse/forward kinematics: %f:%f:%f", id_name_.c_str(),
                 desired_tip_pose_.position_[0], desired_tip_pose_.position_[1], desired_tip_pose_.position_[2],
                 current_tip_pose_.position_[0], current_tip_pose_.position_[1], current_tip_pose_.position_[2]);

  // Display warning messages for associated inverse kinematic deviations
  string axis_label[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i)
  {
    Vector3d position_error = current_tip_pose_.position_ - desired_tip_pose_.position_;
    if (abs(position_error[i]) > IK_TOLERANCE)
    {
      ik_success = 0.0;
      ROS_WARN_COND(!simulation && !params_.ignore_IK_warnings.data,
                    "\nInverse kinematics deviation! Calculated tip %s position of leg %s (%s: %f)"
                    " differs from desired tip position (%s: %f)\n",
                    axis_label[i].c_str(), id_name_.c_str(),
                    axis_label[i].c_str(), current_tip_pose_.position_[i],
                    axis_label[i].c_str(), desired_tip_pose_.position_[i]);
    }
  }
  
  // If IK fails because of constrained rotation - try again with unconstrained rotation
  if (rotation_constrained && !ik_success)
  {
    desired_tip_pose_.rotation_ = UNDEFINED_ROTATION;
    ik_success = applyIK(simulation);
  }
  
  return ik_success;
}

/*******************************************************************************************************************//**
 * Updates joint transforms and applies forward kinematics to calculate a new tip pose. 
 * Sets leg current tip pose to new pose if requested.
 * @param[in] set_current Flag denoting of the calculated tip pose should be set as the current tip pose.
***********************************************************************************************************************/
Pose Leg::applyFK(const bool& set_current)
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
  Pose tip_pose = tip_->getPoseRobotFrame();
  if (set_current)
  {
    if (current_tip_pose_ != Pose::Undefined())
    {
      current_tip_velocity_ = (tip_pose.position_ - current_tip_pose_.position_) / model_->getTimeDelta();
    }
    current_tip_pose_ = tip_pose;
  }
  
  ROS_ASSERT(current_tip_pose_.isValid());
  
  return tip_pose;
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
  if (!params.link_parameters[leg->getIDNumber()][id_number_].initialised)
  {
    ROS_FATAL("\nModel initialisation error for %s\n", id_name_.c_str());
    ros::shutdown();
  }
}

/*******************************************************************************************************************//**
 * Copy Constructor for Link object. Initialises member variables from existing Link object.
 * @param[in] link A pointer to an existing link object.
***********************************************************************************************************************/
Link::Link(shared_ptr<Link> link)
  : parent_leg_(link->parent_leg_)
  , actuating_joint_(link->actuating_joint_)
  , id_number_(link->id_number_)
  , id_name_(link->id_name_)
  , dh_parameter_r_(link->dh_parameter_r_)
  , dh_parameter_theta_(link->dh_parameter_theta_)
  , dh_parameter_d_(link->dh_parameter_d_)
  , dh_parameter_alpha_(link->dh_parameter_alpha_)
{
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
  , unpacked_position_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("unpacked"))
  , max_angular_speed_(params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data.at("max_vel"))
{
  default_position_ = clamped(0.0, min_position_, max_position_);
  
  // Populate packed configuration/s joint position/s
  map<string, double> joint_parameters = params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data;
  bool get_next_packed_position = true;
  string packed_position_key = "packed";
  int i = 0;
  while (get_next_packed_position)
  {
    try 
    {
      packed_positions_.push_back(joint_parameters.at(packed_position_key));
      get_next_packed_position = (packed_position_key != "packed");
    } 
    catch (out_of_range)
    {
      get_next_packed_position = (packed_position_key == "packed");
    }
    packed_position_key = "packed_" + numberToString(i);
    i++;
  }
  
  if (params.joint_parameters[leg->getIDNumber()][id_number_ - 1].initialised)
  {
    identity_transform_ = createDHMatrix(reference_link_->dh_parameter_d_,
                                         reference_link_->dh_parameter_theta_,
                                         reference_link_->dh_parameter_r_,
                                         reference_link_->dh_parameter_alpha_);
    current_transform_ = identity_transform_;
  }
  else
  {
    ROS_FATAL("\nModel initialisation error for %s\n", id_name_.c_str());
    ros::shutdown();
  }
}

/*******************************************************************************************************************//**
 * Copy Constructor for Joint object. Initialises member variables from existing Joint object.
 * @param[in] joint A pointer to an existing Joint object.
***********************************************************************************************************************/
Joint::Joint(shared_ptr<Joint> joint)
  : parent_leg_(joint->parent_leg_)
  , reference_link_(joint->reference_link_)
  , id_number_(joint->id_number_)
  , id_name_(joint->id_name_)
  , min_position_(joint->min_position_)
  , max_position_(joint->max_position_)
  , packed_positions_(joint->packed_positions_)
  , unpacked_position_(joint->unpacked_position_)
  , max_angular_speed_(joint->max_angular_speed_)
{
  current_transform_ = joint->current_transform_;
  identity_transform_ = joint->identity_transform_;

  desired_position_publisher_ = joint->desired_position_publisher_;

  desired_position_ = joint->desired_position_;
  desired_velocity_ = joint->desired_velocity_;
  desired_effort_ = joint->desired_effort_;
  prev_desired_position_ = joint->prev_desired_position_;
  prev_desired_velocity_ = joint->prev_desired_velocity_;
  prev_desired_effort_ = joint->prev_desired_effort_;

  current_position_ = joint->current_position_;
  current_velocity_ = joint->current_velocity_;
  current_effort_ = joint->current_effort_;
  
  default_position_ = joint->default_position_;
  default_velocity_ = joint->default_velocity_;
  default_effort_ = joint->default_effort_;
}

/*******************************************************************************************************************//**
 * Constructor for null joint object. Acts as a null joint object for use in ending kinematic chains.
***********************************************************************************************************************/
Joint::Joint(void)
  : parent_leg_(NULL)
  , reference_link_(NULL)
  , id_number_(0)
  , id_name_("origin")
{
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

/*******************************************************************************************************************//**
 * Copy Constructor for Tip object. Initialises member variables from existing Tip object.
 * @param[in] tip A pointer to an existing tip object.
***********************************************************************************************************************/
Tip::Tip(shared_ptr<Tip> tip)
  : parent_leg_(tip->parent_leg_)
  , reference_link_(tip->reference_link_)
  , id_name_(tip->id_name_)
{
  identity_transform_ = tip->identity_transform_;
  current_transform_ = tip->current_transform_;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
