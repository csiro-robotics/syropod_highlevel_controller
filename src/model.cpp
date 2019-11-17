////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../include/syropod_highlevel_controller/model.h"
#include "../include/syropod_highlevel_controller/walk_controller.h"
#include "../include/syropod_highlevel_controller/pose_controller.h"
#include "../include/syropod_highlevel_controller/debug_visualiser.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Model::Model(const Parameters& params, std::shared_ptr<DebugVisualiser> debug_visualiser)
  : params_(params)
  , debug_visualiser_(debug_visualiser)
  , leg_count_(params_.leg_id.data.size())
  , time_delta_(params_.time_delta.data)
  , current_pose_(Pose::Identity())
  , default_pose_(Pose::Identity())
{
  imu_data_.orientation = UNDEFINED_ROTATION;
  imu_data_.linear_acceleration = Eigen::Vector3d::Zero();
  imu_data_.angular_velocity = Eigen::Vector3d::Zero();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Model::Model(std::shared_ptr<Model> model)
  : params_(model->params_)
  , debug_visualiser_(model->debug_visualiser_)
  , leg_count_(model->leg_count_)
  , time_delta_(model->time_delta_)
  , current_pose_(model->current_pose_)
  , default_pose_(model->default_pose_)
  , imu_data_(model->imu_data_)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::generate(std::shared_ptr<Model> model)
{
  for (int i = 0; i < leg_count_; ++i)
  {
    std::shared_ptr<Leg> leg;
    if (model != NULL)
    {
      std::shared_ptr<Leg> reference_leg = model->leg_container_.find(i)->second;
      leg = std::allocate_shared<Leg>(Eigen::aligned_allocator<Leg>(), reference_leg, shared_from_this());
      leg->generate(reference_leg);
    }
    else
    {
      leg = std::allocate_shared<Leg>(Eigen::aligned_allocator<Leg>(), shared_from_this(), i, params_);
      leg->generate();
    }
    leg_container_.insert(LegContainer::value_type(i, leg));
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::initLegs(const bool& use_default_joint_positions)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    leg->init(use_default_joint_positions);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Model::legsBearingLoad(void)
{
  double body_height_estimate = 0.0;
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    body_height_estimate += leg->getCurrentTipPose().position_[2];
  }
  return -(body_height_estimate / leg_count_) > HALF_BODY_DEPTH; // TODO Parameterise this value
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Leg> Model::getLegByIDName(const std::string& leg_id_name)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    if (leg->getIDName() == leg_id_name)
    {
      return leg;
    }
  }
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::updateDefaultConfiguration(void)
{
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    leg->updateDefaultConfiguration();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::generateWorkspaces(void)
{
  // Create copy of model for searching for kinematic limitations
  std::shared_ptr<Model> search_model = std::allocate_shared<Model>(Eigen::aligned_allocator<Model>(),
                                                                    shared_from_this());
  search_model->generate(shared_from_this());
  search_model->initLegs(true);
  
  // Run workspace generation for each leg in model
  LegContainer::iterator leg_it;
  for (leg_it = search_model->getLegContainer()->begin(); leg_it != search_model->getLegContainer()->end(); ++leg_it)
  {
    std::shared_ptr<Leg> search_leg = leg_it->second;
    ROS_INFO("\n[SHC] Generating workspace (%d%%) . . .\n", roundToInt(100.0 * search_leg->getIDNumber() / leg_count_));
    std::shared_ptr<Leg> leg = leg_container_.at(search_leg->getIDNumber());
    leg->setWorkspace(search_leg->generateWorkspace());
  }
  ROS_INFO("\n[SHC] Generating workspace (100%%) . . .\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Model::updateModel(void)
{
  // Model uses posed tip positions, adds deltaZ from admittance controller and applies inverse kinematics on each leg
  LegContainer::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    leg->setDesiredTipPose();
    leg->applyIK();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d Model::estimateGravity(void)
{
  Eigen::Vector3d euler = quaternionToEulerAngles(imu_data_.orientation);
  Eigen::AngleAxisd pitch(-euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll(-euler[0], Eigen::Vector3d::UnitX());
  Eigen::Vector3d gravity(0, 0, GRAVITY_ACCELERATION);
  gravity = pitch * gravity;
  gravity = roll * gravity;
  return gravity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Leg::Leg(std::shared_ptr<Model> model, const int& id_number, const Parameters& params)
  : model_(model)
  , params_(params)
  , id_number_(id_number)
  , id_name_(params_.leg_id.data.at(id_number))
  , joint_count_(params_.leg_DOF.data.at(id_name_))
  , leg_state_(WALKING)
  , admittance_delta_(Eigen::Vector3d::Zero())
  , admittance_state_(std::vector<double>(2))
{
  desired_tip_pose_ = Pose::Undefined();
  desired_tip_velocity_ = Eigen::Vector3d::Zero();
  current_tip_pose_ = Pose::Undefined();
  current_tip_velocity_ = Eigen::Vector3d::Zero();
  tip_force_calculated_ = Eigen::Vector3d::Zero();
  tip_force_measured_ = Eigen::Vector3d::Zero();
  tip_torque_calculated_ = Eigen::Vector3d::Zero();
  tip_torque_measured_ = Eigen::Vector3d::Zero();
  step_plane_pose_ = Pose::Undefined();
  group_ = (id_number % 2); // Even/odd groups
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Leg::Leg(std::shared_ptr<Leg> leg, std::shared_ptr<Model> model)
  : params_(leg->params_)
  , id_number_(leg->id_number_)
  , id_name_(leg->id_name_)
  , joint_count_(leg->joint_count_)
  , leg_state_(leg->leg_state_)
  , admittance_state_(leg->admittance_state_)
{
  model_ = (model == NULL ? leg->model_ : model);
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
  tip_force_calculated_ = leg->tip_force_calculated_;
  tip_force_measured_ = leg->tip_force_measured_;
  tip_torque_calculated_ = leg->tip_torque_calculated_;
  tip_torque_measured_ = leg->tip_torque_calculated_;
  step_plane_pose_ = leg->step_plane_pose_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::generate(std::shared_ptr<Leg> leg)
{
  // Null joint acts as origin
  std::shared_ptr<Joint> null_joint = std::allocate_shared<Joint>(Eigen::aligned_allocator<Joint>()); 
  std::shared_ptr<Link> base_link = 
    std::allocate_shared<Link>(Eigen::aligned_allocator<Link>(), shared_from_this(), null_joint, 0, params_);
  link_container_.insert(LinkContainer::value_type(0, base_link));
  std::shared_ptr<Link> prev_link = base_link;
  for (int i = 1; i < joint_count_ + 1; ++i)
  {
    std::shared_ptr<Joint> new_joint = 
      std::allocate_shared<Joint>(Eigen::aligned_allocator<Joint>(), shared_from_this(), prev_link, i, params_);
    std::shared_ptr<Link> new_link = 
      std::allocate_shared<Link>(Eigen::aligned_allocator<Link>(), shared_from_this(), new_joint, i, params_);
    joint_container_.insert(JointContainer::value_type(i, new_joint));
    link_container_.insert(LinkContainer::value_type(i, new_link));
    prev_link = new_link;
  }
  tip_ = std::allocate_shared<Tip>(Eigen::aligned_allocator<Tip>(), shared_from_this(), prev_link);

  // If given reference leg, copy member element variables to this leg object
  if (leg != NULL)
  {
    // Copy Joint variables
    JointContainer::iterator joint_it;
    for (joint_it = leg->joint_container_.begin(); joint_it != leg->joint_container_.end(); ++joint_it)
    {
      std::shared_ptr<Joint> old_joint = joint_it->second;
      std::shared_ptr<Joint> new_joint = joint_container_.find(old_joint->id_number_)->second;
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
    leg_stepper_ = std::allocate_shared<LegStepper>(Eigen::aligned_allocator<LegStepper>(), leg->getLegStepper());
    leg_stepper_->setParentLeg(shared_from_this());
    
    // Copy LegPoser
    leg_poser_ = std::allocate_shared<LegPoser>(Eigen::aligned_allocator<LegPoser>(), leg->getLegPoser());
    leg_poser_->setParentLeg(shared_from_this());
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::init(const bool& use_default_joint_positions)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Workspace Leg::generateWorkspace(void)
{
  bool debug = params_.debug_workspace_calc.data;
  bool display_debug_visualisation = debug && params_.debug_rviz.data;
  bool workspace_generation_complete = false;
  bool simple_workspace = !params_.rough_terrain_mode.data;
  
  // Publish static transforms for visualisation purposes
  if (display_debug_visualisation)
  {
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_odom_ideal_to_base_link;
    static_odom_ideal_to_base_link.header.stamp = ros::Time::now();
    static_odom_ideal_to_base_link.header.frame_id = "odom_ideal";
    static_odom_ideal_to_base_link.child_frame_id = "base_link";
    static_odom_ideal_to_base_link.transform = model_->getCurrentPose().toTransformMessage();
    static_broadcaster.sendTransform(static_odom_ideal_to_base_link);
    geometry_msgs::TransformStamped static_base_link_to_walk_plane = static_odom_ideal_to_base_link;
    static_base_link_to_walk_plane.header.frame_id = "base_link";
    static_base_link_to_walk_plane.child_frame_id = "walk_plane";
    static_base_link_to_walk_plane.transform = (~model_->getCurrentPose()).toTransformMessage();
    static_broadcaster.sendTransform(static_base_link_to_walk_plane);
  }
  
  // Init maximal/minimal workplanes
  Workplane max_workplane;
  Workplane min_workplane;
  for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP)
  {
    max_workplane.insert(Workplane::value_type(bearing, MAX_WORKSPACE_RADIUS));
    min_workplane.insert(Workplane::value_type(bearing, 0.0));
  }
  workspace_.clear();
  
  // Calculate Identity tip pose
  Pose current_pose = model_->getCurrentPose();
  Eigen::Vector3d identity_tip_position = 
    current_pose.inverseTransformVector(leg_stepper_-> getIdentityTipPose().position_);
  
  // Set zero workspace if unable to reach idenity_tip_pose
  if ((identity_tip_position - current_tip_pose_.position_).norm() > IK_TOLERANCE)
  {
    workspace_.insert(Workspace::value_type(0.0, min_workplane));
    return workspace_;
  }

  if (simple_workspace)
  {
    workspace_.insert(Workspace::value_type(0.0, max_workplane));
  }

  bool found_lower_limit = simple_workspace ? true : false;
  bool found_upper_limit = simple_workspace ? true : false;
  double max_plane_height = simple_workspace ? 0.0 : MAX_WORKSPACE_RADIUS;
  double min_plane_height = simple_workspace ? 0.0 : -MAX_WORKSPACE_RADIUS;
  double search_height_delta = MAX_WORKSPACE_RADIUS / WORKSPACE_LAYERS;
  double search_height = 0.0;
  int search_bearing = 0;
  bool within_limits = true;
  int iteration = 1;
  Eigen::Vector3d origin_tip_position, target_tip_position;
  double distance_from_origin;
  int number_iterations;
  
  // Iterate through all legs and move legs along search bearings to kinematic limits
  while(true)
  {
    Pose current_pose = model_->getCurrentPose();
    Eigen::Vector3d identity_tip_position = 
      current_pose.inverseTransformVector(leg_stepper_->getIdentityTipPose().position_);
    identity_tip_position[2] += search_height;
    
    // Set origin and target for linear interpolation
    if (iteration == 1)
    {
      within_limits = true;
      init(true); // Reset leg to default stance
      // Search for upper and lower vertical limit of workspace
      if (!found_lower_limit || !found_upper_limit)
      {
        number_iterations = roundToInt(MAX_WORKSPACE_RADIUS / MAX_POSITION_DELTA);
        origin_tip_position = identity_tip_position;
        Eigen::Vector3d search_limit = 
          (found_lower_limit ? MAX_WORKSPACE_RADIUS : -MAX_WORKSPACE_RADIUS) * Eigen::Vector3d::UnitZ();
        target_tip_position = identity_tip_position + search_limit;
      }
      // Track to new workplane origin at search height
      else if (search_bearing == 0)
      {
        number_iterations = roundToInt(search_height_delta / MAX_POSITION_DELTA);
        origin_tip_position = current_tip_pose_.position_;
        target_tip_position = identity_tip_position;
      }
      // Search along search bearing for limits
      else
      {
        number_iterations = roundToInt(MAX_WORKSPACE_RADIUS / MAX_POSITION_DELTA);
        origin_tip_position = identity_tip_position;
        target_tip_position = origin_tip_position;
        target_tip_position[0] += MAX_WORKSPACE_RADIUS * cos(degreesToRadians(search_bearing));
        target_tip_position[1] += MAX_WORKSPACE_RADIUS * sin(degreesToRadians(search_bearing));
      }
    }
  
    // Move tip position linearly along search bearing in search of kinematic workspace limit
    double i = double(iteration) / number_iterations; // Interpolation control variable
    Eigen::Vector3d desired_tip_position = origin_tip_position * (1.0 - i) + target_tip_position * i; // Interpolate
    // Quaterniond desired_tip_rotation = leg_stepper->getIdentityTipPose().rotation_;
    setDesiredTipPose(Pose(desired_tip_position, UNDEFINED_ROTATION));
    double ik_result = applyIK(true);
    distance_from_origin = Eigen::Vector3d(current_tip_pose_.position_ - identity_tip_position).norm();
    
    // Check if leg is still within limits
    within_limits = within_limits && ik_result != 0.0;
    
    // Display debugging messages
    ROS_DEBUG_COND(debug && search_bearing != 0,
                    "LEG: %s\tSEARCH: %f:%d:%d\tDISTANCE: %f\tIK_RESULT: %f\tWITHIN LIMITS: %s",
                    id_name_.c_str(), search_height, search_bearing,
                    iteration, distance_from_origin, ik_result, within_limits ? "TRUE" : "FALSE");

    // Search not complete -> iterate along current search bearing
    if (within_limits && iteration < number_iterations)
    {
      iteration++;
    }
    // Current search along bearing complete -> save min distance and iterate search bearing
    else 
    {
      iteration = 1;
      // Lower vertical limit found - reset to search for upper vertical limit
      if (!found_lower_limit)
      {
        found_lower_limit = true;
        min_plane_height = -distance_from_origin;
        workspace_.insert(Workspace::value_type(min_plane_height, min_workplane));
        continue;
      }
      // Upper vertical limit found - reset to start searching for limits within intermediate workplanes
      else if (!found_upper_limit)
      {
        found_upper_limit = true;
        max_plane_height = distance_from_origin;
        search_height_delta = (max_plane_height - min_plane_height) / WORKSPACE_LAYERS;
        int upper_levels = int(abs(max_plane_height) / search_height_delta);
        search_height = upper_levels * search_height_delta;
        workspace_.insert(Workspace::value_type(max_plane_height, min_workplane));
        workspace_.insert(Workspace::value_type(search_height, max_workplane));
        continue;
      }
      // Tracked to origin of new workplane, update default configuration to easily reset betweeen search bearings
      else if (search_bearing == 0)
      {
        updateDefaultConfiguration();
      }
      // Search along bearing complete - save in workspace
      else
      {
        workspace_.at(search_height)[search_bearing] = distance_from_origin;
      }

      // Iterate search bearing (0 -> 360 anti-clockwise)
      if (search_bearing + BEARING_STEP <= 360)
      {
        search_bearing += BEARING_STEP;
      }
      // Iterate search height (top to bottom)
      else
      {
        search_bearing = 0;
        workspace_.at(search_height)[0] = workspace_.at(search_height).at(360);
        search_height -= search_height_delta;
        if (search_height >= min_plane_height)
        {
          workspace_.insert(Workspace::value_type(search_height, max_workplane));
        }
        // All searches complete - set workspace generation complete and reset
        else
        {
          workspace_generation_complete = true;;
        }
      }
    }
    // Display robot model and workspace for debugging purposes
    if (display_debug_visualisation)
    {
      std::shared_ptr<DebugVisualiser> debug = model_->getDebugVisualiser();
      debug->generateRobotModel(model_);
      debug->generateWorkspace(shared_from_this(), params_.body_clearance.data);
      ros::Rate r(100);
      ros::spinOnce();
      r.sleep();
    }
    if (workspace_generation_complete)
    {
      return workspace_;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Workplane Leg::getWorkplane(const double& height)
{  
  bool within_workspace = (height >= workspace_.begin()->first && height <= workspace_.rbegin()->first);
  if (!within_workspace)
  {
    ROS_WARN("\n[SHC] Requested workplane does not exist within workspace.\n");
    Workplane undefined;
    return undefined;
  }
  else if (workspace_.size() == 1)
  {
    return workspace_.at(0.0);
  }
  
  // Get bounding existing workplanes within workspace
  ROS_ASSERT(workspace_.size() > 1);
  Workspace::iterator upper_bound_it = workspace_.upper_bound(height);
  Workspace::iterator lower_bound_it = prev(upper_bound_it);
  double upper_workplane_height = setPrecision(upper_bound_it->first, 3);
  double lower_workplane_height = setPrecision(lower_bound_it->first, 3);
  Workplane upper_workplane = upper_bound_it->second;
  Workplane lower_workplane = lower_bound_it->second;
  
  // Calculate interpolation value of target workplane height between existing workspace planes
  double i = (height - lower_workplane_height) / (upper_workplane_height - lower_workplane_height);
  
  // Generate workplane radii
  Workplane workplane = upper_workplane;
  Workplane::iterator workplane_it;
  for (workplane_it = workplane.begin(); workplane_it != workplane.end(); ++workplane_it)
  {
    int bearing = workplane_it->first;
    double radius = lower_workplane.at(bearing) * (1.0 - i) + upper_workplane.at(bearing) * i;
    workplane[bearing] = within_workspace ? radius : 0.0;
  }
  return workplane;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d Leg::makeReachable(const Eigen::Vector3d& reference_tip_position)
{
  // Get workplane containing test tip position
  Pose pose = model_->getCurrentPose();
  Eigen::Vector3d test_tip_position = pose.inverseTransformVector(reference_tip_position);
  Eigen::Vector3d identity_tip_position = leg_stepper_->getIdentityTipPose().position_;
  Eigen::Vector3d identity_to_test = test_tip_position - identity_tip_position;
  double distance_to_test = Eigen::Vector2d(identity_to_test[0], identity_to_test[1]).norm();
  LimitMap workplane = getWorkplane(test_tip_position[2]);

  // Find distance to workplane limit along bearing to test tip position
  double raw_bearing = atan2(test_tip_position[1], test_tip_position[0]);
  int bearing = mod(roundToInt(radiansToDegrees(raw_bearing)), 360);
  int upper_bound = workplane.lower_bound(bearing)->first;
  int lower_bound = mod(upper_bound - BEARING_STEP, 360);
  bearing += (bearing < lower_bound) ? 360 : 0;
  upper_bound += (upper_bound < lower_bound) ? 360 : 0;
  double interpolation_progress = (bearing - lower_bound) / (upper_bound - lower_bound);
  double distance_to_limit = interpolate(workplane.at(lower_bound),
                                         workplane.at(mod(upper_bound, 360)),
                                         interpolation_progress);
  
  // If test tip position is beyond limit, calculate new position along same workplane bearing within limits
  if (distance_to_test >  distance_to_limit)
  {
    Eigen::Vector3d new_tip_position = 
      Eigen::AngleAxisd(raw_bearing, Eigen::Vector3d::UnitZ()) * (Eigen::Vector3d::UnitX() * distance_to_limit);
    new_tip_position[2] = test_tip_position[2];
    new_tip_position = pose.transformVector(new_tip_position);
    return new_tip_position;
  }
  else
  {
    return reference_tip_position;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::updateDefaultConfiguration(void)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    joint->default_position_ = joint->desired_position_;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::generateDesiredJointStateMsg(sensor_msgs::JointState* joint_state_msg)
{
  joint_state_msg->header.stamp = ros::Time::now();
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    joint_state_msg->name.push_back(joint->id_name_);
    joint_state_msg->position.push_back(joint->desired_position_);
    joint_state_msg->velocity.push_back(joint->desired_velocity_);
    joint_state_msg->effort.push_back(joint->desired_effort_);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Joint> Leg::getJointByIDName(const std::string& joint_id_name)
{
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    if (joint->id_name_ == joint_id_name)
    {
      return joint;
    }
  }
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Link> Leg::getLinkByIDName(const std::string& link_id_name)
{
  LinkContainer::iterator link_it;
  for (link_it = link_container_.begin(); link_it != link_container_.end(); ++link_it)
  {
    std::shared_ptr<Link> link = link_it->second;
    if (link->id_name_ == link_id_name)
    {
      return link;
    }
  }
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::setDesiredTipPose(const Pose& tip_pose, bool apply_delta)
{
  // Don't apply delta to manually manipulated legs
  apply_delta = apply_delta && !(leg_state_ == MANUAL || leg_state_ == WALKING_TO_MANUAL);

  bool use_poser_tip_pose = (Pose::Undefined() == tip_pose);

  desired_tip_pose_ = use_poser_tip_pose ? leg_poser_->getCurrentTipPose() : tip_pose;
  desired_tip_pose_.position_ += (apply_delta ? admittance_delta_ : Eigen::Vector3d::Zero());
  ROS_ASSERT(desired_tip_pose_.isValid());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::calculateTipForce(void)
{
  std::shared_ptr<Joint> first_joint = joint_container_.begin()->second;

  Eigen::Vector3d pe = tip_->getTransformFromJoint(first_joint->id_number_).block<3, 1>(0, 3);
  Eigen::Vector3d z0(0, 0, 1);
  Eigen::Vector3d p0(0, 0, 0);

  Eigen::MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); // Linear velocity
  jacobian.block<3, 1>(3, 0) = z0; // Angular velocity

  Eigen::VectorXd joint_torques(joint_count_);
  joint_torques[0] = first_joint->current_effort_;
  
  // Skip first joint dh parameters since it is a fixed transformation
  int i = 1;
  JointContainer::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    Eigen::Matrix4d t = joint->getTransformFromJoint(first_joint->id_number_);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); // Linear velocity
    jacobian.block<3, 1>(3, i) = t.block<3, 1>(0, 2); // Angular velocity
    joint_torques[i] = joint->current_effort_;
  }

  // Transpose and invert jacobian
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(joint_count_, joint_count_);
  Eigen::MatrixXd transformation = 
    jacobian * ((jacobian.transpose()*jacobian + sqr(DLS_COEFFICIENT) * identity).inverse());
  
  Eigen::VectorXd raw_tip_force_leg_frame = transformation * joint_torques;
  Eigen::Quaterniond rotation = (first_joint->getPoseJointFrame()).rotation_;
  Eigen::VectorXd raw_tip_force = rotation._transformVector(raw_tip_force_leg_frame.block<3, 1>(0, 0));
  
  // Low pass filter and force gain applied to calculated raw tip force
  double s = 0.15; // Smoothing Factor
  tip_force_calculated_[0] = s*raw_tip_force[0]*params_.force_gain.current_value + (1 - s)*tip_force_calculated_[0];
  tip_force_calculated_[1] = s*raw_tip_force[1]*params_.force_gain.current_value + (1 - s)*tip_force_calculated_[1];
  tip_force_calculated_[2] = s*raw_tip_force[2]*params_.force_gain.current_value + (1 - s)*tip_force_calculated_[2];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Leg::touchdownDetection(void)
{
  if (tip_force_measured_.norm() > params_.touchdown_threshold.data && step_plane_pose_ == Pose::Undefined())
  {
    step_plane_pose_ = current_tip_pose_;
  }
  else if (tip_force_measured_.norm() < params_.liftoff_threshold.data)
  {
    step_plane_pose_ = Pose::Undefined();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd Leg::solveIK(const Eigen::MatrixXd& delta, const bool& solve_rotation)
{
  // Calculate Jacobian from DH matrices along kinematic chain. Ref:
  // robotics.stackexchange.com/questions/2760/computing-inverse-kinematic-with-jacobian-matrices-for-6-dof-manipulator
  std::shared_ptr<Joint> first_joint = joint_container_.begin()->second;
  Eigen::Vector3d pe = tip_->getTransformFromJoint(first_joint->id_number_).block<3, 1>(0, 3);
  Eigen::Vector3d z0(0, 0, 1);
  Eigen::Vector3d p0(0, 0, 0);

  Eigen::MatrixXd jacobian(6, joint_count_);
  jacobian.block<3, 1>(0, 0) = z0.cross(pe - p0); // Linear velocity
  jacobian.block<3, 1>(3, 0) = solve_rotation ? z0 : Eigen::Vector3d::Zero(); // Angular velocity

  JointContainer::iterator joint_it;
  int i = 1; // Skip first joint dh parameters since it is a fixed transformation
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    Eigen::Matrix4d t = joint->getTransformFromJoint(first_joint->id_number_);
    jacobian.block<3, 1>(0, i) = t.block<3, 1>(0, 2).cross(pe - t.block<3, 1>(0, 3)); // Linear velocity
    jacobian.block<3, 1>(3, i) = solve_rotation ? t.block<3, 1>(0, 2) : Eigen::Vector3d(0,0,0); // Angular velocity
  }

  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd jacobian_inverse(joint_count_, 6);
  Eigen::MatrixXd j = jacobian;
  
  // Calculate jacobian inverse using damped least squares method
  // REF: Chapter 5 of Introduction to Inverse Kinematics... , Samuel R. Buss 2009
  jacobian_inverse = j.transpose() * ((j * j.transpose() + sqr(DLS_COEFFICIENT) * identity).inverse()); //DLS Method
  
  // Generate joint limit cost function and gradient
  // REF: Chapter 2.4 of Autonomous Robots - Kinematics, Path Planning and Control, Farbod. Fahimi 2008
  i = 0;
  double position_limit_cost = 0.0;
  double velocity_limit_cost = 0.0;
  Eigen::VectorXd position_cost_gradient = Eigen::VectorXd::Zero(joint_count_);
  Eigen::VectorXd velocity_cost_gradient = Eigen::VectorXd::Zero(joint_count_);
  Eigen::VectorXd combined_cost_gradient = Eigen::VectorXd::Zero(joint_count_);
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++i)
  {
    std::shared_ptr<Joint> joint = joint_it->second;

    // POSITION LIMITS
    double joint_position_range = joint->max_position_ - joint->min_position_;
    double position_range_centre = joint->min_position_ + joint_position_range/2.0;
    if (joint_position_range != 0.0)
    {
      position_limit_cost += 
        sqr(abs(JOINT_LIMIT_COST_WEIGHT * (joint->desired_position_ - position_range_centre) / joint_position_range));
      position_cost_gradient[i] = 
        -sqr(JOINT_LIMIT_COST_WEIGHT) * (joint->desired_position_ - position_range_centre) / sqr(joint_position_range);
    }

    // VELOCITY LIMITS
    double joint_velocity_range = 2 * joint->max_angular_speed_;
    double velocity_range_centre = 0.0;
    velocity_limit_cost += 
      sqr(abs(JOINT_LIMIT_COST_WEIGHT * (joint->desired_velocity_ - velocity_range_centre) / joint_velocity_range));
    velocity_cost_gradient[i] = 
      -sqr(JOINT_LIMIT_COST_WEIGHT) * (joint->desired_velocity_ - velocity_range_centre) / sqr(joint_velocity_range);
  }
  position_cost_gradient *= (position_limit_cost == 0.0 ? 0.0 : 1.0 / sqrt(position_limit_cost));
  velocity_cost_gradient *= (velocity_limit_cost == 0.0 ? 0.0 : 1.0 / sqrt(velocity_limit_cost));
  combined_cost_gradient = interpolate(position_cost_gradient, velocity_cost_gradient, 0.75);

  // Calculate joint position change
  identity = Eigen::MatrixXd::Identity(joint_count_, joint_count_);
  return jacobian_inverse * delta + (identity - jacobian_inverse * j) * combined_cost_gradient;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Leg::updateJointPositions(const Eigen::VectorXd& delta, const bool& simulation)
{
  int index = 0;
  std::string clamping_events;
  double min_limit_proximity = 1.0;
  JointContainer::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it, ++index)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
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
    if (params_.clamp_joint_positions.data)
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

    // Calculates the proximity of the joint closest to one of it's limits. Used for preventing exceeding workspace.
    // (1.0 = furthest possible from limit, 0.0 = equal to limit)
    double min_diff = abs(joint->min_position_ - joint->desired_position_);
    double max_diff = abs(joint->max_position_ - joint->desired_position_);
    double half_joint_range = (joint->max_position_ - joint->min_position_) / 2.0;
    double limit_proximity = half_joint_range != 0 ? std::min(min_diff, max_diff) / half_joint_range : 1.0;
    min_limit_proximity = std::min(limit_proximity, min_limit_proximity);
    
    // Report clamping events
    ROS_WARN_COND(!clamping_events.empty() && !params_.ignore_IK_warnings.data && !simulation,
                  "\nIK Clamping Event/s:%s\n", clamping_events.c_str());
  }
  
  return min_limit_proximity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Leg::applyIK(const bool& simulation)
{
  // Generate position delta vector in reference to the base of the leg
  std::shared_ptr<Joint> base_joint = joint_container_.begin()->second;
  Pose leg_frame_desired_tip_pose = base_joint->getPoseJointFrame(desired_tip_pose_);
  Pose leg_frame_current_tip_pose = base_joint->getPoseJointFrame(current_tip_pose_);
  Eigen::Vector3d position_delta = leg_frame_desired_tip_pose.position_ - leg_frame_current_tip_pose.position_;
  ROS_ASSERT(position_delta.norm() < UNASSIGNED_VALUE);
  
  Eigen::MatrixXd delta = Eigen::Matrix<double, 6, 1>::Zero();
  delta(0) = position_delta[0];
  delta(1) = position_delta[1];
  delta(2) = position_delta[2];
  
  // Calculate change in joint positions for change in tip position
  Eigen::VectorXd joint_position_delta(joint_count_);
  joint_position_delta = solveIK(delta, false);
  
  // Update change in joint positions for change in tip rotation to desired tip rotation if defined
  bool rotation_constrained = !desired_tip_pose_.rotation_.isApprox(UNDEFINED_ROTATION);
  if (rotation_constrained)
  {
    // Update model
    updateJointPositions(joint_position_delta, true);
    applyFK();
    
    // Generate rotation delta vector in reference to the base of the leg
    Eigen::Vector3d desired_tip_direction = 
      leg_frame_desired_tip_pose.rotation_._transformVector(Eigen::Vector3d::UnitX());
    Eigen::Vector3d current_tip_direction = 
      leg_frame_current_tip_pose.rotation_._transformVector(Eigen::Vector3d::UnitX());
    Eigen::Quaterniond difference = Eigen::Quaterniond::FromTwoVectors(current_tip_direction, desired_tip_direction);
    Eigen::AngleAxisd axis_rotation(difference.normalized());
    Eigen::Vector3d rotation_delta = axis_rotation.axis() * axis_rotation.angle();
    delta = Eigen::Matrix<double, 6, 1>::Zero();
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
  std::string axis_label[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d position_error = current_tip_pose_.position_ - desired_tip_pose_.position_;
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
  
  calculateTipForce();
  
  return ik_success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pose Leg::applyFK(const bool& set_current, const bool& use_actual)
{
  // Update joint transforms - skip first joint since it's transform is constant
  JointContainer::iterator joint_it;
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    std::shared_ptr<Joint> joint = joint_it->second;
    const std::shared_ptr<Link> reference_link = joint->reference_link_;
    double joint_angle = reference_link->actuating_joint_->desired_position_;
    if (use_actual)
    {
      joint_angle = reference_link->actuating_joint_->current_position_;
    }
    joint->current_transform_ = createDHMatrix(reference_link->dh_parameter_d_,
                                               reference_link->dh_parameter_theta_ + joint_angle,
                                               reference_link->dh_parameter_r_,
                                               reference_link->dh_parameter_alpha_);
  }
  const std::shared_ptr<Link> reference_link = tip_->reference_link_;
  double joint_angle = reference_link->actuating_joint_->desired_position_;
  if (use_actual)
  {
    joint_angle = reference_link->actuating_joint_->current_position_;
  }
  tip_->current_transform_ = createDHMatrix(reference_link->dh_parameter_d_,
                                            reference_link->dh_parameter_theta_ + joint_angle,
                                            reference_link->dh_parameter_r_,
                                            reference_link->dh_parameter_alpha_);

  // Get world frame position of tip
  Pose tip_pose = tip_->getPoseRobotFrame();
  if (set_current && !use_actual)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Link::Link(std::shared_ptr<Leg> leg, std::shared_ptr<Joint> actuating_joint,
           const int& id_number, const Parameters& params)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Link::Link(std::shared_ptr<Link> link)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(std::shared_ptr<Leg> leg, std::shared_ptr<Link> reference_link,
             const int& id_number, const Parameters& params)
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
  std::map<std::string, double> joint_parameters = params.joint_parameters[leg->getIDNumber()][id_number_ - 1].data;
  bool get_next_packed_position = true;
  std::string packed_position_key = "packed";
  int i = 0;
  while (get_next_packed_position)
  {
    try 
    {
      packed_positions_.push_back(joint_parameters.at(packed_position_key));
      get_next_packed_position = (packed_position_key != "packed");
    } 
    catch (std::out_of_range)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(std::shared_ptr<Joint> joint)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(void)
  : parent_leg_(NULL)
  , reference_link_(NULL)
  , id_number_(0)
  , id_name_("origin")
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Tip::Tip(std::shared_ptr<Leg> leg, std::shared_ptr<Link> reference_link)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Tip::Tip(std::shared_ptr<Tip> tip)
  : parent_leg_(tip->parent_leg_)
  , reference_link_(tip->reference_link_)
  , id_name_(tip->id_name_)
{
  identity_transform_ = tip->identity_transform_;
  current_transform_ = tip->current_transform_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
