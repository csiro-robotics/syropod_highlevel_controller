/*******************************************************************************************************************//**
 *  @file    debug_visualiser.cpp
 *  @brief   Handles publishing of Syropod model info for debugging in RVIZ.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    April 2018
 *  @version 0.5.10
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

#include "syropod_highlevel_controller/debug_visualiser.h"
#include "syropod_highlevel_controller/walk_controller.h"

/*******************************************************************************************************************//**
 * Constructor for debug visualiser class. Sets up publishers for the visualisation markers and initialises odometry.
***********************************************************************************************************************/
DebugVisualiser::DebugVisualiser(void)
{
  ros::NodeHandle n;
  robot_model_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/robot_model", 1000);
  tip_trajectory_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/tip_trajectories", 1000);
  bezier_curve_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/bezier_curves", 1000);
  default_tip_position_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/default_tip_positions", 1000);
  target_tip_position_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/target_tip_positions", 1000);
  walkspace_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/walkspace", 1000);
  workspace_publisher_ = n.advertise<visualization_msgs::MarkerArray>("/shc/debug/workspace", 1000);
  walk_plane_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/walk_plane", 1000);
  stride_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/stride", 1000);
  tip_force_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/tip_force", 1000);
  joint_torque_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/joint_torque", 1000);
  tip_rotation_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/tip_rotation", 1000);
  gravity_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/gravity", 1000);
  terrain_publisher_ = n.advertise<visualization_msgs::Marker>("/shc/debug/terrain", 1000);
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
 * linking the origin points of each joint and tip of each leg.
 * @param[in] model A pointer to the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateRobotModel(shared_ptr<Model> model)
{
  // Estimate of robot body length used in scaling markers
  if (marker_scale_ == 0)
  {
    marker_scale_ = model->getLegByIDNumber(0)->getJointByIDNumber(1)->getPoseRobotFrame().position_.norm() * 2.0;
  }
  
  visualization_msgs::Marker leg_line_list;
  leg_line_list.header.frame_id = "/base_link";
  leg_line_list.header.stamp = ros::Time(0);
  leg_line_list.ns = "robot_model";
  leg_line_list.action = visualization_msgs::Marker::ADD;
  leg_line_list.id = 0;
  leg_line_list.type = visualization_msgs::Marker::LINE_LIST;
  leg_line_list.frame_locked = true;
  leg_line_list.scale.x = 0.01 * sqrt(marker_scale_);
  leg_line_list.color.r = 1; //WHITE
  leg_line_list.color.g = 1;
  leg_line_list.color.b = 1;
  leg_line_list.color.a = 1;
  leg_line_list.pose = Pose::Identity().toPoseMessage();

  geometry_msgs::Point point;
  Vector3d previous_body_position = Vector3d::Zero();
  Vector3d initial_body_position = Vector3d::Zero();

  LegContainer::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;

    // Generate line segment between 1st joint of each leg (creating body)
    point.x = previous_body_position[0];
    point.y = previous_body_position[1];
    point.z = previous_body_position[2];
    leg_line_list.points.push_back(point);

    shared_ptr<Joint> first_joint = leg->getJointContainer()->begin()->second;
    Vector3d first_joint_position = first_joint->getPoseRobotFrame().position_;
    point.x = first_joint_position[0];
    point.y = first_joint_position[1];
    point.z = first_joint_position[2];
    leg_line_list.points.push_back(point);

    Vector3d previous_joint_position = first_joint_position;
    previous_body_position = first_joint_position;

    if (leg->getIDNumber() == 0)
    {
      initial_body_position = first_joint_position;
    }

    // Generate line segment between joint positions
    JointContainer::iterator joint_it; //Start at second joint
    for (joint_it = ++leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      point.x = previous_joint_position[0];
      point.y = previous_joint_position[1];
      point.z = previous_joint_position[2];
      leg_line_list.points.push_back(point);

      shared_ptr<Joint> joint = joint_it->second;
      Vector3d joint_position = joint->getPoseRobotFrame().position_;
      point.x = joint_position[0];
      point.y = joint_position[1];
      point.z = joint_position[2];

      leg_line_list.points.push_back(point);
      previous_joint_position = joint_position;
    }

    // Generate line segment from last joint to tip position
    point.x = previous_joint_position[0];
    point.y = previous_joint_position[1];
    point.z = previous_joint_position[2];
    leg_line_list.points.push_back(point);

    Vector3d tip_position = leg->getCurrentTipPose().position_;
    point.x = tip_position[0];
    point.y = tip_position[1];
    point.z = tip_position[2];
    leg_line_list.points.push_back(point);
  }

  // Generate final line segment to close body
  point.x = previous_body_position[0];
  point.y = previous_body_position[1];
  point.z = previous_body_position[2];
  leg_line_list.points.push_back(point);

  point.x = initial_body_position[0];
  point.y = initial_body_position[1];
  point.z = initial_body_position[2];
  leg_line_list.points.push_back(point);

  robot_model_publisher_.publish(leg_line_list);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the estimated walking plane.
  * @param[in] walk_plane A Vector representing the walk plane
***********************************************************************************************************************/
void DebugVisualiser::generateWalkPlane(const Vector3d& walk_plane, const Vector3d& walk_plane_normal)
{
  visualization_msgs::Marker walk_plane_marker;
  walk_plane_marker.header.frame_id = "/walk_plane";
  walk_plane_marker.header.stamp = ros::Time::now();
  walk_plane_marker.ns = "walk_plane_marker";
  walk_plane_marker.id = 0;
  walk_plane_marker.type = visualization_msgs::Marker::CUBE;
  walk_plane_marker.action = visualization_msgs::Marker::ADD;
  walk_plane_marker.scale.x = 2.0 * sqrt(marker_scale_);
  walk_plane_marker.scale.y = 2.0 * sqrt(marker_scale_);
  walk_plane_marker.scale.z = 1e-3 * sqrt(marker_scale_);
  walk_plane_marker.color.g = 1;
  walk_plane_marker.color.b = 1;
  walk_plane_marker.color.a = 0.5;
  walk_plane_marker.pose = Pose::Identity().toPoseMessage();
  
  walk_plane_marker.pose.position.x = 0.0;
  walk_plane_marker.pose.position.y = 0.0;
  walk_plane_marker.pose.position.z = walk_plane[2];
  
  Quaterniond walk_plane_orientation = Quaterniond::FromTwoVectors(Vector3d::UnitZ(), walk_plane_normal);
  walk_plane_marker.pose.orientation.w = walk_plane_orientation.w();
  walk_plane_marker.pose.orientation.x = walk_plane_orientation.x();
  walk_plane_marker.pose.orientation.y = walk_plane_orientation.y();
  walk_plane_marker.pose.orientation.z = walk_plane_orientation.z();
  
  walk_plane_publisher_.publish(walk_plane_marker);
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the trajectory of the tip of the input leg.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateTipTrajectory(shared_ptr<Leg> leg)
{
  visualization_msgs::Marker tip_position_marker;
  tip_position_marker.header.frame_id = "/base_link";
  tip_position_marker.header.stamp = ros::Time::now();
  tip_position_marker.ns = "tip_trajectory_markers";
  tip_position_marker.id = tip_position_id_;
  tip_position_marker.action = visualization_msgs::Marker::ADD;
  tip_position_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  tip_position_marker.scale.x = 0.005 * sqrt(marker_scale_);
  tip_position_marker.color.r = 1; //RED
  tip_position_marker.color.a = 1;
  tip_position_marker.lifetime = ros::Duration(TRAJECTORY_DURATION);
  tip_position_marker.pose = Pose::Identity().toPoseMessage();

  Vector3d tip_position = leg->getCurrentTipPose().position_;
  geometry_msgs::Point point;
  point.x = tip_position[0];
  point.y = tip_position[1];
  point.z = tip_position[2];
  ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
  tip_position_marker.points.push_back(point);

  tip_trajectory_publisher_.publish(tip_position_marker);
  tip_position_id_ = (tip_position_id_ + 1) % ID_LIMIT; // Ensures the trajectory marker id does not exceed overflow
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent an estimate of the terrain being traversed
 * @param[in] model A pointer to the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateTerrainEstimate(shared_ptr<Model> model)
{
  LegContainer::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    if (leg->getLegStepper()->getSwingProgress() == 1.0)
    {
      Vector3d tip_position = leg->getCurrentTipPose().position_;
      
      visualization_msgs::Marker terrain_marker;
      terrain_marker.header.frame_id = "/base_link";
      terrain_marker.header.stamp = ros::Time::now();
      terrain_marker.ns = "terrain_markers";
      terrain_marker.id = terrain_marker_id_;
      terrain_marker.action = visualization_msgs::Marker::ADD;
      terrain_marker.type = visualization_msgs::Marker::CUBE_LIST;
      terrain_marker.scale.x = 0.25 * sqrt(marker_scale_);
      terrain_marker.scale.y = 0.25 * sqrt(marker_scale_);
      terrain_marker.scale.z = tip_position[2] + 0.5;
      terrain_marker.color.r = 1; //RED
      terrain_marker.color.a = 0.5;
      Pose pose(Vector3d(0, 0, -terrain_marker.scale.z / 2.0), model->getCurrentPose().rotation_.inverse());
      terrain_marker.pose = pose.toPoseMessage();
      
      geometry_msgs::Point point;
      point.x = tip_position[0];
      point.y = tip_position[1];
      point.z = tip_position[2];
      terrain_marker.points.push_back(point);
      terrain_publisher_.publish(terrain_marker);
      terrain_marker_id_ = (terrain_marker_id_ + 1) % (model->getLegCount() * 10);
    }
  }
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip
 * trajectory of the input leg.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateBezierCurves(shared_ptr<Leg> leg)
{
  visualization_msgs::Marker swing_1_nodes;
  swing_1_nodes.header.frame_id = "/walk_plane";
  swing_1_nodes.header.stamp = ros::Time::now();
  swing_1_nodes.ns = "primary_swing_control_nodes";
  swing_1_nodes.id = leg->getIDNumber();
  swing_1_nodes.action = visualization_msgs::Marker::ADD;
  swing_1_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  swing_1_nodes.scale.x = 0.02 * sqrt(marker_scale_);
  swing_1_nodes.color.g = 1; //YELLOW
  swing_1_nodes.color.r = 1;
  swing_1_nodes.color.a = 1;
  swing_1_nodes.pose = Pose::Identity().toPoseMessage();

  visualization_msgs::Marker swing_2_nodes;
  swing_2_nodes.header.frame_id = "/walk_plane";
  swing_2_nodes.header.stamp = ros::Time::now();
  swing_2_nodes.ns = "secondary_swing_control_nodes";
  swing_2_nodes.id = leg->getIDNumber();
  swing_2_nodes.action = visualization_msgs::Marker::ADD;
  swing_2_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  swing_2_nodes.scale.x = 0.02 * sqrt(marker_scale_);
  swing_2_nodes.scale.y = 0.02 * sqrt(marker_scale_);
  swing_2_nodes.scale.z = 0.02 * sqrt(marker_scale_);
  swing_2_nodes.color.r = 1; //YELLOW
  swing_2_nodes.color.g = 1;
  swing_2_nodes.color.a = 1;
  swing_2_nodes.pose = Pose::Identity().toPoseMessage();

  visualization_msgs::Marker stance_nodes;
  stance_nodes.header.frame_id = "/walk_plane";
  stance_nodes.header.stamp = ros::Time::now();
  stance_nodes.ns = "stance_control_nodes";
  stance_nodes.id = leg->getIDNumber();
  stance_nodes.action = visualization_msgs::Marker::ADD;
  stance_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  stance_nodes.scale.x = 0.02 * sqrt(marker_scale_);
  stance_nodes.color.r = 1; //YELLOW
  stance_nodes.color.g = 1;
  stance_nodes.color.a = 1;
  stance_nodes.pose = Pose::Identity().toPoseMessage();

  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

  for (int i = 0; i < 5; ++i) // For each of 5 control nodes
  {
    geometry_msgs::Point point;
    if (leg_stepper->getStepState() == STANCE)
    {
      Vector3d stance_node = leg_stepper->getStanceControlNode(i);
      point.x = stance_node[0];
      point.y = stance_node[1];
      point.z = stance_node[2];
      ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
      stance_nodes.points.push_back(point);
    }
    else if (leg_stepper->getStepState() == SWING)
    {
      Vector3d swing_1_node = leg_stepper->getSwing1ControlNode(i);
      point.x = swing_1_node[0];
      point.y = swing_1_node[1];
      point.z = swing_1_node[2];
      ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
      swing_1_nodes.points.push_back(point);
      Vector3d swing_2_node = leg_stepper->getSwing2ControlNode(i);
      point.x = swing_2_node[0];
      point.y = swing_2_node[1];
      point.z = swing_2_node[2];
      ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
      swing_2_nodes.points.push_back(point);
    }
  }

  bezier_curve_publisher_.publish(stance_nodes);
  bezier_curve_publisher_.publish(swing_1_nodes);
  bezier_curve_publisher_.publish(swing_2_nodes);
}

/*******************************************************************************************************************//**
  * Publises visualisation markers which represent the default tip position of the leg
  * @param[in] leg A pointer to a leg of the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateDefaultTipPositions(shared_ptr<Leg> leg)
{
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

  visualization_msgs::Marker default_tip_position;
  default_tip_position.header.frame_id = "/walk_plane";
  default_tip_position.header.stamp = ros::Time::now();
  default_tip_position.ns = "default_tip_position_markers";
  default_tip_position.id = leg->getIDNumber();
  default_tip_position.type = visualization_msgs::Marker::SPHERE;
  default_tip_position.action = visualization_msgs::Marker::ADD;
  default_tip_position.scale.x = 0.02 * sqrt(marker_scale_);
  default_tip_position.scale.y = 0.02 * sqrt(marker_scale_);
  default_tip_position.scale.z = 0.02 * sqrt(marker_scale_);
  default_tip_position.color.g = 1;
  default_tip_position.color.b = leg_stepper->isAtCorrectPhase() ? 0.0 : 1.0;
  default_tip_position.color.a = 1;
  
  default_tip_position.pose = Pose::Identity().toPoseMessage();
  default_tip_position.pose.position.x = leg_stepper->getDefaultTipPose().position_[0];
  default_tip_position.pose.position.y = leg_stepper->getDefaultTipPose().position_[1];
  default_tip_position.pose.position.z = leg_stepper->getDefaultTipPose().position_[2];
  
  default_tip_position_publisher_.publish(default_tip_position);
}

/*******************************************************************************************************************//**
  * Publises visualisation markers which represent the target tip position of the leg
  * @param[in] leg A pointer to a leg of the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateTargetTipPositions(shared_ptr<Leg> leg)
{
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

  visualization_msgs::Marker target_tip_position;
  target_tip_position.header.frame_id = "/walk_plane";
  target_tip_position.header.stamp = ros::Time::now();
  target_tip_position.ns = "target_tip_position_markers";
  target_tip_position.id = leg->getIDNumber();
  target_tip_position.type = visualization_msgs::Marker::SPHERE;
  target_tip_position.action = visualization_msgs::Marker::ADD;
  target_tip_position.scale.x = 0.02 * sqrt(marker_scale_);
  target_tip_position.scale.y = 0.02 * sqrt(marker_scale_);
  target_tip_position.scale.z = 0.02 * sqrt(marker_scale_);
  target_tip_position.color.r = 1;
  target_tip_position.color.a = 1;
  
  target_tip_position.pose = Pose::Identity().toPoseMessage();
  target_tip_position.pose.position.x = leg_stepper->getTargetTipPose().position_[0];
  target_tip_position.pose.position.y = leg_stepper->getTargetTipPose().position_[1];
  target_tip_position.pose.position.z = leg_stepper->getTargetTipPose().position_[2];
  
  target_tip_position_publisher_.publish(target_tip_position);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the 2D walkspace for each leg.
  * @param[in] leg A pointer to a leg of the robot model object
  * @param[in] walkspace  A map of walkspace radii for a range of bearings to be visualised
***********************************************************************************************************************/
void DebugVisualiser::generateWalkspace(shared_ptr<Leg> leg, const LimitMap& walkspace)
{
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
  
  visualization_msgs::Marker walkspace_marker;
  walkspace_marker.header.frame_id = "/walk_plane";
  walkspace_marker.header.stamp = ros::Time::now();
  walkspace_marker.ns = "walkspace_markers";
  walkspace_marker.id = leg->getIDNumber();
  walkspace_marker.type = visualization_msgs::Marker::LINE_STRIP;
  walkspace_marker.action = visualization_msgs::Marker::ADD;
  walkspace_marker.scale.x = 0.002 * sqrt(marker_scale_);
  walkspace_marker.color.g = 1;
  walkspace_marker.color.b = 1;
  walkspace_marker.color.a = 1;
  Pose pose(Vector3d::Zero(), Quaterniond::FromTwoVectors(Vector3d::UnitZ(), leg_stepper->getWalkPlaneNormal()));
  walkspace_marker.pose = pose.toPoseMessage();
  
  geometry_msgs::Point origin_point;
  Vector3d walkspace_origin = leg_stepper->getDefaultTipPose().position_;
  walkspace_origin = pose.inverseTransformVector(walkspace_origin);
  origin_point.x = walkspace_origin[0];
  origin_point.y = walkspace_origin[1];
  origin_point.z = walkspace_origin[2];
  
  geometry_msgs::Point first_point;
  LimitMap::const_iterator it;
  for (it = walkspace.begin(); it != walkspace.end(); ++it)
  {
    if (it->second != UNASSIGNED_VALUE)
    {
      geometry_msgs::Point point;
      point.x = origin_point.x + it->second * cos(degreesToRadians(it->first));
      point.y = origin_point.y + it->second * sin(degreesToRadians(it->first));
      point.z = origin_point.z;
      if (it->first == 0)
      {
        first_point = point;
      }
      walkspace_marker.points.push_back(point);
    }
  }
  walkspace_publisher_.publish(walkspace_marker);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the 3D workspace for each leg.
  * @param[in] leg A pointer to a leg of the robot model object
  * @param[in] body_clearance The vertical offset of the body above the walk plane
***********************************************************************************************************************/
void DebugVisualiser::generateWorkspace(shared_ptr<Leg> leg, const double& body_clearance)
{
  Workspace workspace = leg->getWorkspace();
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
  
  visualization_msgs::MarkerArray workspace_cage_marker_array;
  map<int, visualization_msgs::Marker> workspace_cage_markers;
  
  visualization_msgs::MarkerArray workspace_marker_array;
  visualization_msgs::Marker workspace_marker;
  workspace_marker.header.frame_id = "/base_link";
  workspace_marker.header.stamp = ros::Time(0);
  workspace_marker.ns = leg->getIDName() + "_workspace_markers";
  workspace_marker.type = visualization_msgs::Marker::LINE_STRIP;
  workspace_marker.action = visualization_msgs::Marker::ADD;
  workspace_marker.scale.x = 0.002 * sqrt(marker_scale_);
  workspace_marker.color.b = 1;
  workspace_marker.color.a = 1;
  workspace_marker.pose = Pose::Identity().toPoseMessage();

  Workspace::const_iterator workspace_it;
  int workspace_id = 1;
  for (workspace_it = workspace.begin(); workspace_it != workspace.end(); ++workspace_it, ++workspace_id)
  {
    double plane_height = workspace_it->first;
    Workplane workplane = workspace_it->second;
    workspace_marker.id = workspace_id;
    geometry_msgs::Point origin_point;
    Vector3d identity_tip_position = leg_stepper->getIdentityTipPose().position_ - Vector3d::UnitZ() * body_clearance;
    Vector3d workplane_origin = identity_tip_position + Vector3d::UnitZ() * plane_height;
    //workspace_origin = pose.inverseTransformVector(workspace_origin);
    origin_point.x = workplane_origin[0];
    origin_point.y = workplane_origin[1];
    origin_point.z = workplane_origin[2];
    
    geometry_msgs::Point first_point;
    Workplane::const_iterator workplane_it;
    for (workplane_it = workplane.begin(); workplane_it != workplane.end(); ++workplane_it)
    {
      int bearing = workplane_it->first;
      double radius = workplane_it->second;
      if (radius != UNASSIGNED_VALUE)
      {
        geometry_msgs::Point point;
        point.x = origin_point.x + radius * cos(degreesToRadians(bearing));
        point.y = origin_point.y + radius * sin(degreesToRadians(bearing));
        point.z = origin_point.z;
        if (bearing == 0)
        {
          first_point = point;
        }
        
        workspace_marker.points.push_back(point);
        
        // Generate workspace cage markers (vertical connection between same bearings of different workspaces)
        if (workspace_cage_markers.find(bearing) == workspace_cage_markers.end())
        {
          visualization_msgs::Marker workspace_cage_marker = workspace_marker;
          workspace_cage_marker.ns = leg->getIDName() + "_workspace_markers";
          workspace_cage_marker.id = bearing;
          workspace_cage_markers.insert(map<int, visualization_msgs::Marker>::value_type(bearing, workspace_cage_marker));
        }
        workspace_cage_markers[bearing].points.push_back(point);
      }
    }
    
    workspace_marker.points.push_back(first_point);
    workspace_marker_array.markers.push_back(workspace_marker);
    workspace_marker.points.clear();
    
    // Publish workspace cage markers
    map<int, visualization_msgs::Marker>::iterator cage_it;
    for (cage_it = workspace_cage_markers.begin(); cage_it != workspace_cage_markers.end(); ++cage_it)
    {
      workspace_cage_marker_array.markers.push_back(cage_it->second);
    }
  }
  
  workspace_publisher_.publish(workspace_marker_array);
  workspace_publisher_.publish(workspace_cage_marker_array);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent requested stride vector for each leg.
  * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateStride(shared_ptr<Leg> leg)
{
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
  Vector3d stride_vector = leg_stepper->getStrideVector();

  visualization_msgs::Marker stride;
  stride.header.frame_id = "/walk_plane";
  stride.header.stamp = ros::Time::now();
  stride.ns = "stride_markers";
  stride.id = leg->getIDNumber();
  stride.type = visualization_msgs::Marker::ARROW;
  stride.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point origin;
  geometry_msgs::Point target;
  origin.x = leg_stepper->getDefaultTipPose().position_[0];
  origin.y = leg_stepper->getDefaultTipPose().position_[1];
  origin.z = leg_stepper->getDefaultTipPose().position_[2];
  target = origin;
  target.x += (stride_vector[0] / 2.0);
  target.y += (stride_vector[1] / 2.0);
  target.z += (stride_vector[2] / 2.0);
  stride.points.push_back(origin);
  stride.points.push_back(target);
  stride.scale.x = 0.01 * sqrt(marker_scale_);
  stride.scale.y = 0.015 * sqrt(marker_scale_);
  stride.scale.z = 0.02 * sqrt(marker_scale_);
  stride.color.g = 1; //GREEN
  stride.color.a = 1;
  stride.pose = Pose::Identity().toPoseMessage();

  stride_publisher_.publish(stride);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the calculated & measured estimated tip force vector for input leg.
  * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateTipForce(shared_ptr<Leg> leg)
{
  visualization_msgs::Marker tip_force;
  tip_force.header.frame_id = "/base_link";
  tip_force.header.stamp = ros::Time::now();
  tip_force.id = leg->getIDNumber();
  tip_force.type = visualization_msgs::Marker::ARROW;
  tip_force.action = visualization_msgs::Marker::ADD;
  Vector3d tip_position = leg->getCurrentTipPose().position_;
  geometry_msgs::Point origin;
  geometry_msgs::Point target;
  origin.x = tip_position[0];
  origin.y = tip_position[1];
  origin.z = tip_position[2];
  tip_force.scale.x = 0.01 * sqrt(marker_scale_);
  tip_force.scale.y = 0.015 * sqrt(marker_scale_);
  tip_force.scale.z = 0.02 * sqrt(marker_scale_);
  tip_force.color.a = 1;
  tip_force.pose = Pose::Identity().toPoseMessage();
  
  // Tip Force Calculated
  visualization_msgs::Marker tip_force_calculated = tip_force;
  tip_force_calculated.ns = "tip_force_calculated_markers";
  target = origin;
  target.x += leg->getTipForceCalculated()[0];
  target.y += leg->getTipForceCalculated()[1];
  target.z += leg->getTipForceCalculated()[2];
  tip_force_calculated.points.push_back(origin);
  tip_force_calculated.points.push_back(target);
  tip_force_calculated.color.b = 1; // MAGENTA
  tip_force_calculated.color.r = 1;
  tip_force_publisher_.publish(tip_force_calculated);
  
  // Tip Force Measured
  visualization_msgs::Marker tip_force_measured = tip_force;
  tip_force_measured.ns = "tip_force_measured_markers";
  target = origin;
  target.x += leg->getTipForceMeasured()[0];
  target.y += leg->getTipForceMeasured()[1];
  target.z += leg->getTipForceMeasured()[2];
  tip_force_measured.points.push_back(origin);
  tip_force_measured.points.push_back(target);
  tip_force_measured.color.b = 1; // CYAN
  tip_force_measured.color.g = 1;
  tip_force_publisher_.publish(tip_force_measured);
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the estimated percentage of max torque in each joint.
  * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateJointTorques(shared_ptr<Leg> leg)
{
  JointContainer::iterator joint_it;
  int marker_id = leg->getIDNumber() * leg->getJointCount();
  for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
  {
    shared_ptr<Joint> joint = joint_it->second;
    visualization_msgs::Marker joint_torque;
    joint_torque.header.frame_id = "/base_link";
    joint_torque.header.stamp = ros::Time::now();
    joint_torque.ns = "joint_torque_markers";
    joint_torque.type = visualization_msgs::Marker::SPHERE;
    joint_torque.action = visualization_msgs::Marker::ADD;
    Vector3d joint_position = joint->getPoseRobotFrame().position_;
    joint_torque.pose = Pose(joint_position, Quaterniond::Identity()).toPoseMessage();
    double torque_maximum_ratio = clamped(abs(joint->current_effort_) * 5.0, 0.1, 1.0);
    double sphere_radius = 0.1 * sqrt(marker_scale_) * torque_maximum_ratio;
    
    // Actual value
    joint_torque.id = marker_id++;
    joint_torque.scale.x = sphere_radius;
    joint_torque.scale.y = sphere_radius;
    joint_torque.scale.z = sphere_radius;
    joint_torque.color.r = torque_maximum_ratio;
    joint_torque.color.g = 1.0 - torque_maximum_ratio;
    joint_torque.color.b = 0.0;
    joint_torque.color.a = 1.0;
    joint_torque_publisher_.publish(joint_torque);
  }
}

/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent the estimate of the gravitational acceleration vector.
  * @param[in] gravity_estimate An estimate of the gravitational acceleration vector
***********************************************************************************************************************/
void DebugVisualiser::generateGravity(const Vector3d& gravity_estimate)
{
  visualization_msgs::Marker gravity;
  gravity.header.frame_id = "/base_link";
  gravity.header.stamp = ros::Time::now();
  gravity.ns = "gravity_marker";
  gravity.id = 0;
  gravity.type = visualization_msgs::Marker::ARROW;
  gravity.action = visualization_msgs::Marker::ADD;
  
  Vector3d robot_position = Vector3d::Zero();
  geometry_msgs::Point origin;
  origin.x = robot_position[0];
  origin.y = robot_position[1];
  origin.z = robot_position[2];
  
  Vector3d direction_vector = gravity_estimate.normalized() * 0.1 * sqrt(marker_scale_); //Arrow Length  
  geometry_msgs::Point target;
  target = origin;
  target.x += direction_vector[0];
  target.y += direction_vector[1];
  target.z += direction_vector[2];
  gravity.points.push_back(origin);
  gravity.points.push_back(target);
  gravity.scale.x = 0.01 * sqrt(marker_scale_);
  gravity.scale.y = 0.015 * sqrt(marker_scale_);
  gravity.scale.z = 0.02 * sqrt(marker_scale_);
  gravity.color.r = 1; // YELLOW
  gravity.color.g = 1;
  gravity.color.a = 1;
  gravity.pose = Pose::Identity().toPoseMessage();

  gravity_publisher_.publish(gravity);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
