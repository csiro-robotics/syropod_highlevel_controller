/*******************************************************************************************************************//**
 *  @file    debug_visualiser.cpp
 *  @brief   Handles publishing of Syropod model info for debugging in RVIZ.
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

#include "syropod_highlevel_controller/debug_visualiser.h"

/*******************************************************************************************************************//**
 * Constructor for debug visualiser class. Sets up publishers for the visualisation markers and initialises odometry.
***********************************************************************************************************************/
DebugVisualiser::DebugVisualiser(void)
{
  string node_name = ros::this_node::getName();
  robot_model_publisher_ = n_.advertise<visualization_msgs::Marker>(node_name + "/debug/robot_model", 1000);
  tip_trajectory_publisher_ = n_.advertise<visualization_msgs::Marker>(node_name + "/debug/tip_trajectories", 1000);
  bezier_curve_publisher_ = n_.advertise<visualization_msgs::Marker>(node_name + "/debug/bezier_curves", 1000);
  workspace_publisher_ = n_.advertise<visualization_msgs::Marker>(node_name + "/debug/workspaces", 1000);
  odometry_pose_ = Pose::identity();
}

/*******************************************************************************************************************//**
 * Updates the odometry pose of the robot body from velocity inputs
 * @param[in] linear_body_velocity The linear velocity of the robot body in the x/y plane
 * @param[in] angular_body_velocity The angular velocity of the robot body
 * @param[in] height The desired height of the robot body above ground
***********************************************************************************************************************/
void DebugVisualiser::updatePose(const Vector2d& linear_body_velocity,
                                 const double& angular_body_velocity, const double& height)
{
  Vector3d linear_body_velocity_3d = Vector3d(linear_body_velocity[0], linear_body_velocity[1], 0);
  Vector3d angular_body_velocity_3d = Vector3d(0.0, 0.0, angular_body_velocity);
  odometry_pose_.position_ += odometry_pose_.rotation_.rotateVector(linear_body_velocity_3d);
  odometry_pose_.rotation_ *= Quat(angular_body_velocity_3d);
  odometry_pose_.position_[2] = height;
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
 * linking the origin points of each joint and tip of each leg.
 * @param[in] model A pointer to the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateRobotModel(shared_ptr<Model> model)
{
  visualization_msgs::Marker leg_line_list;
  leg_line_list.header.frame_id = "/fixed_frame";
  leg_line_list.header.stamp = ros::Time::now();
  leg_line_list.ns = "robot_model";
  leg_line_list.action = visualization_msgs::Marker::ADD;
  leg_line_list.id = ROBOT_MODEL_ID;
  leg_line_list.type = visualization_msgs::Marker::LINE_LIST;
  leg_line_list.scale.x = 0.005;
  leg_line_list.color.r = 1; //WHITE
  leg_line_list.color.g = 1;
  leg_line_list.color.b = 1;
  leg_line_list.color.a = 1;

  Pose pose = odometry_pose_;
  Pose current_pose = model->getCurrentPose();
  pose.position_ += pose.rotation_.rotateVector(current_pose.position_);
  pose.rotation_ *= current_pose.rotation_;

  geometry_msgs::Point point;
  Vector3d previous_body_position = pose.transformVector(Vector3d(0.0, 0.0, 0.0));
  Vector3d initial_body_position = pose.transformVector(Vector3d(0.0, 0.0, 0.0));

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
    Vector3d first_joint_position = pose.transformVector(first_joint->getPositionRobotFrame());
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
      Vector3d joint_position = pose.transformVector(joint->getPositionRobotFrame());
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

    Vector3d tip_position = pose.transformVector(leg->getCurrentTipPosition());
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
 * Publishes visualisation markers which represent the trajectory of the tip of the input leg.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
 * @param[in] current_pose The current pose of the body in the robot model - modifies the tip trajectory
***********************************************************************************************************************/
void DebugVisualiser::generateTipTrajectory(shared_ptr<Leg> leg, const Pose& current_pose)
{
  Pose pose = odometry_pose_;
  pose.position_ += pose.rotation_.rotateVector(current_pose.position_);
  pose.rotation_ *= current_pose.rotation_;

  visualization_msgs::Marker tip_position_marker;
  tip_position_marker.header.frame_id = "/fixed_frame";
  tip_position_marker.header.stamp = ros::Time::now();
  tip_position_marker.ns = "tip_trajectory_markers";
  tip_position_marker.id = tip_position_id_;
  tip_position_marker.action = visualization_msgs::Marker::ADD;
  tip_position_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  tip_position_marker.scale.x = 0.005;
  tip_position_marker.color.r = 1; //RED
  tip_position_marker.color.a = 1;
  tip_position_marker.lifetime = ros::Duration(TRAJECTORY_DURATION);

  Vector3d tip_position = pose.transformVector(leg->getCurrentTipPosition());
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
 * Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip
 * trajectory of the input leg.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateBezierCurves(shared_ptr<Leg> leg)
{
  visualization_msgs::Marker swing_1_nodes;
  swing_1_nodes.header.frame_id = "/fixed_frame";
  swing_1_nodes.header.stamp = ros::Time::now();
  swing_1_nodes.ns = "primary_swing_control_nodes";
  swing_1_nodes.id = SWING_BEZIER_CURVE_1_MARKER_ID + leg->getIDNumber();
  swing_1_nodes.action = visualization_msgs::Marker::ADD;
  swing_1_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  swing_1_nodes.scale.x = 0.02;
  swing_1_nodes.color.g = 1; //YELLOW
  swing_1_nodes.color.r = 1;
  swing_1_nodes.color.a = 1;
  swing_1_nodes.lifetime = ros::Duration(time_delta_);

  visualization_msgs::Marker swing_2_nodes;
  swing_2_nodes.header.frame_id = "/fixed_frame";
  swing_2_nodes.header.stamp = ros::Time::now();
  swing_2_nodes.ns = "secondary_swing_control_nodes";
  swing_2_nodes.id = SWING_BEZIER_CURVE_2_MARKER_ID + leg->getIDNumber();
  swing_2_nodes.action = visualization_msgs::Marker::ADD;
  swing_2_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  swing_2_nodes.scale.x = 0.02;
  swing_2_nodes.scale.y = 0.02;
  swing_2_nodes.scale.z = 0.02;
  swing_2_nodes.color.r = 1; //YELLOW
  swing_2_nodes.color.g = 1;
  swing_2_nodes.color.a = 1;
  swing_2_nodes.lifetime = ros::Duration(time_delta_);

  visualization_msgs::Marker stance_nodes;
  stance_nodes.header.frame_id = "/fixed_frame";
  stance_nodes.header.stamp = ros::Time::now();
  stance_nodes.ns = "stance_control_nodes";
  stance_nodes.id = STANCE_BEZIER_CURVE_MARKER_ID + leg->getIDNumber();
  stance_nodes.action = visualization_msgs::Marker::ADD;
  stance_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  stance_nodes.scale.x = 0.02;
  stance_nodes.color.r = 1; //YELLOW
  stance_nodes.color.g = 1;
  stance_nodes.color.a = 1;
  stance_nodes.lifetime = ros::Duration(time_delta_);

  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

  for (int i = 0; i < 5; ++i) // For each of 5 control nodes
  {
    geometry_msgs::Point point;
    if (leg_stepper->getStepState() == STANCE)
    {
      Vector3d stance_node = odometry_pose_.transformVector(leg_stepper->getStanceControlNode(i));
      point.x = stance_node[0];
      point.y = stance_node[1];
      point.z = stance_node[2];
      ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
      stance_nodes.points.push_back(point);
    }
    else if (leg_stepper->getStepState() == SWING)
    {
      Vector3d swing_1_node = odometry_pose_.transformVector(leg_stepper->getSwing1ControlNode(i));
      point.x = swing_1_node[0];
      point.y = swing_1_node[1];
      point.z = swing_1_node[2];
      ROS_ASSERT(point.x + point.y + point.z < 1e3); //Check that point has valid values
      swing_1_nodes.points.push_back(point);
      Vector3d swing_2_node = odometry_pose_.transformVector(leg_stepper->getSwing2ControlNode(i));
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
  * Publises visualisation markers which represent the workspace for each leg.
  * @param[in] workspace_map A map of worksapce radii for a range of bearings
  * @param[in] model A pointer to the robot model object
***********************************************************************************************************************/
void DebugVisualiser::generateWorkspace(map<int, double> workspace_map, shared_ptr<Model> model)
{
  LegContainer::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    visualization_msgs::Marker workspace;
    workspace.header.frame_id = "/fixed_frame";
    workspace.header.stamp = ros::Time::now();
    workspace.ns = "workspace_markers";
    workspace.id = WORKSPACE_ID + leg->getIDNumber();
    workspace.type = visualization_msgs::Marker::LINE_STRIP;
    workspace.action = visualization_msgs::Marker::ADD;
    workspace.scale.x = 0.002;
    workspace.color.g = 1;
    workspace.color.b = 1;
    workspace.color.a = 1;

    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    geometry_msgs::Point origin_point;
    origin_point.x = leg_stepper->getDefaultTipPosition()[0];
    origin_point.y = leg_stepper->getDefaultTipPosition()[1];
    origin_point.z = 0.0;
    map<int, double>::iterator it;
    geometry_msgs::Point first_point;
    for (it = workspace_map.begin(); it != workspace_map.end(); ++it)
    {
      if (it->second != UNASSIGNED_VALUE)
      {
        geometry_msgs::Point point;
        point.x = origin_point.x + it->second * cos(degreesToRadians(it->first));
        point.y = origin_point.y + it->second * sin(degreesToRadians(it->first));
        point.z = 0.0;
        if (it == workspace_map.begin())
        {
          first_point = point;
        }
        workspace.points.push_back(point);
      }
    }
    workspace.points.push_back(first_point);
    workspace_publisher_.publish(workspace);
  }
}


/*******************************************************************************************************************//**
  * Publishes visualisation markers which represent requested stride vector for each leg.
  * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugVisualiser::generateStride(shared_ptr<Leg> leg)
{
  shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
  Vector2d stride_vector = leg_stepper->getStrideVector();

  visualization_msgs::Marker stride;
  stride.header.frame_id = "/fixed_frame";
  stride.header.stamp = ros::Time::now();
  stride.ns = "workspace_markers";
  stride.id = STRIDE_MARKER_ID + leg->getIDNumber();
  stride.type = visualization_msgs::Marker::ARROW;
  stride.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point origin;
  geometry_msgs::Point target;
  origin.x = leg_stepper->getDefaultTipPosition()[0];
  origin.y = leg_stepper->getDefaultTipPosition()[1];
  target = origin;
  target.x += stride_vector[0] / 2.0;
  target.y += stride_vector[1] / 2.0;
  stride.points.push_back(origin);
  stride.points.push_back(target);
  stride.scale.x = 0.01;
  stride.scale.y = 0.015;
  stride.scale.z = 0.02;
  stride.color.g = 1; //TRANSPARENT GREEN
  stride.color.a = 1;
  stride.lifetime = ros::Duration(time_delta_);

  workspace_publisher_.publish(stride);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
