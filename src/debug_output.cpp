/*******************************************************************************************************************//**
 *  @file    debug_output.cpp
 *  @brief   Handles publishing of Syropod model info for debugging in RVIZ.
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

#include "syropod_highlevel_controller/debug_output.h"

/*******************************************************************************************************************//**
 * Constructor for debug output class. Sets up publishers for the visualisation markers and initialises odometry.
***********************************************************************************************************************/
DebugOutput::DebugOutput(void) 
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
void DebugOutput::updatePose(Vector2d linear_body_velocity, double angular_body_velocity, double height)
{
  Vector3d linear_body_velocity_3d = Vector3d(linear_body_velocity[0], linear_body_velocity[1], 0);
  Vector3d angular_body_velocity_3d = Vector3d(0.0, 0.0, angular_body_velocity);
  odometry_pose_ *= Pose(linear_body_velocity_3d, Quat(angular_body_velocity_3d));
  odometry_pose_.position_[2] = height;
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
 * linking the origin points of each joint and tip of each leg.
 * @param[in] model A pointer to the robot model object
***********************************************************************************************************************/
void DebugOutput::generateRobotModel(Model* model)
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

  Pose pose = odometry_pose_.addPose(model->getCurrentPose());

  geometry_msgs::Point point;
  Vector3d previous_body_position = pose.transformVector(Vector3d(0.0, 0.0, 0.0));
  Vector3d initial_body_position = pose.transformVector(Vector3d(0.0, 0.0, 0.0));

  map<int, Leg*>::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;

    // Generate line segment between 1st joint of each leg (creating body)
    point.x = previous_body_position[0];
    point.y = previous_body_position[1];
    point.z = previous_body_position[2];
    leg_line_list.points.push_back(point);

    Joint* first_joint = leg->getJointContainer()->begin()->second;
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
    map<int, Joint*>::iterator joint_it; //Start at second joint
    for (joint_it = ++leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      point.x = previous_joint_position[0];
      point.y = previous_joint_position[1];
      point.z = previous_joint_position[2];
      leg_line_list.points.push_back(point);
      
      Joint* joint = joint_it->second;
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

    Vector3d tip_point = pose.transformVector(leg->getCurrentTipPosition());
    point.x = tip_point[0];
    point.y = tip_point[1];
    point.z = tip_point[2];
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
void DebugOutput::generateTipTrajectory(Leg* leg, Pose current_pose)
{
  Pose pose = odometry_pose_.addPose(current_pose);

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
  tip_position_id_ = (tip_position_id_ + 1)%ID_LIMIT; // Ensures the trajectory marker id does not exceed overflow
}

/*******************************************************************************************************************//**
 * Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip 
 * trajectory of the input leg.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
***********************************************************************************************************************/
void DebugOutput::generateBezierCurves(Leg* leg)
{
  visualization_msgs::Marker swing_1_nodes;
  swing_1_nodes.header.frame_id = "/fixed_frame";
  swing_1_nodes.header.stamp = ros::Time::now();
  swing_1_nodes.ns = "primary_swing_control_nodes";
  swing_1_nodes.id = marker_id_++;
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
  swing_2_nodes.id = marker_id_++;
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
  stance_nodes.id = marker_id_++;
  stance_nodes.action = visualization_msgs::Marker::ADD;
  stance_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  stance_nodes.scale.x = 0.02;
  stance_nodes.color.r = 1; //YELLOW
  stance_nodes.color.g = 1;
  stance_nodes.color.a = 1;
  stance_nodes.lifetime = ros::Duration(time_delta_);
  
  LegStepper* leg_stepper = leg->getLegStepper();
  
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
 * Publishes visualisation markers which represent the walking workspace of the input leg as well as the current 
 * requested stride length.
 * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
 * @param[in] walker A pointer to the walk controller object
***********************************************************************************************************************/
void DebugOutput::generateWorkspace(Leg* leg, WalkController* walker)
{
  LegStepper* leg_stepper = leg->getLegStepper();
  double workspace_radius = walker->getWorkspaceRadius();
  double workspace_height = walker->getParameters()->step_clearance.current_value;
  double stride_length = walker->getStrideLength();
  
  visualization_msgs::Marker workspace;
  workspace.header.frame_id = "/fixed_frame";
  workspace.header.stamp = ros::Time::now();
  workspace.ns = "workspace_markers";
  workspace.id = marker_id_++;
  workspace.type = visualization_msgs::Marker::CYLINDER;
  workspace.action = visualization_msgs::Marker::ADD;
  workspace.pose.position.x = leg_stepper->getDefaultTipPosition()[0];
  workspace.pose.position.y = leg_stepper->getDefaultTipPosition()[1];
  workspace.pose.position.z = workspace_height / 2.0;
  workspace.scale.x = workspace_radius * 2.0;
  workspace.scale.y = workspace_radius * 2.0;
  workspace.scale.z = workspace_height;
  workspace.color.g = 1; //TRANSPARENT CYAN
  workspace.color.b = 1;
  workspace.color.a = 0.1;
  workspace.lifetime = ros::Duration(time_delta_);

  visualization_msgs::Marker stride;
  stride.header.frame_id = "/fixed_frame";
  stride.header.stamp = ros::Time::now();
  stride.ns = "workspace_markers";
  stride.id = marker_id_++;
  stride.type = visualization_msgs::Marker::CYLINDER;
  stride.action = visualization_msgs::Marker::ADD;
  stride.pose.position.x = leg_stepper->getDefaultTipPosition()[0];
  stride.pose.position.y = leg_stepper->getDefaultTipPosition()[1];
  stride.scale.x = stride_length;
  stride.scale.y = stride_length;
  stride.scale.z = 0.005;
  stride.color.g = 1; //TRANSPARENT GREEN
  stride.color.a = 0.25;
  stride.lifetime = ros::Duration(time_delta_);
  
  workspace_publisher_.publish(workspace);
  workspace_publisher_.publish(stride);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
