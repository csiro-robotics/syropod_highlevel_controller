/*******************************************************************************************************************//**
 *  \file    debug_output.cpp
 *  \brief   Handles publishing of hexapod info for debugging in RVIZ. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   March 2017
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

#include "simple_hexapod_controller/debugOutput.h"

/*******************************************************************************************************************//**
 * Draw visualisation markers for robot body, legs and walking debug data
***********************************************************************************************************************/
DebugOutput::DebugOutput()
{
	string node_name = ros::this_node::getName();
	robot_publisher_ = n.advertise<visualization_msgs::Marker>(node_name + "/robot_visualisation", 1000);
	visualization_publisher_ = n.advertise<visualization_msgs::Marker>(node_name + "/walking_debug_visualisation", 1000);
	odometry_pose_ = Pose::identity();
	reset();
}

/*******************************************************************************************************************//**
 * Update pose tracking odometry from velocity inputs
***********************************************************************************************************************/
void DebugOutput::updatePose(Vector2d linear_body_velocity, double angular_body_velocity, double height)
{
	Vector3d linear_body_velocity_3d = Vector3d(linear_body_velocity[0], linear_body_velocity[1], 0);
	Vector3d angular_body_velocity_3d = Vector3d(0.0,0.0,angular_body_velocity);
	odometry_pose_.position_ += odometry_pose_.rotation_.rotateVector(linear_body_velocity_3d);	
	odometry_pose_.rotation_ *= Quat(angular_body_velocity_3d);
	odometry_pose_.position_[2] = height;
}

/*******************************************************************************************************************//**
 * Draw visualisation markers for robot body and legs
***********************************************************************************************************************/
void DebugOutput::drawRobot(Model* model)
{
	visualization_msgs::Marker leg_line_list;
	leg_line_list.header.frame_id = "/my_frame";
	leg_line_list.header.stamp = ros::Time::now();
	leg_line_list.ns = "trajectory_polyline_" + to_string(robot_ID_);
	leg_line_list.action = visualization_msgs::Marker::ADD;
	leg_line_list.id = robot_ID_++;
	leg_line_list.type = visualization_msgs::Marker::LINE_LIST;
	leg_line_list.scale.x = 0.005;
	leg_line_list.color.r = 1; //WHITE
	leg_line_list.color.g = 1;
	leg_line_list.color.b = 1;
	leg_line_list.color.a = 1;
	
	Pose pose;
	pose.position_ = odometry_pose_.position_ + model->getCurrentPose().position_;
	pose.rotation_ = odometry_pose_.rotation_ * model->getCurrentPose().rotation_;

	geometry_msgs::Point point;
	Vector3d previous_body_position = pose.transformVector(Vector3d(0.0,0.0,0.0));
	Vector3d initial_body_position = pose.transformVector(Vector3d(0.0,0.0,0.0));
	
	map<int, Leg*>::iterator leg_it;
	for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
	{
		Leg* leg = leg_it->second;
		
		//Draw body
		Joint* first_joint = leg->getJointContainer()->begin()->second;
		Vector3d first_joint_position = pose.transformVector(first_joint->getPositionWorldFrame());
		point.x = previous_body_position[0];
		point.y = previous_body_position[1];
		point.z = previous_body_position[2];
		leg_line_list.points.push_back(point);
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
	
		//Draw legs
		map<int, Joint*>::iterator joint_it; //Start at second joint
		for (joint_it = ++leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
		{
			Joint* joint = joint_it->second;
			point.x = previous_joint_position[0];
			point.y = previous_joint_position[1];
			point.z = previous_joint_position[2];
			leg_line_list.points.push_back(point);
			Vector3d joint_position = pose.transformVector(joint->getPositionWorldFrame());
			point.x = joint_position[0];
			point.y = joint_position[1];
			point.z = joint_position[2];
			leg_line_list.points.push_back(point);
			previous_joint_position = joint_position;
		}
		point.x = previous_joint_position[0];
		point.y = previous_joint_position[1];
		point.z = previous_joint_position[2];
		leg_line_list.points.push_back(point);
		Vector3d tip_point = pose.transformVector(leg->getLocalTipPosition());
		point.x = tip_point[0];
		point.y = tip_point[1];
		point.z = tip_point[2];
		leg_line_list.points.push_back(point);
	}
	point.x = previous_body_position[0];
	point.y = previous_body_position[1];
	point.z = previous_body_position[2];
	leg_line_list.points.push_back(point);
	point.x = initial_body_position[0];
	point.y = initial_body_position[1];
	point.z = initial_body_position[2];
	leg_line_list.points.push_back(point); 
	robot_publisher_.publish(leg_line_list);
}

/*******************************************************************************************************************//**
 * Draw visualisation markers for tip position history and walking debug data
***********************************************************************************************************************/
void DebugOutput::drawPoints(Model* model, WalkController* walker, bool debug_trajectory)
{
	double workspace_radius = walker->getWorkspaceRadius();	
	double workspace_height = walker->getParameters()->step_clearance.current_value;
	double stride_length = walker->getStrideLength();
	double time_delta = walker->getParameters()->time_delta.data;
	int num_legs = model->getLegCount();
	
	Pose pose;
	pose.position_ = odometry_pose_.position_ + model->getCurrentPose().position_;
	pose.rotation_ = odometry_pose_.rotation_ * model->getCurrentPose().rotation_;

	visualization_msgs::Marker tip_position_history;
	tip_position_history.header.frame_id = "/my_frame";
	tip_position_history.header.stamp = ros::Time::now();
	tip_position_history.ns = "tip_tracking_markers";
	tip_position_history.id = 0;
	tip_position_history.action = visualization_msgs::Marker::ADD;
	tip_position_history.type = visualization_msgs::Marker::SPHERE_LIST;
	tip_position_history.scale.x = 0.005;
	tip_position_history.color.r = 1; //RED
	tip_position_history.color.a = 1;
	tip_position_history.lifetime = ros::Duration();

	visualization_msgs::Marker swing_1_nodes;
	swing_1_nodes.header.frame_id = "/my_frame";
	swing_1_nodes.header.stamp = ros::Time::now();
	swing_1_nodes.ns = "primary_swing_control_nodes";
	swing_1_nodes.id = 1;
	swing_1_nodes.action = visualization_msgs::Marker::ADD;
	swing_1_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
	swing_1_nodes.scale.x = 0.02;
	swing_1_nodes.color.g = 1; //YELLOW
	swing_1_nodes.color.r = 1;
	swing_1_nodes.color.a = 1;  
	swing_1_nodes.lifetime = ros::Duration(time_delta);

	visualization_msgs::Marker swing_2_nodes;
	swing_2_nodes.header.frame_id = "/my_frame";
	swing_2_nodes.header.stamp = ros::Time::now();
	swing_2_nodes.ns = "secondary_swing_control_nodes";
	swing_2_nodes.id = 2;
	swing_2_nodes.action = visualization_msgs::Marker::ADD;
	swing_2_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
	swing_2_nodes.scale.x = 0.02;
	swing_2_nodes.scale.y = 0.02;
	swing_2_nodes.scale.z = 0.02;
	swing_2_nodes.color.r = 1; //YELLOW
	swing_2_nodes.color.g = 1;
	swing_2_nodes.color.a = 1;
	swing_2_nodes.lifetime = ros::Duration(time_delta);

	visualization_msgs::Marker stance_nodes;
	stance_nodes.header.frame_id = "/my_frame";
	stance_nodes.header.stamp = ros::Time::now();
	stance_nodes.ns = "stance_control_nodes";
	stance_nodes.id = 3;
	stance_nodes.action = visualization_msgs::Marker::ADD;
	stance_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
	stance_nodes.scale.x = 0.02;
	stance_nodes.color.r = 1; //YELLOW
	stance_nodes.color.g = 1;
	stance_nodes.color.a = 1;
	stance_nodes.lifetime = ros::Duration(time_delta);

	std::map<int, Leg*>::iterator leg_it;
	for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
	{
		Leg* leg = leg_it->second;
		LegStepper* leg_stepper = leg->getLegStepper();

		if (debug_trajectory && walker->getWalkState() != STOPPED)
		{
			visualization_msgs::Marker workspace;
			workspace.header.frame_id = "/my_frame";
			workspace.header.stamp = ros::Time::now();
			workspace.ns = "workspace_markers";
			workspace.id = leg->getIDNumber()+4; // Id after initial 4 markers
			workspace.type = visualization_msgs::Marker::CYLINDER;
			workspace.action = visualization_msgs::Marker::ADD;
			workspace.pose.position.x = pose.transformVector(leg_stepper->getDefaultTipPosition())[0];
			workspace.pose.position.y = pose.transformVector(leg_stepper->getDefaultTipPosition())[1];
			workspace.pose.position.z = workspace_height/2.0;
			workspace.scale.x = workspace_radius*2.0;
			workspace.scale.y = workspace_radius*2.0;
			workspace.scale.z = workspace_height;
			workspace.color.g = 1; //TRANSPARENT CYAN
			workspace.color.b = 1;
			workspace.color.a = 0.1;
			workspace.lifetime = ros::Duration(time_delta);
			visualization_publisher_.publish(workspace);

			visualization_msgs::Marker stride;
			stride.header.frame_id = "/my_frame";
			stride.header.stamp = ros::Time::now();
			stride.ns = "max_stride_markers";
			stride.id = leg->getIDNumber()+num_legs+4; // Id after intial 4 markers and workspace marker for each leg
			stride.type = visualization_msgs::Marker::CYLINDER;
			stride.action = visualization_msgs::Marker::ADD;
			stride.pose.position.x = pose.transformVector(leg_stepper->getDefaultTipPosition())[0];
			stride.pose.position.y = pose.transformVector(leg_stepper->getDefaultTipPosition())[1];
			stride.scale.x = stride_length;
			stride.scale.y = stride_length;
			stride.scale.z = 0.005;
			stride.color.g = 1; //TRANSPARENT GREEN
			stride.color.a = 0.25;    
			stride.lifetime = ros::Duration(time_delta);
			visualization_publisher_.publish(stride);  

			for (int i=0; i < 5; ++i) // For each of 5 control nodes
			{
				geometry_msgs::Point point;
				if (leg_stepper->getStepState() == STANCE)
				{
					Vector3d stance_node = odometry_pose_.transformVector(leg_stepper->getStanceControlNode(i));
					point.x = stance_node[0];
					point.y = stance_node[1];
					point.z = stance_node[2];
					stance_nodes.points.push_back(point);
				}
				else if (leg_stepper->getStepState() == SWING)
				{
					Vector3d swing_1_node = odometry_pose_.transformVector(leg_stepper->getSwing1ControlNode(i));
					point.x = swing_1_node[0];
					point.y = swing_1_node[1];
					point.z = swing_1_node[2];
					swing_1_nodes.points.push_back(point);
					Vector3d swing_2_node = odometry_pose_.transformVector(leg_stepper->getSwing2ControlNode(i));
					point.x = swing_2_node[0];
					point.y = swing_2_node[1];
					point.z = swing_2_node[2];
					swing_2_nodes.points.push_back(point);
				}
			}
		}

		Vector3d tip_position = pose.transformVector(leg->getLocalTipPosition());
		tip_position_history_.insert(tip_position_history_.begin(), tip_position);
		if (tip_position_history_.size() > 4000) // Tip history limit
		{
			tip_position_history_.pop_back();
		}
	}

	vector<Vector3d>::iterator it;
	for (it = tip_position_history_.begin(); it != tip_position_history_.end(); ++it)
	{
		geometry_msgs::Point point;
		Vector3d tip_position = *it;
		point.x = tip_position[0];
		point.y = tip_position[1];
		point.z = tip_position[2];
		tip_position_history.points.push_back(point);
	}
	visualization_publisher_.publish(tip_position_history);
	visualization_publisher_.publish(swing_1_nodes);
	visualization_publisher_.publish(swing_2_nodes);
	visualization_publisher_.publish(stance_nodes);
}
