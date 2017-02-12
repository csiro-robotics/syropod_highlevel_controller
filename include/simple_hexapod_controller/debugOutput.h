#ifndef SIMPLE_HEXAPOD_CONTROLLER_DEBUG_OUTPUT_H
#define SIMPLE_HEXAPOD_CONTROLLER_DEBUG_OUTPUT_H
/** 
 *  \file    debug_output.h
 *  \brief   Handles publishing of hexapod info for debugging in RVIZ. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   January 2017
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
 */
#include "standardIncludes.h"
#include <visualization_msgs/Marker.h>
#include "pose.h"
#include "model.h"
#include "walkController.h"

class DebugOutput
{
public:
  DebugOutput();
  void drawPoints(Model* model, bool debug_trajectory, double workspace_radius, double workspace_height, double max_stride_length);
  void drawRobot(Model* model);
  inline void reset()
  {
    robot_ID_ = 0;
    plot_ID_ = 0;
  }
  inline void updatePose(Vector2d linear_body_velocity, double angular_body_velocity, double height)
  {
    odometry_pose_.position_ += odometry_pose_.rotation_.rotateVector(Vector3d(linear_body_velocity[0], linear_body_velocity[1], 0));
    odometry_pose_.position_[2] = height;
    odometry_pose_.rotation_ *= Quat(Vector3d(0.0,0.0,angular_body_velocity));
  }

private:
  int robot_ID_;
  int plot_ID_;
  ros::NodeHandle n;
  ros::Publisher robot_publisher_;
  ros::Publisher visualization_publisher_;
  Pose odometry_pose_;
  vector<Vector3d> tip_position_history_;
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_DEBUG_OUTPUT_H */
