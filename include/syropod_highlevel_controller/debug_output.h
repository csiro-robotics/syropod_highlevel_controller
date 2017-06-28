#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
/*******************************************************************************************************************//**
 *  @file    debug_output.h
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

#include "standard_includes.h"
#include <visualization_msgs/Marker.h>
#include "pose.h"
#include "model.h"
#include "walk_controller.h"

#define ROBOT_MODEL_ID 0       // Id for robot model
#define ID_LIMIT 10000         // Id value limit to prevent overflow
#define TRAJECTORY_DURATION 10 // Time for trajectory markers to exist (seconds)

/***********************************************************************************************************************
***********************************************************************************************************************/
class DebugOutput
{
public:
  /** Constructor for debug output class. Sets up publishers for the visualisation markers and initialises odometry. */
	DebugOutput(void);
  
  /** Resets the marker id to zero */
  inline void resetMarkerID(void) { marker_id_ = 0; };
  
  /** Modifier for the time_delta_ member variable */
  inline void setTimeDelta(const double& time_delta) { time_delta_ = time_delta; };
  
  /**
   * Updates the odometry pose of the robot body from velocity inputs
   * @param[in] linear_body_velocity The linear velocity of the robot body in the x/y plane
   * @param[in] angular_body_velocity The angular velocity of the robot body
   * @param[in] height The desired height of the robot body above ground
   */
  void updatePose(const Vector2d& linear_body_velocity, const double& angular_body_velocity, const double& height);

  /**
   * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
   * linking the origin points of each joint and tip of each leg.
   * @param[in] model A pointer to the robot model object
   */
  void generateRobotModel(shared_ptr<Model> model);
  
  /**
   * Publishes visualisation markers which represent the trajectory of the tip of the input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] current_pose The current pose of the body in the robot model - modifies the tip trajectory
   */
  void generateTipTrajectory(shared_ptr<Leg> leg, const Pose& current_pose);
  
  /**
   * Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip 
   * trajectory of the input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateBezierCurves(shared_ptr<Leg> leg);
  
  /**
   * Publishes visualisation markers which represent the walking workspace of the input leg as well as the current 
   * requested stride length.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] walker A pointer to the walk controller object.
   */
  void generateWorkspace(shared_ptr<Leg> leg, shared_ptr<WalkController> walker);
	
private:
	ros::NodeHandle n_;                        ///! Ros node handle
	ros::Publisher robot_model_publisher_;     ///! Publisher for topic "/robot_model"
  ros::Publisher tip_trajectory_publisher_;  ///! Publisher for topic "/tip_trajectories"
  ros::Publisher bezier_curve_publisher_;    ///! Publisher for topic "/bezier_curves"
  ros::Publisher workspace_publisher_;       ///! Publisher for topic "/workspaces"
  
	Pose odometry_pose_;      ///! Pose representing odometry pf robot model
  double time_delta_ = 0.0; ///! Time period of main loop cycle used for marker duration
  int marker_id_ = 0;       ///! Id for single loop duration markers
  int tip_position_id_ = 0; ///! Id for tip trajectory markers
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H */
