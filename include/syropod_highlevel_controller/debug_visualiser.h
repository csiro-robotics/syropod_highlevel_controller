#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
/*******************************************************************************************************************//**
 *  @file    debug_output.h
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

#include "standard_includes.h"
#include <visualization_msgs/Marker.h>
#include "pose.h"
#include "model.h"

/***********************************************************************************************************************
***********************************************************************************************************************/
class DebugVisualiser
{
public:
  /** Constructor for debug output class. Sets up publishers for the visualisation markers and initialises odometry. */
  DebugVisualiser(void);

  /** Modifier for the time_delta_ member variable */
  inline void setTimeDelta(const double& time_delta) { time_delta_ = time_delta; };

  /**
   * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
   * linking the origin points of each joint and tip of each leg.
   * @param[in] model A pointer to the robot model object
   * @param[in] ref_world A bool denoting if the generated markers should reference the world frame or default frame.
   */
  void generateRobotModel(shared_ptr<Model> model, const bool& ref_world = false);
  
  /**
   * Publishes visualisation markers which represent the estimated walking plane.
   * @param[in] walk_plane A Vector representing the walk plane
   * @param[in] walk_plane_normal A Vector of the normal to the walk plane
   */
  void generateWalkPlane(const Vector3d& walk_plane, const Vector3d& walk_plane_normal);

  /**
   * Publishes visualisation markers which represent the trajectory of the tip of the input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] limit The maximum number of trajectory markers to show.
   */
  void generateTipTrajectory(shared_ptr<Leg> leg, const int& limit);
  
  /**
   * Publishes visualisation markers which represent an estimate of the terrain being traversed
   * @param[in] model A pointer to the robot model object
   */
  void generateTerrainEstimate(shared_ptr<Model> model);

  /**
   * Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip
   * trajectory of the input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateBezierCurves(shared_ptr<Leg> leg);

  /**
   * Publises visualisation markers which represent the workspace for each leg.
   * @param[in] leg A pointer to a leg of the robot model object
   * @param[in] workspace_map A map of worksapce radii for a range of bearings
   * @param[in] ref_world A bool denoting if the generated markers should reference the world frame or default frame.
   */
  void generateWorkspace(shared_ptr<Leg> leg, map<int, double> workspace_map, const bool& ref_world = false);

  /**
   * Publishes visualisation markers which represent requested stride vector for each leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateStride(shared_ptr<Leg> leg);
  
  /**
   * Publishes visualisation markers which represent the estimated tip force vector for input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateTipForce(shared_ptr<Leg> leg);
  
  /**
   * Publishes visualisation markers which represent the estimated percentage of max torque in each joint.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateJointTorques(shared_ptr<Leg> leg);
  
  /**
   * Publishes visualisation markers which represent the estimate of the gravitational acceleration vector.
   * @param[in] gravity_estimate An estimate of the gravitational acceleration vector
   */
  void generateGravity(const Vector3d& gravity_estimate);

private:
  ros::NodeHandle n_;                        ///< Ros node handle
  ros::Publisher robot_model_publisher_;     ///< Publisher for topic "/shc/visualisation/robot_model"
  ros::Publisher tip_trajectory_publisher_;  ///< Publisher for topic "/shc/visualisation/tip_trajectories"
  ros::Publisher bezier_curve_publisher_;    ///< Publisher for topic "/shc/visualisation/bezier_curves"
  ros::Publisher workspace_publisher_;       ///< Publisher for topic "/shc/visualisation/workspaces"
  ros::Publisher walk_plane_publisher_;      ///< Publisher for topic "/shc/visualisation/walk_plane"
  ros::Publisher stride_publisher_;          ///< Publisher for topic "/shc/visualisation/stride"
  ros::Publisher tip_force_publisher_;       ///< Publisher for topic "/shc/visualisation/tip_force"
  ros::Publisher joint_torque_publisher_;    ///< Publisher for topic "/shc/visualisation/joint_torque"
  ros::Publisher tip_rotation_publisher_;    ///< Publisher for topic "/shc/visualisation/tip_rotation"
  ros::Publisher gravity_publisher_;         ///< Publisher for topic "/shc/visualisation/gravity"
  ros::Publisher terrain_publisher_;         ///< Publisher for topic "/shc/visualisation/terrain"

  double time_delta_ = 0.0;   ///< Time period of main loop cycle used for marker duration
  int tip_position_id_ = 0;   ///< Id for tip trajectory markers
  int terrain_marker_id_ = 0; ///< Id for terrain markers
  double marker_scale_ = 0.0; ///< Value used to scale marker sizes based on estimate of robot 'size'
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H */
