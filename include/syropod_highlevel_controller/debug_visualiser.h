#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
/*******************************************************************************************************************//**
 *  @file    debug_output.h
 *  @brief   Handles publishing of Syropod model info for debugging in RVIZ.
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

#include "standard_includes.h"
#include <visualization_msgs/Marker.h>
#include "pose.h"
#include "model.h"

#define ROBOT_MODEL_ID 0                  ///< Id for robot model visualisation
#define WALK_PLANE_ID 10                  ///< Id for walk plane visualisation
#define WORKSPACE_ID 20                   ///< Base id for workspace visualisations
#define DEFAULT_TIP_POSITION_ID 30        ///< Base id for default tip positions
#define STRIDE_MARKER_ID 40               ///< Base id for stride visualisations
#define SWING_BEZIER_CURVE_1_MARKER_ID 50 ///< Base id for bezier curve visualisations
#define SWING_BEZIER_CURVE_2_MARKER_ID 60 ///< Base id for bezier curve visualisations
#define STANCE_BEZIER_CURVE_MARKER_ID 70  ///< Base id for bezier curve visualisations
#define TIP_FORCE_MARKER_ID 80            ///< Base id for tip force vector visualisations
#define TIP_ROTATION_MARKER_ID 90         ///< Base id for tip rotation visualisations
#define ID_LIMIT 10000                    ///< Id value limit to prevent overflow
#define TRAJECTORY_DURATION 10            ///< Time for trajectory markers to exist (sec)

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
   * Updates the odometry pose of the robot body from velocity inputs
   * @param[in] linear_body_velocity The linear velocity of the robot body in the x/y plane
   * @param[in] angular_body_velocity The angular velocity of the robot body
   * @param[in] walk_plane A Vector representing the walk plane
   */
  void updatePose(const Vector2d& linear_body_velocity,
                  const double& angular_body_velocity,
                  const Vector3d& walk_plane);

  /**
   * Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments
   * linking the origin points of each joint and tip of each leg.
   * @param[in] model A pointer to the robot model object
   */
  void generateRobotModel(shared_ptr<Model> model);
  
  /**
   * Publishes visualisation markers which represent the estimated walking plane.
   * @param[in] walk_plane A Vector representing the walk plane
   */
  void generateWalkPlane(const Vector3d& walk_plane);

  /**
   * Publishes visualisation markers which represent the trajectory of the tip of the input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] current_pose The current pose of the body in the robot model - modifies the tip trajectory
   */
  void generateTipTrajectory(shared_ptr<Leg> leg, const Pose& current_pose);
  
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
   */
  void generateWorkspace(shared_ptr<Leg> leg, map<int, double> workspace_map);

  /**
   * Publishes visualisation markers which represent requested stride vector for each leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   */
  void generateStride(shared_ptr<Leg> leg);
  
  /**
   * Publishes visualisation markers which represent the estimated tip force vector for input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] current_pose The current pose of the body in the robot model.
   */
  void generateTipForce(shared_ptr<Leg> leg, const Pose& current_pose);
  
  /**
   * Publishes visualisation markers which represent the orientation of the tip for input leg.
   * @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published.
   * @param[in] current_pose The current pose of the body in the robot model.
   */
  void generateTipRotation(shared_ptr<Leg> leg, const Pose& current_pose);

private:
  ros::NodeHandle n_;                        ///< Ros node handle
  ros::Publisher robot_model_publisher_;     ///< Publisher for topic "/shc/visualisation/robot_model"
  ros::Publisher tip_trajectory_publisher_;  ///< Publisher for topic "/shc/visualisation/tip_trajectories"
  ros::Publisher bezier_curve_publisher_;    ///< Publisher for topic "/shc/visualisation/bezier_curves"
  ros::Publisher workspace_publisher_;       ///< Publisher for topic "/shc/visualisation/workspaces"
  ros::Publisher stride_publisher_;          ///< Publisher for topic "/shc/visualisation/stride"
  ros::Publisher tip_force_publisher_;       ///< Publisher for topic "/shc/visualisation/tip_force"
  ros::Publisher tip_rotation_publisher_;    ///< Publisher for topic "/shc/visualisation/tip_rotation"

  Pose odometry_pose_;        ///< Pose representing odometry pf robot model
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
