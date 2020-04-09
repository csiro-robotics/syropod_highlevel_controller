////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H

#include "standard_includes.h"
#include "pose.h"
#include "model.h"
#include "walk_controller.h"

#define ID_LIMIT 10000                    ///< Id value limit to prevent overflow
#define TRAJECTORY_DURATION 10            ///< Time for trajectory markers to exist (sec)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles generation and publishing of visualisations for display in rviz for debugging purposes.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DebugVisualiser
{
public:
  /// Constructor for debug output class. Sets up publishers for the visualisation markers and initialises odometry.
  DebugVisualiser(void);

  /// Modifier for the time_delta_ member variable.
  inline void setTimeDelta(const double &time_delta) { time_delta_ = time_delta; };

  /// Publishes visualisation markers which represent the robot model for display in RVIZ. Consists of line segments.
  /// linking the origin points of each joint and tip of each leg.
  /// @param[in] model A pointer to the robot model object
  void generateRobotModel(std::shared_ptr<Model> model);

  /// Publishes visualisation markers which represent the estimated walking plane.
  /// @param[in] walk_plane A Vector representing the walk plane
  /// @param[in] walk_plane_normal A Vector of the normal to the walk plane
  void generateWalkPlane(const Eigen::Vector3d &walk_plane, const Eigen::Vector3d &walk_plane_normal);

  /// Publishes visualisation markers which represent the trajectory of the tip of the input leg.
  /// @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published
  void generateTipTrajectory(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent an estimate of the terrain being traversed.
  /// @param[in] model A pointer to the robot model object
  void generateTerrainEstimate(std::shared_ptr<Model> model);

  /// Publishes visualisation markers which represent the control nodes of the three bezier curves used to control tip.
  /// trajectory of the input leg.
  /// @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published
  void generateBezierCurves(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent the default tip position of the leg.
  /// @param[in] leg A pointer to a leg of the robot model object
  void generateDefaultTipPositions(std::shared_ptr<Leg> leg);

  /// Publises visualisation markers which represent the target tip position of the leg.
  /// @param[in] leg A pointer to a leg of the robot model object
  void generateTargetTipPositions(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent the 2D walkspace for each leg.
  /// @param[in] leg A pointer to a leg of the robot model object
  /// @param[in] walkspace  A map of walkspace radii for a range of bearings to be visualised
  void generateWalkspace(std::shared_ptr<Leg> leg, const LimitMap &walkspace);

  /// Publishes visualisation markers which represent the 3D workspace for each leg.
  /// @param[in] leg A pointer to a leg of the robot model object
  /// @param[in] body_clearance The vertical offset of the body above the walk plane
  void generateWorkspace(std::shared_ptr<Leg> leg, const double &body_clearance);

  /// Publishes visualisation markers which represent requested stride vector for each leg.
  /// @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published
  void generateStride(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent the estimated tip force vector for input leg.
  /// @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published
  void generateTipForce(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent the estimated percentage of max torque in each joint.
  /// @param[in] leg A pointer to the leg associated with the tip trajectory that is to be published
  void generateJointTorques(std::shared_ptr<Leg> leg);

  /// Publishes visualisation markers which represent the estimate of the gravitational acceleration vector.
  /// @param[in] gravity_estimate An estimate of the gravitational acceleration vector
  void generateGravity(const Eigen::Vector3d& gravity_estimate);

private:
  ros::Publisher robot_model_publisher_;          ///< Publisher for topic "/shc/visualisation/robot_model"
  ros::Publisher tip_trajectory_publisher_;       ///< Publisher for topic "/shc/visualisation/tip_trajectories"
  ros::Publisher bezier_curve_publisher_;         ///< Publisher for topic "/shc/visualisation/bezier_curves"
  ros::Publisher default_tip_position_publisher_; ///< Publisher for topic "/shc/visualisation/default_tip_positions"
  ros::Publisher target_tip_position_publisher_;  ///< Publisher for topic "/shc/visualisation/target_tip_positions"
  ros::Publisher walkspace_publisher_;            ///< Publisher for topic "/shc/visualisation/walkspace"
  ros::Publisher workspace_publisher_;            ///< Publisher for topic "/shc/visualisation/workspace"
  ros::Publisher walk_plane_publisher_;           ///< Publisher for topic "/shc/visualisation/walk_plane"
  ros::Publisher stride_publisher_;               ///< Publisher for topic "/shc/visualisation/stride"
  ros::Publisher tip_force_publisher_;            ///< Publisher for topic "/shc/visualisation/tip_force"
  ros::Publisher joint_torque_publisher_;         ///< Publisher for topic "/shc/visualisation/joint_torque"
  ros::Publisher tip_rotation_publisher_;         ///< Publisher for topic "/shc/visualisation/tip_rotation"
  ros::Publisher gravity_publisher_;              ///< Publisher for topic "/shc/visualisation/gravity"
  ros::Publisher terrain_publisher_;              ///< Publisher for topic "/shc/visualisation/terrain"

  double time_delta_ = 0.0;   ///< Time period of main loop cycle used for marker duration
  int tip_position_id_ = 0;   ///< Id for tip trajectory markers
  int terrain_marker_id_ = 0; ///< Id for terrain markers
  double marker_scale_ = 0.0; ///< Value used to scale marker sizes based on estimate of robot 'size'

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_DEBUG_OUTPUT_H