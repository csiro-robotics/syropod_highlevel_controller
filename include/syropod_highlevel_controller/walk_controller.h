#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H
/*******************************************************************************************************************//**
 *  @file    walk_controller.h
 *  @brief   Handles control of Syropod walking.
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
 **********************************************************************************************************************/

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "pose.h"
#include "model.h"

#define BEARING_STEP 45 ///< Step to increment bearing in workspace generation algorithm (deg)
#define WORKSPACE_GENERATION_MAX_ITERATIONS 100 ///< Maximum number of iterations to find limits of workspace
#define MAX_WORKSPACE_RADIUS 2.0 ///< Maximum radius allowed in workspace polygon (m)

class DebugVisualiser;

/*******************************************************************************************************************//**
 * This class handles top level management of the walk cycle state machine and calls each leg's LegStepper object to
 * update tip trajectories. This class also handles generation of default walk stance tip positions, calculation of
 * maximum body velocities and accelerations and transformation of input desired body velocities to individual tip
 * stride vectors.
***********************************************************************************************************************/
class WalkController : public enable_shared_from_this<WalkController>
{
public:
  /**
    * Constructor for the walk controller.
    * @param[in] model A pointer to the robot model.
    * @param[in] params A copy of the parameter data structure
    * @param[in] visualiser A pointer to the visualisation object
    */
  WalkController(shared_ptr<Model> model, const Parameters& params, shared_ptr<DebugVisualiser> visualiser);

  /** Accessor for pointer to parameter data structure. */
  inline const Parameters& getParameters(void) { return params_; };

  /** Accessor for step cycle phase length. */
  inline int getPhaseLength(void) { return phase_length_; };
  
  /** Accessor for step cycle swing period phase length. */
  inline int getSwingLength(void) { return swing_length_; };
  
  /** Accessor for step cycle stance period phase length. */
  inline int getStanceLength(void) { return stance_length_; };

  /** Accessor for phase for start of swing period of step cycle. */
  inline int getSwingStart(void) { return swing_start_; };

  /** Accessor for phase for end of swing period of step cycle. */
  inline int getSwingEnd(void) { return swing_end_; };

  /** Accessor for phase for start of stance period of step cycle. */
  inline int getStanceStart(void) { return stance_start_; };

  /** Accessor for phase for end of stance period of step cycle. */
  inline int getStanceEnd(void) { return stance_end_; };

  /** Accessor for ros cycle time period. */
  inline double getTimeDelta(void) { return time_delta_; };

  /** Accessor for step cycle frequency. */
  inline double getStepFrequency(void) { return step_frequency_; };

  /** Accessor for step clearance. */
  inline double getStepClearance(void) { return step_clearance_; };
  
  /** Calculates ratio of walk cycle that occurs on the walk plane. */
  inline double getOnGroundRatio(void) { return double(stance_length_) / double(phase_length_); };

  /** Accessor for default body clearance above ground. */
  inline double getBodyHeight(void) { return body_clearance_; };

  /** Accessor for desired linear body velocity. */
  inline Vector2d getDesiredLinearVelocity(void) { return desired_linear_velocity_; };

  /** Accessor for desired angular body velocity. */
  inline double getDesiredAngularVelocity(void) { return desired_angular_velocity_; };

  /** Accessor for walk cycle state. */
  inline WalkState getWalkState(void) { return walk_state_; };
  
  /** Accessor for workspace map. */
  inline map<int, double> getWorkspaceMap(void) { return workspace_map_; };
  
  /** Accessor for walk plane estimate. */
  inline Vector3d getWalkPlane(void) { return walk_plane_; };
  
  /** Accessor for normal to walk plane estimate. */
  inline Vector3d getWalkPlaneNormal(void) { return walk_plane_normal_; };
  
  /** Accessor for ideal odemetry pose. */
  inline Pose getOdometryIdeal(void) { return odometry_ideal_; };
  
  /** Accessor for model current pose. */
  inline Pose getModelCurrentPose(void) { return model_->getCurrentPose(); };

  /**
    * Modifier for posing state.
    * @param[in] state  The new posing state.
    */
  inline void setPoseState(const PosingState& state) { pose_state_ = state; };
  
  /** Modifier for generate workspace flag. */
  inline void regenerateWorkspace(void) { generate_workspace_ = true; };

  /**
   * Initialises walk controller by setting desired default walking stance tip positions from parameters and creating
   * LegStepper objects for each leg. Also populates workspace map with initial values by finding bisector line between
   * adjacent leg tip positions.
   */
  void init(void);

  /**
   * Generates universal workspace (map of radii for range of search bearings) for all legs by having each leg search
   * for joint limitations through bearings ranging from zero to 360 degrees. Workspace may be asymetrical, symetrical
   * (all opposite bearing pairs having equal distance) or circular (all search bearings having equal distance).
   * @params[in] self_loop Flag that determines if workspace generation occurs iteratively through multiple calls of 
   * this function or in while loop via a single call of this function.
   */
  int generateWorkspace(const bool& self_loop = true);

  /*
   * Calculate maximum linear and angular speed/acceleration for each workspace radius in workspace map. These
   * calculated values will accomodate overshoot of tip outside defined workspace whilst body accelerates, effectively
   * scaling usable
   */
  void calculateMaxSpeed(void);

  /**
    * Calculates walk controller walk cycle parameters, normalising base parameters according to step frequency.
    */
  void setGaitParams(void);

  /**
    * Updates all legs in the walk cycle. Calculates stride vectors for all legs from robot body velocity inputs and
    * calls trajectory update functions for each leg to update individual tip positions. Also manages the overall walk
    * state via state machine and input velocities as well as the individual step state of each leg as they progress
    * through stance and swing states.
    * @params[in] linear_velocity_input An input for the desired linear velocity of the robot body in the x/y plane.
    * @params[in] angular_velocity_input An input for the desired angular velocity of the robot body about the z axis.
    */
  void updateWalk(const Vector2d& linear_velocity_input, const double& angular_velocity_input);

  /**
    * Updates the tip position for legs in the manual state from tip velocity inputs. Two modes are available: joint
    * control allows manipulation of joint positions directly but only works for 3DOF legs; tip control allows
    * manipulation of the tip in cartesian space in the robot frame.
    * @params[in] primary_leg_selection_ID The designation of a leg selected (in the primary role) for manipulation.
    * @params[in] primary_tip_velocity_input The velocity input to move the 1st leg tip position in the robot frame.
    * @params[in] secondary_leg_selection_ID The designation of a leg selected (in the secondary role) for manipulation.
    * @params[in] secondary_tip_velocity_input The velocity input to move the 2nd leg tip position in the robot frame.
    */
  void updateManual(const int& primary_leg_selection_ID, const Vector3d& primary_tip_velocity_input,
                    const int& secondary_leg_selection_ID, const Vector3d& secondary_tip_velocity_input);
  
  /**
   * Calculates a estimated walk plane which best fits the tip positions of all legs using least squares method.
   * Transitions to updated walk plane estimates using the swing progress of a swinging leg as control input.
   * Walk plane vector in form: [a, b, c] where plane equation equals: ax + by + c = z.
   */
  void updateWalkPlane(void);
  
  /**
   * Estimates the acceleration vector due to gravity.
   * @return The estimated acceleration vector due to gravity.
   */
  Vector3d estimateGravity(void);
  
  /**
   * Calculates the change in pose over the desired time period assuming constant desired body velocity and walk plane.
   * @params[in] time_period The period of time for which to estimate the odometry pose change.
   * @return The estimated odometry pose change over the desired time period.
   */
  Pose calculateOdometry(const double& time_period);

private:
  shared_ptr<Model> model_;            ///< Pointer to robot model object.
  const Parameters& params_;           ///< Pointer to parameter data structure for storing parameter variables.
  shared_ptr<DebugVisualiser> debug_visualiser_; ///< Pointer to debug visualiser object
  double time_delta_;                  ///< The time period of the ros cycle.

  WalkState walk_state_ = STOPPED;           ///< The current walk cycle state.
  PosingState pose_state_ = POSING_COMPLETE; ///< The current state of auto posing.

  // Walk parameters
  double step_frequency_; ///< The frequency of the step cycle.
  double step_clearance_; ///< The desired clearance of the leg tip above default position during swing period.
  double step_depth_;     ///< The desired depth of the leg tip below default position during stance period.
  double body_clearance_; ///< The desired clearance of the body above the default tip positions.

  // Gait cycle parameters
  int phase_length_;  ///< The phase length of the step cycle.
  int swing_length_;  ///< The phase length of the swing period of the step cycle.
  int stance_length_; ///< The phase length of the stance period of the step cycle.
  int stance_end_;    ///< The phase at which the stance period ends.
  int swing_start_;   ///< The phase at which the swing period starts.
  int swing_end_;     ///< The phase at which the swing period ends.
  int stance_start_;  ///< The phase at which the stance period starts.

  // Workspace generation variables
  map<int, double> workspace_map_;                ///< A map of workspace radii for given bearing in degrees
  shared_ptr<Model> search_model_;                ///< A copy of the robot model used to search for kinematic limits
  bool workspace_generated_ = false;              ///< Flag denoting if workspace map has been generated.
  bool generate_workspace_ = true;                ///< Flag denoting if workspace is currently being generated
  double stance_radius_;                          ///< The radius of the turning circle used for angular body velocity.
  Vector3d walk_plane_;                           ///< The co-efficients of an estimated planar walk surface
  Vector3d walk_plane_normal_;                    ///< The normal of the estimated planar walk surface
  int iteration_ = 1;                             ///< The iteration of the workspace limit search along current bearing 
  int search_bearing_ = 0;                        ///< The current bearing being searched for kinematic limits
  double search_height_ = 0.0;                    ///< The current height being searched for kinematic limits
  vector<Vector3d> origin_tip_position_;          ///< The origin tip position of the kinematic search along a bearing.
  vector<Vector3d> target_tip_position_;          ///< The target tip position of the kinematic search along a bearing.

  // Velocity/acceleration variables
  Vector2d desired_linear_velocity_;          ///< The desired linear velocity of the robot body.
  double desired_angular_velocity_;           ///< The desired angular velocity of the robot body.
  Pose odometry_ideal_;                       ///< The ideal odometry from the world frame
  map<int, double> max_linear_speed_;         ///< A map of max allowable linear body speeds for potential bearings.
  map<int, double> max_angular_speed_;        ///< A map of max allowable angular speeds for potential bearings.
  map<int, double> max_linear_acceleration_;  ///< A map of max allowable linear accelerations for potential bearings.
  map<int, double> max_angular_acceleration_; ///< A map of max allowable angular accelerations for potential bearings.

  // Leg coordination variables
  int legs_at_correct_phase_ = 0;     ///< A count of legs currently at the correct phase per the walk cycle state.
  int legs_completed_first_step_ = 0; ///< A count of legs whcih have currently completed their first step.

  // Iteration variables
  LegContainer::iterator leg_it_;     ///< Leg iteration member variable used to minimise code
  JointContainer::iterator joint_it_; ///< Joint iteration member variable used to minimise code.
  LinkContainer::iterator link_it_;   ///< Link iteration member variable used to minimise code.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This class handles the generation of leg tip trajectory generation and updating the desired tip position along this
 * trajectory during iteration of the step cycle. Trajectories are generated using 3 bezier curves: a primary and
 * secondary curve for the swing period of the step cycle and one for the stance period of the step cycle.
 * Characteristics of the step trajectory are defined by parameters such as: step frequency, step clearance, step depth
 * and an input stride vector which is calculated from robot morphology and input desired body velocities.
***********************************************************************************************************************/
class LegStepper
{
public:
  /**
    * Leg stepper object constructor, initialises member variables from walk controller.
    * @param[in] walker A pointer to the walk controller.
    * @param[in] leg A pointer to the parent leg object.
    * @param[in] identity_tip_pose The default walking stance tip pose about which the step cycle is based.
    */
  LegStepper(shared_ptr<WalkController> walker, shared_ptr<Leg> leg, const Pose& identity_tip_pose);
  
  /**
   * Leg stepper object copy constructor, initialises member variables from reference leg stepper object.
   * @param[in] leg_stepper The reference leg stepper object to copy.
   */
  LegStepper(shared_ptr<LegStepper> leg_stepper);
  
  /** Accessor for pointer to parent leg object */
  inline shared_ptr<Leg> getParentLeg(void) { return leg_; };

  /** Accessor for the current tip pose according to the walk controller. */
  inline Pose getCurrentTipPose(void) { return current_tip_pose_; };

  /** Accessor for the default tip pose according to the walk controller. */
  inline Pose getDefaultTipPose(void) { return default_tip_pose_;};
  
  /** Accessor for the identity tip pose according to the walk controller. */
  inline Pose getIdentityTipPose(void) { return identity_tip_pose_; };
  
  /** Accessor for the target tip pose according to the walk controller. */
  inline Pose getTargetTipPose(void) { return target_tip_pose_;};

  /** Accessor for the current state of the walk cycle. */
  inline WalkState getWalkState(void) { return walker_->getWalkState(); };
  
  /** Accessor for the saved estimation of the walk plane */
  inline Vector3d getWalkPlane(void) { return walk_plane_; };
  
  /** Accessor for the normal of the saved estimation of the walk plane. */
  inline Vector3d getWalkPlaneNormal(void) { return walk_plane_normal_; };

  /** Accessor for the current state of the step cycle. */
  inline StepState getStepState(void) { return step_state_; };

  /** Accessor for the current phase of the step cycle. */
  inline int getPhase(void) { return phase_; };

  /** Accessor for the current phase offset of the step cycle.  */
  inline int getPhaseOffset(void) { return phase_offset_; };

  /** Accessor for the current stride vector used in the step cycle. */
  inline Vector3d getStrideVector(void) { return stride_vector_; };

  /** Accessor for desired clearance of the leg tip with respect to default position during swing period. */
  inline Vector3d getSwingClearance(void) { return swing_clearance_; };

  /** Accessor for the current progress of the swing period in the step cycle (0.0 -> 1.0 || -1.0). */
  inline double getSwingProgress(void) { return swing_progress_; };

  /** Accessor for the current progress of the stance period in the step cycle (0.0 -> 1.0 || -1.0) */
  inline double getStanceProgress(void) { return stance_progress_; };

  /** Returns true if leg has completed its first step whilst the walk state transitions from STOPPED to MOVING. */
  inline bool hasCompletedFirstStep(void) { return completed_first_step_; };

  /** Returns true if leg is in the correct step cycle phase per the walk controller state. */
  inline bool isAtCorrectPhase(void) { return at_correct_phase_; };

  /**
    * Accessor for control nodes in the primary swing bezier curve.
    * @param[in] i Index of the control node.
    */
  inline Vector3d getSwing1ControlNode(const int& i) { return swing_1_nodes_[i]; };

  /**
    * Accessor for control nodes in the secondary swing bezier curve.
    * @param[in] i Index of the control node.
    */
  inline Vector3d getSwing2ControlNode(const int& i) { return swing_2_nodes_[i]; };

  /**
    * Accessor for control nodes in the stance bezier curve.
    * @param[in] i Index of the control node.
    */
  inline Vector3d getStanceControlNode(const int& i) { return stance_nodes_[i]; };
  
  /** Accessor for the externally set target tip pose request time */
  inline ros::Time getTargetRequestTime(void) { return target_request_time_; };
  
  /**
    * Modifier for the pointer to the parent leg object
    * @param[in] parent_leg The new parent leg pointer.
    */
  inline void setParentLeg(shared_ptr<Leg> parent_leg) { leg_ = parent_leg; };

  /**
    * Modifier for the current tip pose according to the walk controller.
    * @param[in] current_tip_pose The new current tip pose.
    */
  inline void setCurrentTipPose(const Pose& current_tip_pose) { current_tip_pose_ = current_tip_pose; };
  
  /**
    * Modifier for the default tip pose according to the walk controller.
    * @param[in] tip_pose The new default tip pose.
    */
  inline void setDefaultTipPose(const Pose& tip_pose) { default_tip_pose_ = tip_pose; };

  /**
    * Modifier for the current state of step cycle.
    * @param[in] step_state The new state of the step cycle.
    */
  inline void setStepState(const StepState& step_state) { step_state_ = step_state; };

  /**
    * Modifier for the phase of the step cycle.
    * @param[in] phase The new phase.
    */
  inline void setPhase(const int& phase) { phase_ = phase; };
  
  /**
    * Modifier for the progress of the swing period.
    * @param[in] progress The new swing progress.
    */
  inline void setSwingProgress(const int& progress) { swing_progress_ = progress; };
  
  /**
    * Modifier for the progress of the stance period.
    * @param[in] progress The new stance progress.
    */
  inline void setStanceProgress(const int& progress) { stance_progress_ = progress; };

  /**
    * Modifier for the phase offset of the step cycle.
    * @param[in] phase_offset The new phase offset.
    */
  inline void setPhaseOffset(const int& phase_offset) { phase_offset_ = phase_offset;};

  /**
    * Modifier for the flag denoting if the leg has completed its first step.
    * @param[in] completed_first_step The new value for the flag.
    */
  inline void setCompletedFirstStep(const bool& completed_first_step) { completed_first_step_ = completed_first_step; };

  /**
    * Modifier for the flag denoting if the leg in in the correct phase.
    * @param[in] at_correct_phase The new value for the flag.
    */
  inline void setAtCorrectPhase(const bool& at_correct_phase) { at_correct_phase_ = at_correct_phase; };
  
  /**
   * Modifier for the flag denoting touchdown detection enabled.
   * @param[in] touchdown_detection The new value for the flag.
   */
  inline void setTouchdownDetection(const bool& touchdown_detection) { touchdown_detection_ = touchdown_detection; };
  
  /**
   * Modifier for the externally set target tip pose
   * @param[in] pose The new externally set target tip pose
   */
  inline void setExternalTargetTipPose(const Pose& pose) 
  { 
    external_target_tip_pose_ = pose;
    use_default_target_ = false;
  };
  
  /**
   * Modifier for the externally set target tip pose request time
   * @param[in] time The new externally set target tip pose request time
   */
  inline void setTargetRequestTime(const ros::Time& time) { target_request_time_ = time; };
  
  /**
   * Modifier for the transform between target tip pose reference frames at the time of request and current time.
   * @param[in] transform The new transform
   */
  inline void setTargetTipPoseTransform(const Pose& transform) { target_tip_pose_transform_ = transform; };

  /** Iterates the step phase and updates the progress variables */
  void iteratePhase(void);
  
  /** Updates the Step state of this LegStepper according to the phase */
  void updateStepState(void);
  
  /**
   * Updates the stride vector for this leg based on desired linear and angular velocity, with reference to the 
   * estimated walk plane. Also updates the swing clearance vector with reference to the estimated walk plane.
   */
  void updateStride(void);

  /**
    * Updates position of tip using three quartic bezier curves to generate the tip trajectory. Calculates change in
    * tip position using two bezier curves for swing phase and one for stance phase. Each Bezier curve uses 5 control
    * nodes designed specifically to give a C2 smooth trajectory for the entire step cycle.
    * @todo Move proactive target shifting to seperate node and use external target API
    */
  void updateTipPosition(void);
  
  /**
   * Updates rotation of tip orthogonal to the plane of the body during swing period. Interpolation from origin 
   * rotation to orthogonal rotation occurs during first half of swing and is kept orthogonal during second half.
   */
  void updateTipRotation(void);
  
  /**
   * Generates control nodes for quartic bezier curve of the 1st half of swing tip trajectory calculation.
   */
  void generatePrimarySwingControlNodes(void);
  
  /**
   * Generates control nodes for quartic bezier curve of the 2nd half of swing tip trajectory calculation.
   * @param[in] ground_contact Denotes if leg has made ground contact and swing trajectory towards ground should cease.
   */
  void generateSecondarySwingControlNodes(const bool& ground_contact = false);

  /**
   * Generates control nodes for quartic bezier curve of stance tip trajectory calculation.
   * @param[in] stride_scaler A scaling variable which modifies stride vector according to stance length specifically 
   * for STARTING state of walker
   */
  void generateStanceControlNodes(const double& stride_scaler);
  
  /**
   * Updates control nodes for quartic bezier curves of both halves of swing tip trajectory calculation to force the 
   * trajectory of the touchdown period of the swing period to be normal to the walk plane.
   */
  void forceNormalTouchdown(void);

private:
  shared_ptr<WalkController> walker_;  ///< Pointer to walk controller object.
  shared_ptr<Leg> leg_;                ///< Pointer to the parent leg object.

  bool at_correct_phase_ = false;     ///< Flag denoting if the leg is at the correct phase per the walk state.
  bool completed_first_step_ = false; ///< Flag denoting if the leg has completed its first step.
  bool use_default_target_ = true;    ///< Flag denoting if target tip pose is to be generated internally
  
  bool touchdown_detection_ = false;  ///< Flag denoting whether touchdown detection is enabled

  int phase_ = 0;    ///< Step cycle phase.
  int phase_offset_; ///< Step cycle phase offset.

  double swing_progress_ = -1.0;  ///< The progress of the swing period in the step cycle. (0.0->1.0 || -1.0)
  double stance_progress_ = 0.0;  ///< The progress of the stance period in the step cycle. (0.0->1.0 || -1.0)
  
  double average_tip_force_ = 0.0;
  double min_tip_force_ = UNASSIGNED_VALUE;

  StepState step_state_ = STANCE; ///< The state of the step cycle.

  Vector3d swing_1_nodes_[5]; ///< An array of 3d control nodes defining the primary swing bezier curve.
  Vector3d swing_2_nodes_[5]; ///< An array of 3d control nodes defining the secondary swing bezier curve.
  Vector3d stance_nodes_[5];  ///< An array of 3d control nodes defining the stance bezier curve.

  Vector3d walk_plane_;        ///< A saved version of the estimated walk plane which is kept static during swing
  Vector3d walk_plane_normal_; ///< The normal of the saved estimated planar walk surface
  Vector3d stride_vector_;     ///< The desired stride vector.
  Vector3d swing_clearance_;   ///< The position relative to the default tip position to achieve during swing period.

  double swing_delta_t_ = 0.0;
  double stance_delta_t_ = 0.0;

  Pose identity_tip_pose_;        ///< The user defined tip pose assuming a identity walk plane
  Pose default_tip_pose_;         ///< The default tip pose per the walk controller, updated with walk plane.
  Pose current_tip_pose_;         ///< The current tip pose per the walk controller.
  Pose origin_tip_pose_;          ///< The origin tip pose used in interpolation to target rotation.
  Pose target_tip_pose_;          ///< The target tip pose to achieve at the end of a swing period.

  Pose external_target_tip_pose_;  ///< The target tip pose to achieve at the end of a swing period (externally set)
  ros::Time target_request_time_;  ///< The ros time at which the external target tip pose request was made.
  Pose target_tip_pose_transform_; ///< The transform between reference frames at time of request and current time.
  
  Vector3d current_tip_velocity_;   ///< The default tip velocity per the walk controller.
  
  Vector3d swing_origin_tip_position_;  ///< The tip position used as the origin for the bezier curve during swing.
  Vector3d swing_origin_tip_velocity_;  ///< The tip velocity used in the generation of bezier curve during swing.
  Vector3d stance_origin_tip_position_; ///< The tip position used as the origin for the bezier curve during stance.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H */


