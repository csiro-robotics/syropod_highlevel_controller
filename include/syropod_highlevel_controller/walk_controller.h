////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "pose.h"
#include "model.h"

class DebugVisualiser;
typedef std::map<int, double> LimitMap;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Object containing parameters which define the timing of the step cycle.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct StepCycle
{
  double frequency_;  ///< The frequency of the step cycle in Hz
  int period_;        ///< The length of the entire step cycle in iterations
  int swing_period_;  ///< The length of the swing period of the step cycle in iterations
  int stance_period_; ///< The length of the stance period of the step cycle in iterations
  int stance_end_;    ///< The iteration at which the stance period ends
  int swing_start_;   ///< The iteration at which the swing period starts
  int swing_end_;     ///< The iteration at which the swing period ends
  int stance_start_;  ///< The iteration at which the stance period starts
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Object containing parameters which define an externally set target tip pose.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct ExternalTarget
{
  Pose pose_;              ///< The target tip pose
  double swing_clearance_; ///< The height of the swing trajectory clearance normal to walk plane
  std::string frame_id_;        ///< The target tip pose reference frame id
  ros::Time time_;         ///< The ros time of the request for the target tip pose
  Pose transform_;         ///< The transform between reference frames at time of request and current time
  bool defined_ = false;   ///< Flag denoting if external target object has been defined
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles top level management of the walk cycle state machine and calls each leg's LegStepper object to
/// update tip trajectories. This class also handles generation of default walk stance tip positions, calculation of
/// maximum body velocities and accelerations and transformation of input desired body velocities to individual tip
/// stride vectors.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class WalkController : public std::enable_shared_from_this<WalkController>
{
public:
  /// Constructor for the walk controller.
  /// @param[in] model A pointer to the robot model
  /// @param[in] params A copy of the parameter data structure
  WalkController(std::shared_ptr<Model> model, const Parameters& params);

  /// Accessor for pointer to parameter data structure.
  /// @return Pointer to parameter data structure
  inline const Parameters& getParameters(void) { return params_; };

  /// Accessor for step timing object.
  /// @return Step cycle timing object
  inline StepCycle getStepCycle(void) { return step_; };

  /// Accessor for ros cycle time period.
  /// @return ROS cycle time period
  inline double getTimeDelta(void) { return time_delta_; };

  /// Accessor for step clearance.
  /// @return Step clearance
  inline double getStepClearance(void) { return params_.swing_height.current_value; };
  
  /// Accessor for step depth.
  /// @return Step depth
  inline double getStepDepth(void) { return params_.step_depth.current_value; };

  /// Accessor for default body clearance above ground.
  /// @return Body clearance above ground
  inline double getBodyClearance(void) { return params_.body_clearance.data; };

  /// Accessor for desired linear body velocity. 
  /// @return Desired linear body velocity
  inline Eigen::Vector2d getDesiredLinearVelocity(void) { return desired_linear_velocity_; };

  /// Accessor for desired angular body velocity. 
  /// @return Desired angular body velocity
  inline double getDesiredAngularVelocity(void) { return desired_angular_velocity_; };

  /// Accessor for walk cycle state. 
  /// @return Walk cycle state
  inline WalkState getWalkState(void) { return walk_state_; };
  
  /// Accessor for walkspace. 
  /// @return Walkspace
  inline LimitMap getWalkspace(void) { return walkspace_; };
  
  /// Accessor for walk plane estimate. 
  /// @return Walk plane estimate
  inline Eigen::Vector3d getWalkPlane(void) { return walk_plane_; };
  
  /// Accessor for normal to walk plane estimate. 
  /// @return Normal to walk plane estimate
  inline Eigen::Vector3d getWalkPlaneNormal(void) { return walk_plane_normal_; };
  
  /// Accessor for ideal odemetry pose.
  /// @return Ideal odometry pose
  inline Pose getOdometryIdeal(void) { return odometry_ideal_; };
  
  /// Accessor for model current pose. 
  /// @return Model current pose
  inline Pose getModelCurrentPose(void) { return model_->getCurrentPose(); };

  /// Modifier for posing state.
  /// @param[in] state The new posing state
  inline void setPoseState(const PosingState& state) { pose_state_ = state; };
  
  /// Modifier for linear velocity limit map. 
  /// @param[in] limit_map The new linear velocity limit map
  inline void setLinearSpeedLimitMap(const LimitMap& limit_map) { max_linear_speed_ = limit_map; };
  
  /// Modifier for angular velocity limit map. 
  /// @param[in] limit_map The new angular velocity limit map
  inline void setAngularSpeedLimitMap(const LimitMap& limit_map) { max_angular_speed_ = limit_map; };
  
  /// Modifier for linear acceleration limit map. 
  /// @param[in] limit_map The new linear aceleration limit map
  inline void setLinearAccelerationLimitMap(const LimitMap& limit_map) { max_linear_acceleration_ = limit_map; };
  
  /// Modifier for angular acceleration limit map. 
  /// @param[in] limit_map The new angular acceleration limit map
  inline void setAngularAccelerationLimitMap(const LimitMap& limit_map) { max_angular_acceleration_ = limit_map; };
  
  /// Sets flag to regenerate walkspace.
  inline void setRegenerateWalkspace(void) { regenerate_walkspace_ = true; };

  /// Initialises walk controller by setting desired default walking stance tip positions from parameters and creating
  /// LegStepper objects for each leg. Also populates workspace map with initial values by finding bisector line between
  /// adjacent leg tip positions.
  void init(void);

  /// Generates a 2D polygon from leg workspace, representing the acceptable space to walk within.
  /// @todo Remove debugging visualisations
  void generateWalkspace(void);

  /// Generate maximum linear and angular speed/acceleration for each workspace radius in workspace map from a given 
  /// step cycle. These calculated values will accomodate overshoot of tip outside defined workspace whilst body 
  /// accelerates, effectively scaling usable workspace. The calculated values are either set as walk controller limits
  /// OR output to given pointer arguments.
  /// @param[in] step Step cycle timing object
  /// @param[out] max_linear_speed_ptr Pointer to output object to store new maximum linear speed values
  /// @param[out] max_angular_speed_ptr Pointer to output object to store new maximum angular speed values
  /// @param[out] max_linear_acceleration_ptr Pointer to output object to store new maximum linear acceleration values
  /// @param[out] max_angular_acceleration_ptr Pointer to output object to store new maximum angular acceleration values
  void generateLimits(StepCycle step,
                      LimitMap* max_linear_speed_ptr = NULL,
                      LimitMap* max_angular_speed_ptr = NULL, 
                      LimitMap* max_linear_acceleration_ptr = NULL,
                      LimitMap* max_angular_acceleration_ptr = NULL);
  
  /// Generate maximum linear and angular speed/acceleration for each workspace radius in workspace map from pre-set 
  /// step cycle. These calculated values will accomodate overshoot of tip outside defined workspace whilst body 
  /// accelerates, effectively scaling usable workspace. The calculated values are either set as walk controller limits
  /// OR output to given pointer arguments.
  /// @param[out] max_linear_speed_ptr Pointer to output object to store new maximum linear speed values
  /// @param[out] max_angular_speed_ptr Pointer to output object to store new maximum angular speed values
  /// @param[out] max_linear_acceleration_ptr Pointer to output object to store new maximum linear acceleration values
  /// @param[out] max_angular_acceleration_ptr Pointer to output object to store new maximum angular acceleration values
  void inline generateLimits(LimitMap* max_linear_speed_ptr = NULL,
                             LimitMap* max_angular_speed_ptr = NULL, 
                             LimitMap* max_linear_acceleration_ptr = NULL,
                             LimitMap* max_angular_acceleration_ptr = NULL)
  {
    generateLimits(step_, max_linear_speed_ptr, max_angular_speed_ptr,
                   max_linear_acceleration_ptr, max_angular_acceleration_ptr);
  };

  /// Generates step timing object from walk cycle parameters, normalising base parameters according to step frequency.
  /// Returns step timing object and optionally sets step timing in Walk Controller.
  /// @param[in] set_step_cycle Flag denoting if generated step cycle object is to be set in Walk Controller
  /// @return Generated step cycle object
  StepCycle generateStepCycle(const bool set_step_cycle = true);
  
  /// Given an input linear velocity vector and angular velocity, this function calculates a stride bearing then 
  /// an interpolation of the two limits at the bearings (defined by the input limit map) bounding the stride bearing.
  /// This is calculated for each leg and the minimum value returned.
  /// @param[in] linear_velocity_input The velocity input given to the Syropod defining desired linear body motion
  /// @param[in] angular_velocity_input The velocity input given to the Syropod defining desired angular body motion
  /// @param[in] limit The LimitMap object which contains limit data for a range of bearings from 0-360 degrees
  /// @return The smallest interpolated limit for a given bearing from each of the Syropod legs
  double getLimit(const Eigen::Vector2d& linear_velocity_input, const double& angular_velocity_input, const LimitMap& limit);

  /// Updates all legs in the walk cycle. Calculates stride vectors for all legs from robot body velocity inputs and
  /// calls trajectory update functions for each leg to update individual tip positions. Also manages the overall walk
  /// state via state machine and input velocities as well as the individual step state of each leg as they progress
  /// through stance and swing states.
  /// @param[in] linear_velocity_input An input for the desired linear velocity of the robot body in the x/y plane
  /// @param[in] angular_velocity_input An input for the desired angular velocity of the robot body about the z axis
  void updateWalk(const Eigen::Vector2d& linear_velocity_input, const double& angular_velocity_input);

  /// Updates the tip position for legs in the manual state from tip velocity inputs. Two modes are available: joint
  /// control allows manipulation of joint positions directly but only works for 3DOF legs; tip control allows
  /// manipulation of the tip in cartesian space in the robot frame.
  /// @param[in] primary_leg_selection_ID The designation of a leg selected (in the primary role) for manipulation
  /// @param[in] primary_tip_velocity_input The velocity input to move the 1st leg tip position in the robot frame
  /// @param[in] secondary_leg_selection_ID The designation of a leg selected (in the secondary role) for manipulation
  /// @param[in] secondary_tip_velocity_input The velocity input to move the 2nd leg tip position in the robot frame
  void updateManual(const int& primary_leg_selection_ID, const Eigen::Vector3d& primary_tip_velocity_input,
                    const int& secondary_leg_selection_ID, const Eigen::Vector3d& secondary_tip_velocity_input);
  
  /// Calculates a estimated walk plane which best fits the default tip positions of legs in model.
  /// Walk plane vector in form: [a, b, c] where plane equation equals: ax + by + c = z.
  /// Ref: https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  void updateWalkPlane(void);
  
  /// Estimates the acceleration vector due to gravity.
  /// @return The estimated acceleration vector due to gravity
  inline Eigen::Vector3d estimateGravity(void) { return model_->estimateGravity(); };
  
  /// Calculates the change in pose over the desired time period assuming constant desired body velocity and walk plane.
  /// @param[in] time_period The period of time for which to estimate the odometry pose change
  /// @return The estimated odometry pose change over the desired time period
  Pose calculateOdometry(const double& time_period);

private:
  std::shared_ptr<Model> model_;            ///< Pointer to robot model object
  const Parameters& params_;           ///< Pointer to parameter data structure for storing parameter variables
  double time_delta_;                  ///< The time period of the ros cycle

  WalkState walk_state_ = STOPPED;           ///< The current walk cycle state
  PosingState pose_state_ = POSING_COMPLETE; ///< The current state of auto posing

  // Step cycle timing object
  StepCycle step_;

  // Workspace generation variables
  LimitMap walkspace_;                ///< A map of interpolated radii for given bearings in degrees at default stance
  Eigen::Vector3d walk_plane_;               ///< The co-efficients of an estimated planar walk surface
  Eigen::Vector3d walk_plane_normal_;        ///< The normal of the estimated planar walk surface
  bool regenerate_walkspace_ = false; ///< Flag denoting whether walkspace needs to be regenerated

  // Velocity/acceleration variables
  Eigen::Vector2d desired_linear_velocity_;          ///< The desired linear velocity of the robot body
  double desired_angular_velocity_;           ///< The desired angular velocity of the robot body
  Pose odometry_ideal_;                       ///< The ideal odometry from the world frame
  LimitMap max_linear_speed_;         ///< A map of max allowable linear body speeds for potential bearings
  LimitMap max_angular_speed_;        ///< A map of max allowable angular speeds for potential bearings
  LimitMap max_linear_acceleration_;  ///< A map of max allowable linear accelerations for potential bearings
  LimitMap max_angular_acceleration_; ///< A map of max allowable angular accelerations for potential bearings

  // Leg coordination variables
  int legs_at_correct_phase_ = 0;            ///< A count of legs currently at the correct phase per walk cycle state
  int legs_completed_first_step_ = 0;        ///< A count of legs whcih have currently completed their first step
  bool return_to_default_attempted_ = false; ///< Flags whether a leg has already attempted to return to default

  // Iteration variables
  LegContainer::iterator leg_it_;     ///< Leg iteration member variable used to minimise code
  JointContainer::iterator joint_it_; ///< Joint iteration member variable used to minimise code
  LinkContainer::iterator link_it_;   ///< Link iteration member variable used to minimise code

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles the generation of leg tip trajectory generation and updating the desired tip position along this
/// trajectory during iteration of the step cycle. Trajectories are generated using 3 bezier curves: a primary and
/// secondary curve for the swing period of the step cycle and one for the stance period of the step cycle.
/// Characteristics of the step trajectory are defined by parameters such as: step frequency, step clearance, step depth
/// and an input stride vector which is calculated from robot morphology and input desired body velocities.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LegStepper
{
public:
  /// Leg stepper object constructor, initialises member variables from walk controller.
  /// @param[in] walker A pointer to the walk controller
  /// @param[in] leg A pointer to the parent leg object
  /// @param[in] identity_tip_pose The default walking stance tip pose about which the step cycle is based
  LegStepper(std::shared_ptr<WalkController> walker, std::shared_ptr<Leg> leg, const Pose& identity_tip_pose);
  
  /// Leg stepper object copy constructor, initialises member variables from reference leg stepper object.
  /// @param[in] leg_stepper The reference leg stepper object to copy
  LegStepper(std::shared_ptr<LegStepper> leg_stepper);
  
  /// Accessor for pointer to parent leg object.
  /// @return Pointer to parent leg object
  inline std::shared_ptr<Leg> getParentLeg(void) { return leg_; };

  /// Accessor for the current tip pose according to the walk controller.
  /// @return Current tip pose accoding to the walk controller
  inline Pose getCurrentTipPose(void) { return current_tip_pose_; };

  /// Accessor for the default tip pose according to the walk controller.
  /// @return Default tip pose according to the walk controller
  inline Pose getDefaultTipPose(void) { return default_tip_pose_;};
  
  /// Accessor for the identity tip pose according to the walk controller.
  /// @return Identity tip pose according to the walk controller
  inline Pose getIdentityTipPose(void) { return identity_tip_pose_; };
  
  /// Accessor for the target tip pose according to the walk controller.
  /// @return Target tip pose according to the walk controller
  inline Pose getTargetTipPose(void) { return target_tip_pose_;};

  /// Accessor for the current state of the walk cycle.
  /// @return Current state of the walk cycle
  inline WalkState getWalkState(void) { return walker_->getWalkState(); };
  
  /// Accessor for the saved estimation of the walk plane.
  /// @return Saved estimation of the walk plane
  inline Eigen::Vector3d getWalkPlane(void) { return walk_plane_; };
  
  /// Accessor for the normal of the saved estimation of the walk plane.
  /// @return Normal of the saved estimation of the walk plane
  inline Eigen::Vector3d getWalkPlaneNormal(void) { return walk_plane_normal_; };

  /// Accessor for the current state of the step cycle.
  /// @return Current state of the step cycle
  inline StepState getStepState(void) { return step_state_; };

  /// Accessor for the current phase of the step cycle.
  /// @return Current phase of the step cycle
  inline int getPhase(void) { return phase_; };

  /// Accessor for the current phase offset of the step cycle.
  /// @return Current phase offset of the step cycle
  inline int getPhaseOffset(void) { return phase_offset_; };

  /// Accessor for the current stride vector used in the step cycle.
  /// @return Current stride vector used in the step cycle
  inline Eigen::Vector3d getStrideVector(void) { return stride_vector_; };

  /// Accessor for desired clearance of the leg tip with respect to default position during swing period.
  /// @return Desired clearance of the leg tip with respect to default position during swing period
  inline Eigen::Vector3d getSwingClearance(void) { return swing_clearance_; };

  /// Accessor for the current progress of the swing period in the step cycle (0.0 -> 1.0 || -1.0). 
  /// @return Current progress of the swing period in the step cycle (0.0 -> 1.0 || -1.0)
  inline double getSwingProgress(void) { return swing_progress_; };

  /// Accessor for the current progress of the stance period in the step cycle (0.0 -> 1.0 || -1.0).
  /// @return Current progress of the stance period in the step cycle (0.0 -> 1.0 || -1.0)
  inline double getStanceProgress(void) { return stance_progress_; };

  /// Returns true if leg has completed its first step whilst the walk state transitions from STOPPED to MOVING. 
  /// @return Flag denoting whether the leg has completed its first step whilst the walk state transitions from STOPPED to MOVING
  inline bool hasCompletedFirstStep(void) { return completed_first_step_; };

  /// Returns true if leg is in the correct step cycle phase per the walk controller state. 
  /// @return Flag denoting whether the leg is in the correct step cycle phase per the walk controller state
  inline bool isAtCorrectPhase(void) { return at_correct_phase_; };

  /// Accessor for control nodes in the primary swing bezier curve.
  /// @param[in] i Index of the control node
  /// @return Control node in the primary swing bezier curve of the given index
  inline Eigen::Vector3d getSwing1ControlNode(const int& i) { return swing_1_nodes_[i]; };

  /// Accessor for control nodes in the secondary swing bezier curve.
  /// @param[in] i Index of the control node
  /// @return Control node in the secondary swing bezier curve of the given index
  inline Eigen::Vector3d getSwing2ControlNode(const int& i) { return swing_2_nodes_[i]; };

  /// Accessor for control nodes in the stance bezier curve.
  /// @param[in] i Index of the control node
  /// @return Control node in the stance bezier curve of the given index
  inline Eigen::Vector3d getStanceControlNode(const int& i) { return stance_nodes_[i]; };
  
  /// Accessor for the externally set target tip pose object.
  /// @return Externally set target tip pose object
  inline ExternalTarget getExternalTarget(void) { return external_target_; };
  
  /// Accessor for the externally set default tip pose object.
  /// @return Externally set default tip pose object
  inline ExternalTarget getExternalDefault(void) { return external_default_; };
  
  /// Modifier for the pointer to the parent leg object.
  /// @param[in] parent_leg The new parent leg pointer
  inline void setParentLeg(std::shared_ptr<Leg> parent_leg) { leg_ = parent_leg; };

  /// Modifier for the current tip pose according to the walk controller.
  /// @param[in] current_tip_pose The new current tip pose
  inline void setCurrentTipPose(const Pose& current_tip_pose) { current_tip_pose_ = current_tip_pose; };
  
  /// Modifier for the default tip pose according to the walk controller.
  /// @param[in] tip_pose The new default tip pose
  inline void setDefaultTipPose(const Pose& tip_pose) { default_tip_pose_ = tip_pose; };

  /// Modifier for the current state of step cycle.
  /// @param[in] step_state The new state of the step cycle
  inline void setStepState(const StepState& step_state) { step_state_ = step_state; };

  /// Modifier for the phase of the step cycle.
  /// @param[in] phase The new phase
  inline void setPhase(const int& phase) { phase_ = phase; };
  
  /// Modifier for the progress of the swing period.
  /// @param[in] progress The new swing progress
  inline void setSwingProgress(const int& progress) { swing_progress_ = progress; };
  
  /// Modifier for the progress of the stance period.
  /// @param[in] progress The new stance progress
  inline void setStanceProgress(const int& progress) { stance_progress_ = progress; };

  /// Modifier for the phase offset of the step cycle.
  /// @param[in] phase_offset The new phase offset
  inline void setPhaseOffset(const int& phase_offset) { phase_offset_ = phase_offset;};

  /// Modifier for the flag denoting if the leg has completed its first step.
  /// @param[in] completed_first_step The new value for the flag
  inline void setCompletedFirstStep(const bool& completed_first_step) { completed_first_step_ = completed_first_step; };

  /// Modifier for the flag denoting if the leg in in the correct phase.
  /// @param[in] at_correct_phase The new value for the flag
  inline void setAtCorrectPhase(const bool& at_correct_phase) { at_correct_phase_ = at_correct_phase; };
  
  /// Modifier for the flag denoting touchdown detection enabled.
  /// @param[in] touchdown_detection The new value for the flag
  inline void setTouchdownDetection(const bool& touchdown_detection) { touchdown_detection_ = touchdown_detection; };
  
  /// Modifier for the externally set target tip pose.
  /// @param[in] external_target The new externally set target tip pose object
  inline void setExternalTarget(const ExternalTarget& external_target) { external_target_ = external_target; };
  
  /// Modifier for the externally set default tip pose object.
  /// @param[in] external_default The new externally set default tip pose object
  inline void setExternalDefault(const ExternalTarget& external_default) { external_default_ = external_default; };
  
  /// Updates phase for new step cycle parameters.
  void updatePhase(void);

  /// Iterates the step phase and updates the progress variables.
  void iteratePhase(void);
  
  /// Updates the Step state of this LegStepper according to the phase.
  void updateStepState(void);
  
  /// Updates the stride vector for this leg based on desired linear and angular velocity, with reference to the 
  /// estimated walk plane. Also updates the swing clearance vector with reference to the estimated walk plane.
  void updateStride(void);
  
  /// Calculates the lateral change in distance from identity tip position to new default tip position for a leg.
  /// @return The change from identity tip position to the new default tip position
  Eigen::Vector3d calculateStanceSpanChange(void);
  
  /// Update Default Tip Position based on external definitions, stance span or tip position at beginning of stance.
  void updateDefaultTipPosition(void);

  /// Updates position of tip using three quartic bezier curves to generate the tip trajectory. Calculates change in
  /// tip position using two bezier curves for swing phase and one for stance phase. Each Bezier curve uses 5 control
  /// nodes designed specifically to give a C2 smooth trajectory for the entire step cycle.
  /// @todo Move proactive target shifting to seperate node and use external target API
  void updateTipPosition(void);
  
  /// Updates rotation of tip orthogonal to the plane of the body during swing period. Interpolation from origin 
  /// rotation to orthogonal rotation occurs during first half of swing and is kept orthogonal during second half.
  void updateTipRotation(void);
  
  /// Generates control nodes for quartic bezier curve of the 1st half of swing tip trajectory calculation.
  void generatePrimarySwingControlNodes(void);
  
  /// Generates control nodes for quartic bezier curve of the 2nd half of swing tip trajectory calculation.
  /// @param[in] ground_contact Denotes if leg has made ground contact and swing trajectory towards ground should cease
  void generateSecondarySwingControlNodes(const bool& ground_contact = false);

  /// Generates control nodes for quartic bezier curve of stance tip trajectory calculation.
  /// @param[in] stride_scaler A scaling variable which modifies stride vector according to stance length specifically 
  /// for STARTING state of walker
  void generateStanceControlNodes(const double& stride_scaler);
  
  /// Updates control nodes for quartic bezier curves of both halves of swing tip trajectory calculation to force the 
  /// trajectory of the touchdown period of the swing period to be normal to the walk plane.
  void forceNormalTouchdown(void);

private:
  std::shared_ptr<WalkController> walker_;  ///< Pointer to walk controller object
  std::shared_ptr<Leg> leg_;                ///< Pointer to the parent leg object

  bool at_correct_phase_ = false;     ///< Flag denoting if the leg is at the correct phase per the walk state
  bool completed_first_step_ = false; ///< Flag denoting if the leg has completed its first step  
  bool touchdown_detection_ = false;  ///< Flag denoting whether touchdown detection is enabled

  int phase_ = 0;    ///< Step cycle phase
  int phase_offset_; ///< Step cycle phase offset

  double step_progress_ = 0.0;      ///< The progress of the entire step cycle (0.0->1.0 || -1.0)
  double swing_progress_ = -1.0;    ///< The progress of the swing period in the step cycle. (0.0->1.0 || -1.0)
  double stance_progress_ = -1.0;   ///< The progress of the stance period in the step cycle. (0.0->1.0 || -1.0)

  StepState step_state_ = STANCE; ///< The state of the step cycle

  Eigen::Vector3d swing_1_nodes_[5]; ///< An array of 3d control nodes defining the primary swing bezier curve
  Eigen::Vector3d swing_2_nodes_[5]; ///< An array of 3d control nodes defining the secondary swing bezier curve
  Eigen::Vector3d stance_nodes_[5];  ///< An array of 3d control nodes defining the stance bezier curve

  Eigen::Vector3d walk_plane_;        ///< A saved version of the estimated walk plane which is kept static during swing
  Eigen::Vector3d walk_plane_normal_; ///< The normal of the saved estimated planar walk surface
  Eigen::Vector3d stride_vector_;     ///< The desired stride vector
  Eigen::Vector3d swing_clearance_;   ///< The position relative to the default tip position to achieve during swing period

  double swing_delta_t_ = 0.0;
  double stance_delta_t_ = 0.0;

  Pose identity_tip_pose_;        ///< The user defined tip pose assuming a identity walk plane
  Pose default_tip_pose_;         ///< The default tip pose per the walk controller, updated with walk plane
  Pose current_tip_pose_;         ///< The current tip pose per the walk controller
  Pose origin_tip_pose_;          ///< The origin tip pose used in interpolation to target rotation
  Pose target_tip_pose_;          ///< The target tip pose to achieve at the end of a swing period

  ExternalTarget external_target_;  ///< The externally set target tip pose to achieve at the end of a swing period
  ExternalTarget external_default_; ///< The externally set default tip pose which defines default stance while at rest
  
  Eigen::Vector3d current_tip_velocity_;   ///< The default tip velocity per the walk controller
  
  Eigen::Vector3d swing_origin_tip_position_;  ///< The tip position used as the origin for the bezier curve during swing
  Eigen::Vector3d swing_origin_tip_velocity_;  ///< The tip velocity used in the generation of bezier curve during swing
  Eigen::Vector3d stance_origin_tip_position_; ///< The tip position used as the origin for the bezier curve during stance

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_WALK_CONTROLLER_H
