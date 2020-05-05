////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_POSE_CONTROLLER_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_POSE_CONTROLLER_H

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "pose.h"
#include "model.h"
#include "walk_controller.h"

#define JOINT_TOLERANCE 0.01           ///< Tolerance allowing assumption that joints are in correct position (rad)
#define TIP_TOLERANCE 0.01             ///< Tolerance allowing assumption that tip is in correct position (m)
#define SAFETY_FACTOR 0.15             ///< Joint limit safety factor (i.e. during sequence joints will initially leave 15% buffer)
#define HORIZONTAL_TRANSITION_TIME 1.0 ///< Step time during horizontal transition (seconds @ step frequency == 1.0)
#define VERTICAL_TRANSITION_TIME 3.0   ///< Body raise time during vertical transtion (seconds @ step frequency == 1.0)
#define STABILITY_THRESHOLD 100        ///< Rotation correction magnitude threshold, ensuring imu posing PID is not unstable.
#define TRANSITION_STEP_THRESHOLD 20   ///< Number of allowed transition steps before executeSequence() deemed a failure
#define IMU_POSING_DEADBAND 0.0        ///< Rotation deadband for which imu posing assumes correct rotation (radians)

class AutoPoser;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class has two purposes. One is to manage functions which iteratively execute robot leg posing sequences, both
/// via direct joint control, and via tip position control and inverse kinematics (IK). Such sequences include the
/// packing and unpacking of legs into pre-defined joint positions (joint control) and direct movement of legs to target
/// tip positions (tip position control and IK). These functions make use of leg specific classes called Leg Posers
/// which handle the lower-level mechanisms used to move indivudal legs.
/// The second purpose is to generate a pose to be applied to the body of the robot model. This pose is generated via
/// the combination of several seperate sub-poses which, in turn, are each individually generated according to seperate
/// requirements and methods. Generation of a user controlled 'manual' body pose and an IMU feedback based automatic
/// body pose are examples of these sub-poses, which are combined and applied to the robot model body.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef Eigen::aligned_allocator<std::shared_ptr<AutoPoser>> AutoPoserAlignedAllocator;
typedef std::vector<std::shared_ptr<AutoPoser>, AutoPoserAlignedAllocator> AutoPoserContainer;
class PoseController : public std::enable_shared_from_this<PoseController>
{
public:
  /// PoseController class constructor. Initialises member variables.
  /// @param[in] model Pointer to the robot model class object
  /// @param[in] params Pointer to the parameter struct object
  PoseController(std::shared_ptr<Model> model, const Parameters &params);

  /// Accessor for pose reset mode.
  /// @return The pose reset mode (Mode for controlling which posing axes to reset to zero)
  inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };

  /// Accessor for auto posing state.
  /// @return The state of auto posing
  inline PosingState getAutoPoseState(void) { return auto_posing_state_; };

  /// Accessor for parameter object.
  /// @return Pointer to to parameter data structure for storing parameter variables
  inline const Parameters &getParameters(void) { return params_; };

  /// Accessor for auto pose.
  /// @return Cyclical custom automatic body pose, a component of total applied body pose
  inline Pose getAutoPose(void) { return auto_pose_; };

  /// Accessor for pose phase length.
  /// @return The phase length of the auto posing cycle
  inline int getPhaseLength(void) { return pose_phase_length_; };

  /// Accessor for normaliser value.
  /// @return The normaliser value (The value used to scale base posing cycle start/end ratios)
  inline int getNormaliser(void) { return normaliser_; };

  /// Accessor for pose frequency.
  /// @return The pose frequence (The frequency used in determining auto-pose phase length)
  inline double getPoseFrequency(void) { return pose_frequency_; };

  /// Accessor for error in rotation absement - used in imu posing PID.
  /// @return The error in rotation absement (Difference between current and desired rotation absement for
  /// IMU posing PID)
  inline Eigen::Vector3d getRotationAbsementError(void) { return rotation_absement_error_; };

  /// Accessor for error in rotation position - used in imu posing PID.
  /// @return The error in rotation position (Difference between current and desired rotation position for
  /// IMU posing PID)
  inline Eigen::Vector3d getRotationPositionError(void) { return rotation_position_error_; };

  /// Accessor for error in rotation velocity - used in imu posing PID.
  /// @return The error in rotation velocity (Difference between current and desired rotation velocity for
  /// IMU posing PID)
  inline Eigen::Vector3d getRotationVelocityError(void) { return rotation_velocity_error_; };

  /// Modifier for pose phase length.
  /// @param[in] phase_length The phase length to be set as the phase length of the auto posing cycle
  inline void setPhaseLength(const int &phase_length) { pose_phase_length_ = phase_length; };

  /// Modifier for normalisation value.
  /// @param[in] normaliser The value to be set as the normaliser value (The value used to scale base posing cycle
  /// start/end ratios)
  inline void setNormaliser(const int &normaliser) { normaliser_ = normaliser; };

  /// Modifier for pose reset mode.
  /// @param[in] mode The mode to be set as the pose reset mode (Mode for controlling which posing axes to reset
  /// to zero)
  inline void setPoseResetMode(const PoseResetMode &mode) { pose_reset_mode_ = mode; };

  /// Modifier for target body pose.
  /// @param[in] pose The pose to be set as the target body pose from planner to be transitioned to
  inline void setTargetBodyPose(const Pose &pose) { target_body_pose_ = pose; };

  /// Modifier for manual pose velocity input.
  /// @param[in] translation The value to be set as the velocity input for controlling the translation component of
  /// manual pose
  /// @param[in] rotation The value to be set as the velocity input for controlling the rotational component of
  /// manual pose
  inline void setManualPoseInput(const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation)
  {
    translation_velocity_input_ = translation;
    rotation_velocity_input_ = rotation;
  }

  /// Resets all pose contributer variables to the identity pose.
  inline void resetAllPosing(void)
  {
    manual_pose_ = Pose::Identity();
    auto_pose_ = Pose::Identity();
    imu_pose_ = Pose::Identity();
    inclination_pose_ = Pose::Identity();
    admittance_pose_ = Pose::Identity();
    default_pose_ = Pose::Identity();
    ik_error_pose_ = Pose::Identity();
    tip_align_pose_ = Pose::Identity();
    origin_tip_align_pose_ = tip_align_pose_;
    walk_plane_pose_ = Pose::Identity();
    origin_walk_plane_pose_ = walk_plane_pose_;
  }

  /// Modifier for desired configuration.
  /// @param[in] configuration The configuration to be set as the target robot configuration from planner to be
  /// transitioned to
  inline void setTargetConfiguration(const sensor_msgs::JointState &configuration)
  {
    target_configuration_ = configuration;
  }

  /// Iterates through legs in robot model and generates and assigns a leg poser object. Calls function to initialise
  /// auto pose objects. Seperated from constructor due to shared_from_this constraints.
  void init(void);

  /// Initialises auto poser container and populates with auto poser class objects as defined by auto poser parameters.
  /// Also sets auto pose parameters for the leg poser object of each leg object in robot model.
  void setAutoPoseParams(void);

  /// Iterates through legs in robot model and updates each Leg Poser tip position. This new tip position is the tip
  /// position defined from the Leg Stepper, posed using the current desired pose. The applied pose is dependent on the
  /// state of the Leg and Leg Poser specific auto posing.
  void updateStance(void);

  /// Executes saved transition sequence in direction defined by 'sequence' (START_UP or SHUT_DOWN) through the use of
  /// the function StepToPosition() to move to pre-defined tip positions for each leg in the robot model. If no sequence
  /// exists for target stance, it generates one iteratively by checking workspace limitations.
  /// @param[in] sequence The requested sequence - either START_UP or SHUT_DOWN
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  /// @see parameters_and_states.h (for SequenceSelection definition)
  /// @todo Make sequential leg stepping coordination an option instead of only simultaneous (direct) & groups (tripod)
  int executeSequence(const SequenceSelection &sequence);

  /// Iterates through legs in robot model and, in simulation, moves them in a linear trajectory directly from
  /// their current tip position to its default tip position (as defined by the walk controller). The joint states for
  /// each leg are saved for the deafult tip position and then the joint moved inpdependently from initial position to
  /// saved default positions. This motion completes in a time limit defined by the parameter time_to_start.
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int directStartup(void);

  /// Iterates through legs in robot model and attempts to step each from their current tip position to their default
  /// tip position (as defined by the walk controller). The stepping motion is coordinated such that half of the legs
  /// execute the step at any one time (for a hexapod this results in a Tripod stepping coordination). The time period
  /// and height of the stepping maneuver is controlled by the user parameters step_frequency and step_clearance.
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int stepToNewStance(void);

  /// Iterates through the legs in the robot model and generates a pose for each that is best for leg manipulation. This
  /// pose is generated to attempt to move the centre of gravity within the support polygon of the load bearing legs.
  /// All legs simultaneously step to each new generated pose and the time period and height of the stepping maneuver is
  /// controlled by the user parameters step_frequency and step_clearance
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int poseForLegManipulation(void);

  /// Iterate through legs in robot model and directly move joints into 'packed' configuration as defined by joint
  /// parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
  /// @param[in] time_to_pack The time period in which to execute the packing maneuver
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int packLegs(const double &time_to_pack);

  /// Iterate through legs in robot model and directly move joints into 'unpacked' configuration as defined by joint
  /// parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
  /// @param[in] time_to_unpack The time period in which to execute the packing maneuver
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int unpackLegs(const double &time_to_unpack);

  /// Iterate through legs in robot model and directly move joints to positions defined by desired configuration. This
  /// transition occurs simultaneously for all legs in a time period defined by the input argument.
  /// @param[in] transition_time The time period in which to execute the transition
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int transitionConfiguration(const double &transition_time);

  /// Iterate through legs in robot model and directly move tips to pose defined by target tip pose and target body
  /// pose. This transition occurs simultaneously for all legs in a time period defined by the input argument.
  /// @param[in] transition_time The time period in which to execute the transition
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int transitionStance(const double &transition_time);

  /// Depending on parameter flags, calls multiple posing functions and combines individual poses to update the current
  /// desired pose of the robot model.
  /// @param[in] robot_state The current state of the robot
  void updateCurrentPose(const RobotState &robot_state);

  /// Generates a manual pose to be applied to the robot model, based on linear (x/y/z) and angular (roll/pitch/yaw)
  /// velocity body posing inputs. Clamps the posing within set limits and resets the pose to zero in specified axes
  /// depending on the pose reset mode.
  void updateManualPose(void);

  /// Poses the body of the robot according to errors in IK for each leg. Ideally, moves legs into configuration
  /// to achieve desired tip positions which cannot be achieved otherwise.
  /// @todo Improve method for returning ik error pose to zero
  void updateIKErrorPose(void);

  /// Updates a body pose that, when applied, orients the last joint of a swinging leg inline with the tip along the
  /// walk plane normal. This causes the last link of the leg to be oriented orthogonal to the walk plane estimate
  /// during the 2nd half of swing. This is used to orient tip sensors to point toward the desired tip landing position
  /// at the end of the swing.
  void updateTipAlignPose(void);

  /// Calculates a pose for the robot body such that the robot body is parallel to a calculated walk plane at a normal
  /// offset of the body clearance parameter. The optimal average walking plane is calculated from tip positions of legs
  /// in stance.
  void updateWalkPlanePose(void);

  /// Updates the auto pose by feeding each Auto Poser object a phase value and combining the output of each Auto Poser
  /// object into a single pose. The input master phase is either an iteration of the pose phase or synced to the step
  /// phase from the Walk Controller. This function also iterates through all leg objects in the robot model and updates
  /// each Leg Poser's specific Auto Poser pose (this pose is used when the leg needs to ignore the default auto pose)
  void updateAutoPose(void);

  /// Attempts to generate a pose (pitch/roll rotation only) for the robot model to 'correct' any differences between
  /// the desired pose rotation and the that estimated by the IMU. A low pass filter is used to smooth out velocity
  /// inputs from the IMU and a basic PID controller is used to do control the output pose.
  void updateIMUPose(void);

  /// Attempts to generate a pose (x/y linear translation only) which shifts the assumed centre of gravity of the body
  /// to the vertically projected centre of the support polygon in accordance with the inclination of the terrain.
  void updateInclinationPose(void);

  /// Estimates the acceleration vector due to gravity.
  /// @return The estimated acceleration vector due to gravity.
  inline Eigen::Vector3d estimateGravity(void) { return model_->estimateGravity(); };

  /// Attempts to generate a pose (x/y linear translation only) to position body such that there is a zero sum of
  /// moments from the force acting on the load bearing feet, allowing the robot to shift its centre of mass away from
  /// manually manipulated (non-load bearing) legs and remain balanced.
  void calculateDefaultPose(void);

private:
  std::shared_ptr<Model> model_; ///< Pointer to robot model object
  const Parameters &params_;     ///< Pointer to parameter data structure for storing parameter variables

  std::shared_ptr<Leg> auto_pose_reference_leg_; ///< Reference leg for auto posing system

  PoseResetMode pose_reset_mode_;              ///< Mode for controlling which posing axes to reset to zero
  Eigen::Vector3d translation_velocity_input_; ///< Velocity input for controlling translation component of manual pose
  Eigen::Vector3d rotation_velocity_input_;    ///< Velocity input for controlling rotational component of manual pose

  int legs_completed_step_ = 0; ///< Number of legs having completed the required step in a sequence
  int current_group_ = 0;       ///< The current leg group executing a stepping maneuver

  bool recalculate_default_pose_ = true; ///< Determines if the default pose needs to be recalculated

  Pose manual_pose_;      ///< User controlled manual body pose, a component of total applied body pose
  Pose auto_pose_;        ///< Cyclical custom automatic body pose, a component of total applied body pose
  Pose imu_pose_;         ///< IMU feedback based automatic body pose, a component of total applied body pose
  Pose inclination_pose_; ///< Pose to improve stability on inclined terrain, a component of total applied body pose
  Pose admittance_pose_;  ///< Pose to correct admittance control based sagging, a component of total applied body pose
  Pose default_pose_;     ///< Default pose calculated for different loading patterns
  Pose ik_error_pose_;    ///< Pose used in correcting errors in inverse kinematics for individual legs

  Pose tip_align_pose_;         ///< Pose used to align final links of legs vertically during 2nd half of swing
  Pose origin_tip_align_pose_;  ///< Origin pose used in interpolating tip align pose
  Pose walk_plane_pose_;        ///< Pose used to align robot body parallel with walk plane and normal at clearance
  Pose origin_walk_plane_pose_; ///< Origin pose used in interpolating walk plane pose

  sensor_msgs::JointState target_configuration_; ///< Target robot configuration from planner to be transitioned to
  Pose target_body_pose_;                        ///< Target body pose from planner to be transitioned to

  bool executing_transition_ = false; ///< Flag denoting if the pose controller is executing a transition

  int transition_step_ = 0;                     ///< The current transition step in the sequence being executed
  int transition_step_count_ = 0;               ///< Total number of transition steps in the sequence being executed
  int pack_step_ = 0;                           ///< The current step in pack/unpack sequence
  bool set_target_ = true;                      ///< Flags if the new tip target is to be calculated and set
  bool proximity_alert_ = false;                ///< Flags if a joint has moved beyond the limit proximity buffer
  bool horizontal_transition_complete_ = false; ///< Flags if the horizontal transition has completed without error
  bool vertical_transition_complete_ = false;   ///< Flags if the vertical transition has completed without error
  bool first_sequence_execution_ = true;        ///< Flags if the controller has executed its first sequence
  bool reset_transition_sequence_ = true;       ///< Flags if the saved transition sequence needs to be regenerated

  std::vector<double> default_joint_positions_[8]; ///< Joint positions for default stance used in Direct Startup

  AutoPoserContainer auto_poser_container_;         ///< Object containing all Auto Poser objects
  PosingState auto_posing_state_ = POSING_COMPLETE; ///< The state of auto posing
  int pose_phase_ = 0;                              ///< The current phase used in auto posing if not synced to walk
  double pose_frequency_ = 0.0;                     ///< The frequency used in determining auto-pose phase length
  int pose_phase_length_ = 0;                       ///< The phase length of the auto posing cycle
  int normaliser_ = 1;                              ///< The value used to scale base posing cycle start/end ratios

  Eigen::Vector3d rotation_absement_error_; ///< Difference btw. current & desired rotation absement for IMU posing PID
  Eigen::Vector3d rotation_position_error_; ///< Difference btw. current & desired rotation position for IMU posing PID
  Eigen::Vector3d rotation_velocity_error_; ///< Difference btw. current & desired rotation velocity for IMU posing PID

  LegContainer::iterator leg_it_;     ///< Leg iteration member variable used to minimise code
  JointContainer::iterator joint_it_; ///< Joint iteration member variable used to minimise code

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles the custom auto posing cycles, which when combined create the sub-pose 'auto pose'. Each Auto
/// Poser is not tied to an individual leg but instead is seperate and is defined by user parameters in the
/// auto_pose.yaml config file. Each Auto Poser has its own characteristic posing cycle and outputs a pose based on an
/// input phase which defines the position along this posing cycle.
/// @see config/auto_pose.yaml
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class AutoPoser
{
public:
  /// Auto poser contructor.
  /// @param[in] poser Pointer to the Pose Controller object.
  /// @param[in] id Int defining the id number of the created Auto Poser object.
  AutoPoser(std::shared_ptr<PoseController> poser, const int &id);

  /// Accessor for id number of Auto Poser object.
  /// @return The identification number of the Auto Poser object
  inline int getIDNumber(void) { return id_number_; };

  /// Returns true is Auto Poser is allowed to update its pose (i.e. has not completed posing).
  /// @return Flag denoting whether Auto Poser is allowed to update its pose (i.e. has not completed posing)
  inline bool isPosing(void) { return allow_posing_; };

  /// Modifier for pose cycle start phase.
  /// @param[in] start_phase The value to be set as the phase value denoting start of posing cycle
  inline void setStartPhase(const int &start_phase) { start_phase_ = start_phase; };

  /// Modifier for pose cycle end phase.
  /// @param[in] end_phase The value to be set as the phase value denoting end of posing cycle
  inline void setEndPhase(const int &end_phase) { end_phase_ = end_phase; };

  /// Modifier for amplitude of x axis component of linear translation in auto pose.
  /// @param[in] x The value to be set as the amplitude of x axis component of linear translation in auto pose
  inline void setXAmplitude(const double &x) { x_amplitude_ = x; };

  /// Modifier for amplitude of y axis component of linear translation in auto pose.
  /// @param[in] y The value to be set as the amplitude of y axis component of linear translation in auto pose
  inline void setYAmplitude(const double &y) { y_amplitude_ = y; };

  /// Modifier for amplitude of z axis component of linear translation in auto pose.
  /// @param[in] z The value to be set as the amplitude of z axis component of linear translation in auto pose
  inline void setZAmplitude(const double &z) { z_amplitude_ = z; };

  /// Modifier for amplitude of component linear translation in auto pose in direction of gravity.
  /// @param[in] g The value to be set as the amplitude of component linear translation in auto pose in direction
  /// of gravity
  inline void setGravityAmplitude(const double &g) { gravity_amplitude_ = g; };

  /// Modifier for amplitude of roll component of angular rotation in auto pose.
  /// @param[in] roll The value to be set as the amplitude of roll component of angular rotation in auto pose
  inline void setRollAmplitude(const double &roll) { roll_amplitude_ = roll; };

  /// Modifier for amplitude of pitch component of angular rotation in auto pose.
  /// @param[in] pitch The value to be set as the amplitude of pitch component of angular rotation in auto pose
  inline void setPitchAmplitude(const double &pitch) { pitch_amplitude_ = pitch; };

  /// Modifier for amplitude of yaw component of angular rotation in auto pose.
  /// @param[in] yaw The value to be set as the amplitude of yaw component of angular rotation in auto pose
  inline void setYawAmplitude(const double &yaw) { yaw_amplitude_ = yaw; };

  /// Resets checks used for defining completion of auto posing cycle.
  inline void resetChecks(void)
  {
    start_check_ = false;
    end_check_ = std::pair<bool, bool>(false, false);
  }

  /// Returns a pose which contributes to the auto pose applied to the robot body. The resultant pose is defined by a
  /// 4th order bezier curve for both linear position and angular rotation and iterated along using the phase input
  /// argument. The characteristics of each bezier curves are defined by the user parameters in the auto_pose.yaml
  /// config file.
  /// @param[in] phase The phase is the input value which is used to determine the progression along the bezier curve
  /// which defines the output pose
  /// @return The component of auto pose contributed by this Auto Poser object's posing cycle defined by user parameters
  /// @see config/auto_pose.yaml
  Pose updatePose(int phase);

private:
  std::shared_ptr<PoseController> poser_; ///< Pointer to pose controller object
  int id_number_;                         ///< Identification number of Auto Poser object
  int start_phase_;                       ///< Phase value denoting start of posing cycle
  int end_phase_;                         ///< Phase value denoting end of posing cycle
  bool start_check_;                      ///< Flag denoting if the posing cycle should start/has started
  std::pair<bool, bool> end_check_;       ///< Pair of flags which together denote if posing cycle should end/has ended
  bool allow_posing_ = false;             ///< Flag denoting if Auto Poser is allowed to continue updating pose output

  double x_amplitude_;       ///< Amplitude of x axis component of linear translation in auto pose
  double y_amplitude_;       ///< Amplitude of y axis component of linear translation in auto pose
  double z_amplitude_;       ///< Amplitude of z axis component of linear translation in auto pose
  double gravity_amplitude_; ///< Amplitude of the linear translation in auto pose in the direction of gravity
  double roll_amplitude_;    ///< Amplitude of roll component of angular rotation in auto pose
  double pitch_amplitude_;   ///< Amplitude of pitch component of angular rotation in auto pose
  double yaw_amplitude_;     ///< Amplitude of yaw component of angular rotation in auto pose

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles the lower-level mechanisms for moving a leg to defined target tip or joint positions and stores
/// several leg specific variables including the current tip position of the leg (according to the pose controller) and
/// saved transition target tip positions for generated robot motion sequences. This class also handles the negation of
/// applied auto posing according to leg specific auto pose negation cycles defined in auto_pose.yaml.
/// @see config/auto_pose.yaml
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class LegPoser
{
public:
  /// Leg poser contructor.
  /// @param[in] poser Pointer to the Pose Controller object
  /// @param[in] leg Pointer to the parent leg object associated with the create Leg Poser object
  LegPoser(std::shared_ptr<PoseController> poser, std::shared_ptr<Leg> leg);

  /// Leg poser copy contructor.
  /// @param[in] leg_poser Pointer to the Leg Poser object to be copied from
  LegPoser(std::shared_ptr<LegPoser> leg_poser);

  /// Accessor for pointer to parent leg object.
  /// @return Pointer to parent leg object
  inline std::shared_ptr<Leg> getParentLeg(void) { return leg_; };

  /// Accessor for current tip pose according to the Leg Poser object.
  /// @return The current tip pose according to the Leg Poser object
  inline Pose getCurrentTipPose(void) { return current_tip_pose_; };

  /// Accessor for target tip pose.
  /// @return The target tip pose
  inline Pose getTargetTipPose(void) { return target_tip_pose_; };

  /// Accessor for externally set target tip pose.
  /// @return The externally set target tip pose
  inline ExternalTarget getExternalTarget(void) { return external_target_; };

  /// Accessor for auto pose.
  /// @return Leg specific auto pose
  inline Pose getAutoPose(void) { return auto_pose_; };

  /// Accessor for phase start of auto pose negation cycle.
  /// @return Phase start of auto pose negation cycle
  inline int getPoseNegationPhaseStart(void) { return pose_negation_phase_start_; };

  /// Accessor for phase end of auto pose negation cycle.
  /// @return Phase end of auto pose negation cycle
  inline int getPoseNegationPhaseEnd(void) { return pose_negation_phase_end_; };

  /// Returns true if leg has completed its required step in a sequence.
  /// @return Flag denoting whether the leg has completed its required step in a sequence
  inline bool getLegCompletedStep(void) { return leg_completed_step_; };

  /// Modifier for the pointer to the parent leg.
  /// @param[in] parent_leg Pointer to be set as the pointer to the parent leg
  inline void setParentLeg(std::shared_ptr<Leg> parent_leg) { leg_ = parent_leg; };

  /// Modifier for current tip pose according to the Leg Poser object.
  /// @param[in] current Pose to be set as the current tip pose according to the Leg Poser object
  inline void setCurrentTipPose(const Pose &current) { current_tip_pose_ = current; };

  /// Modifier for target tip pose.
  /// @param[in] target Pose to be set as the target tip pose
  inline void setTargetTipPose(const Pose &target) { target_tip_pose_ = target; };

  /// Modifier for externally set target tip pose.
  /// @param[in] target Pose to be set as the externally set target tip pose
  inline void setExternalTarget(const ExternalTarget &target) { external_target_ = target; };

  /// Modifier for auto pose.
  /// @param[in] auto_pose Pose to be set as the leg specific auto pose
  inline void setAutoPose(const Pose &auto_pose) { auto_pose_ = auto_pose; };

  /// Modifier for phase start of auto pose negation cycle.
  /// @param[in] start The value to be set as the phase start of auto pose negation cycle
  inline void setPoseNegationPhaseStart(const int &start) { pose_negation_phase_start_ = start; };

  /// Modifier for phase end of auto pose negation cycle.
  /// @param[in] end The value to be set as the phase end of auto pose negation cycle
  inline void setPoseNegationPhaseEnd(const int &end) { pose_negation_phase_end_ = end; };

  /// Modifier for ratio of negation cycle for transition.
  /// @param[in] ratio The value to be set as the ratio of negation cycle for transition
  inline void setNegationTransitionRatio(const double &ratio) { negation_transition_ratio_ = ratio; };

  /// Modifier for the flag which denotes if leg has completed its required step in a sequence.
  /// @param[in] complete The boolean value to be set as the flag which denotes if the leg has completed its required
  /// step in a sequence
  inline void setLegCompletedStep(const bool &complete) { leg_completed_step_ = complete; };

  /// Modifier for the desired leg configuration.
  /// @param[in] config The configuration to be set as the desired leg configuration
  inline void setDesiredConfiguration(const sensor_msgs::JointState &config) { desired_configuration_ = config; };

  /// Accessor to the transition tip poses at the requested index.
  /// @param[in] index The index of the required transition tip pose
  /// @return The transition tip pose of the requested index
  inline Pose getTransitionPose(const int &index) { return transition_poses_[index]; }

  /// Returns true if the transition pose, of the requested index, exists.
  /// @param[in] index The index of the transition pose for checking existence
  /// @return Flag denoting whether the transition pose of the requested index exists
  inline bool hasTransitionPose(const int &index) { return int(transition_poses_.size()) > index; };

  /// Adds tip position to vector of transition tip poses.
  /// @param[in] transition Transition pose to be added to the vector of transition tip poses
  inline void addTransitionPose(const Pose &transition) { transition_poses_.push_back(transition); };

  /// Clears all tip poses from the vector of transition tip poses.
  inline void resetTransitionSequence(void) { transition_poses_.clear(); };

  /// Reset the key variables of stepToPosition() ready for new stepping maneuver.
  /// @return 100 if resetting is successful
  inline int resetStepToPosition(void)
  {
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }

  /// Uses a bezier curve to smoothly update (over many iterations) the desired joint position of each joint in the leg
  /// associated with this Leg Poser object, from the original configuration at the first iteration of this function to
  /// the target configuration defined by the pre-set member variable. This transition completes after a time period
  /// defined by the input argument.
  /// @param[in] transition_time The time period in which to complete this transition
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int transitionConfiguration(const double &transition_time);

  /// Uses bezier curves to smoothly update (over many iterations) the desired tip position of the leg associated with
  /// this Leg Poser object, from the original tip position at the first iteration of this function to the target tip
  /// position defined by the input argument.
  /// @param[in] target_tip_pose The target tip pose in reference to the body centre frame
  /// @param[in] target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
  /// @param[in] lift_height The height which the stepping leg trajectory should reach at its peak
  /// @param[in] time_to_step The time period to complete this maneuver
  /// @param[in] apply_delta A bool defining if a position offset value (generated by the admittance controller) should
  /// be applied to the target tip position
  /// @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
  int stepToPosition(const Pose &target_tip_pose, const Pose &target_pose,
                     const double &lift_height, const double &time_to_step, const bool &apply_delta = true);

  /// Sets the leg specific auto pose from the default auto pose defined by auto pose parameters. The leg specific auto
  /// pose may be negated according to user defined parameters. The negated pose is defined by interpolating from
  /// the default auto pose to identity pose (zero pose) and back again over the negation period. The ratio of the
  /// period which is used to interpolate to/from is defined by the negation transition ratio parameter.
  /// @param[in] phase The phase is the input value which is used to determine the progression along the bezier curves
  /// which define the output pose
  void updateAutoPose(const int &phase);

private:
  std::shared_ptr<PoseController> poser_;   ///< Pointer to pose controller object
  std::shared_ptr<Leg> leg_;                ///< Pointer to the parent leg object

  Pose auto_pose_;                          ///< Leg specific auto pose (from default auto pose - negated if required)
  bool negate_auto_pose_ = false;           ///< Flag denoting if negation of auto pose is to occur
  int pose_negation_phase_start_ = 0;       ///< Phase start of auto pose negation cycle
  int pose_negation_phase_end_ = 0;         ///< Phase end of auto pose negation cycle
  double negation_transition_ratio_ = 0.0;  ///< The ratio of the negation period used to transition to total negation

  bool first_iteration_ = true;             ///< Flag denoting if an iterating function is on it's first iteration
  int master_iteration_count_ = 0;          ///< Master iteration count used in generating time input for bezier curves

  sensor_msgs::JointState desired_configuration_;  ///< Configuration target for transitionConfiguration function
  sensor_msgs::JointState origin_configuration_;   ///< Configuration origin for transitionConfiguration function

  Pose origin_tip_pose_;                    ///< Origin tip pose used in bezier curve equations
  Pose current_tip_pose_;                   ///< Current tip pose according to the pose controller
  Pose target_tip_pose_;                    ///< Target tip pose used in bezier curve equations
  ExternalTarget external_target_;          ///< Externally set target tip pose object

  std::vector<Pose, Eigen::aligned_allocator<Pose>> transition_poses_; ///< Vector of transition target tip poses

  bool leg_completed_step_ = false; ///< Flag denoting if leg has completed its required step in a sequence

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_POSE_CONTROLLER_H
