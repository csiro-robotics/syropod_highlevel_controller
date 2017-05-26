#ifndef SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H
/*******************************************************************************************************************//**
 *  \file    pose_controller.h
 *  \brief   Handles control of hexapod body posing. Part of simple hexapod controller.
 *
 *  \author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  \date    June 2017
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

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "quat.h"
#include "pose.h"

#include "model.h"
#include "walk_controller.h"

class AutoPoser;

/*******************************************************************************************************************//**
 * This struct contains data from IMU hardware transformed into the robot frame and in the format of the custom Quat 
 * class and Eigen Vector3d class.
***********************************************************************************************************************/
struct ImuData
{
  Quat orientation;
  Vector3d linear_acceleration;
  Vector3d angular_velocity;
};

/*******************************************************************************************************************//**
 * This class has two purposes. One is to manage functions which iteratively execute robot leg posing sequences, both 
 * via direct joint control, and via tip position control and inverse kinematics (IK). Such sequences include the 
 * packing and unpacking of legs into pre-defined joint positions (joint control) and direct movement of legs to target
 * tip positions (tip position control and IK). These functions make use of leg specific classes called Leg Posers
 * which handle the lower-level mechanisms used to move indivudal legs. 
 * The second purpose is to generate a pose to be applied to the body of the robot model. This pose is generated via
 * the combination of several seperate sub-poses which, in turn, are each individually generated according to seperate
 * requirements and methods. Generation of a user controlled 'manual' body pose and an IMU feedback based automatic
 * body pose are examples of these sub-poses, which are combined and applied to the robot model body.
***********************************************************************************************************************/
class PoseController
{
public:
  /**
    * PoseController class constructor. Iterates through legs in robot model and generates and assigns a leg pose 
    * object. Initialises member variables and calls function to initialise auto pose objects.
    * @param[in] model Pointer to the robot model class object
    * @param[in] params Pointer to the parameter struct object
    */
  PoseController(Model* model, Parameters* params);
  
  /** Accessor for pose reset mode */
  inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };
  
  /** Accessor for auto posing state*/
  inline PosingState getAutoPoseState(void) { return auto_posing_state_; };
  
  /** Accessor for imu data */
  inline ImuData getImuData(void) { return imu_data_; };
  
  /** Accessor for parameter object */
  inline Parameters* getParameters(void) { return params_; };
  
  /** Accessor for auto pose */
  inline Pose getAutoPose(void) { return auto_pose_; };
  
  /** Accessor for pose phase length */
  inline int getPhaseLength(void) { return pose_phase_length_; };
  
  /** Accessor for normaliser value */
  inline int getNormaliser(void) { return normaliser_; };
  
  /** Accessor for pose frequency */
  inline double getPoseFrequency(void) { return pose_frequency_; };
  
  /** Accessor for error in rotation absement - used in imu posing PID  */
  inline Vector3d getRotationAbsementError(void) { return rotation_absement_error_; };
  
  /** Accessor for error in rotation position - used in imu posing PID  */
  inline Vector3d getRotationPositionError(void) { return rotation_position_error_; };
  
  /** Accessor for error in rotation velocity - used in imu posing PID  */
  inline Vector3d getRotationVelocityError(void) { return rotation_velocity_error_; };
  
  /** Modifier for pose phase length */
  inline void setPhaseLength(int phase_length) { pose_phase_length_ = phase_length; };
  
  /** Modifier for normalisation value */
  inline void setNormaliser(int normaliser) { normaliser_ = normaliser; };
  
  /** Modifier for pose reset mode */
  inline void setPoseResetMode(PoseResetMode mode) { pose_reset_mode_ = mode; };
  
  /** Modifier for imu data */
  inline void setImuData(Quat orientation, Vector3d linear_acceleration, Vector3d angular_velocity)
  {
    imu_data_.orientation = orientation;
    imu_data_.linear_acceleration = linear_acceleration;
    imu_data_.angular_velocity = angular_velocity;
  }
  
  /** Modifier for manual pose velocity input */
  inline void setManualPoseInput(Vector3d translation, Vector3d rotation) 
  {
    translation_velocity_input_ = translation;
    rotation_velocity_input_ = rotation;
  }
  
  /** Resets all pose contributer variables to the identity pose */
  inline void resetAllPosing(void)
  {
    manual_pose_ = Pose::identity();
    auto_pose_ = Pose::identity();
    imu_pose_ = Pose::identity();
    inclination_pose_ = Pose::identity();
    impedance_pose_ = Pose::identity();
    default_pose_ = Pose::identity();
  }

  /**
   * Initialises auto poser container and populates with auto poser class objects as defined by auto poser parameters.
   * Also sets auto pose parameters for the leg poser object of each leg object in robot model.
   */
  void setAutoPoseParams(void);

  /**
   * Iterates through legs in robot model and updates each Leg Poser tip position. This new tip position is the tip 
   * position defined from the Leg Stepper, posed using the current desired pose. The applied pose is dependent on the
   * state of the Leg and Leg Poser specific auto posing.
   */
  void updateStance(void);
  
  /**
   * Executes saved transition sequence in direction defined by 'sequence' (START_UP or SHUT_DOWN) through the use of 
   * the function StepToPosition() to move to pre-defined tip positions for each leg in the robot model. If no sequence
   * exists for target stance, it generates one iteratively by checking workspace limitations.
   * @param[in] sequence The requested sequence - either START_UP or SHUT_DOWN
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   * @see parameters_and_states.h (for SequenceSelection definition)
   * @todo Make sequential leg stepping coordination an option instead of only simultaneous (direct) & groups (tripod)
   */
  int executeSequence(SequenceSelection sequence);

  /**
   * Iterates through legs in robot model and attempts to move them simultaneously in a linear trajectory directly from 
   * their current tip position to its default tip position (as defined by the walk controller). This motion completes 
   * in a time limit defined by the parameter time_to_start.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int directStartup(void);
  
  /**
   * Iterates through legs in robot model and attempts to step each from their current tip position to their default tip
   * position (as defined by the walk controller). The stepping motion is coordinated such that half of the legs execute 
   * the step at any one time (for a hexapod this results in a Tripod stepping coordination). The time period and 
   * height of the stepping maneuver is controlled by the user parameters step_frequency and step_clearance.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int stepToNewStance(void);
  
  /**
   * Iterates through the legs in the robot model and generates a pose for each that is best for leg manipulation. This 
   * pose is generated to attempt to move the centre of gravity within the support polygon of the load bearing legs. All 
   * legs simultaneously step to each new generated pose and the time period and height of the stepping maneuver is 
   * controlled by the user parameters step_frequency and step_clearance.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int poseForLegManipulation(void);
  
  /**
   * Iterate through legs in robot model and directly move joints into 'packed' configuration as defined by joint 
   * parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
   * @param[in] time_to_pack The time period in which to execute the packing maneuver.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int packLegs(double time_to_pack);
  
  /**
   * Iterate through legs in robot model and directly move joints into 'unpacked' configuration as defined by joint 
   * parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
   * @param[in] time_to_unpack The time period in which to execute the packing maneuver.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int unpackLegs(double time_to_unpack);

  /**
   * Depending on parameter flags, calls multiple posing functions and combines individual poses to update the current
   * desired pose of the robot model.
   * @param[in] body_height Desired height of the body above ground level - used in inclination posing.
   */
  void updateCurrentPose(double body_height);
  
  /**
   * Generates a manual pose to be applied to the robot model, based on linear (x/y/z) and angular (roll/pitch/yaw)
   * velocity body posing inputs. Clamps the posing within set limits and resets the pose to zero in specified axes 
   * depending on the pose reset mode.
   * @bug Adding pitch and roll simultaneously adds unwanted yaw - fixed by moving away from using quat.h
   */
  void updateManualPose(void);
  
  /**
   * Updates the auto pose by feeding each Auto Poser object a phase value and combining the output of each Auto Poser
   * object into a single pose. The input master phase is either an iteration of the pose phase or synced to the step
   * phase from the Walk Controller. This function also iterates through all leg objects in the robot model and updates
   * each Leg Poser's specific Auto Poser pose (this pose is used when the leg needs to ignore the default auto pose)
   * @bug Adding pitch and roll simultaneously adds unwanted yaw - fixed by moving away from using quat.h
   */
  void updateAutoPose(void);
  
  /**
   * Attempts to generate a pose (pitch/roll rotation only) for the robot model to 'correct' any differences between the
   * desired pose rotation and the that estimated by the IMU. A low pass filter is used to smooth out velocity inputs 
   * from the IMU and a basic PID controller is used to do control the output pose.
   */
  void updateIMUPose(void);
  
  /**
   * Attempts to generate a pose (x/y linear translation only) which shifts the assumed centre of gravity of the body to
   * the vertically projected centre of the support polygon in accordance with the inclination of the terrain.
   * @param[in] body_height The desired height of the body centre, vertically from the ground.
   */
  void updateInclinationPose(double body_height);
  
  /**
   * Attempts to generate a pose (z linear translation only) which corrects for sagging of the body due to the impedance
   * controller and poses the body at the correct desired height above the ground.
   */
  void updateImpedancePose(void);
  
  /**
   * Attempts to generate a pose (x/y linear translation only) to position body such that there is a zero sum of moments
   * from the force acting on the load bearing feet, allowing the robot to shift its centre of mass away from manually
   * manipulated (non-load bearing) legs and remain balanced.
   */
  void calculateDefaultPose(void);

private:
  Model* model_;       ///! Pointer to robot model object.
  ImuData imu_data_;   ///! Imu data structure.
  Parameters* params_; ///! Parameter data structure for storing parameter variables.
  
  PoseResetMode pose_reset_mode_;         ///! Mode for controlling which posing axes to reset to zero.
  Vector3d translation_velocity_input_;   ///! Velocity input for controlling the translation component of manual pose.
  Vector3d rotation_velocity_input_;      ///! Velocity input for controlling the rotational component of manual pose.
  
  int legs_completed_step_ = 0;           ///! Number of legs having completed the required step in a sequence.
  int current_group_ = 0;                 ///! The current leg group executing a stepping maneuver.
  
  bool recalculate_default_pose_ = true;  ///! Determines if the default pose needs to be recalculated.
  
  Pose manual_pose_;      ///! User controlled manual body pose, a component of total applied body pose.
  Pose auto_pose_;        ///! Cyclical custom automatic body pose, a component of total applied body pose.
  Pose imu_pose_;         ///! IMU feedback based automatic body pose, a component of total applied body pose.
  Pose inclination_pose_; ///! Pose to improve stability on inclined terrain, a component of total applied body pose.
  Pose impedance_pose_ ;  ///! Pose to correct impedance control based sagging, a component of total applied body pose.
  Pose default_pose_;     ///! Default pose calculated for different loading patterns
  
  int transition_step_ = 0;                     ///! The current transition step in the sequence being executed
  int transition_step_count_ = 0;               ///! The total number of transition steps in the sequence being executed
  bool set_target_ = true;                      ///! Flags if the new tip target is to be calculated and set
  bool proximity_alert_ = false;                ///! Flags if a joint has moved beyond the limit proximity buffer
  bool horizontal_transition_complete_ = false; ///! Flags if the horizontal transition has completed without error
  bool vertical_transition_complete_ = false;   ///! Flags if the vertical transition has completed without error
  bool first_sequence_execution_ = true;        ///! Flags if the controller has executed its first sequence
  bool reset_transition_sequence_ = true;       ///! Flags if the saved transition sequence needs to be regenerated  
  int legs_ready_for_direct_ = 0;               ///! Number of legs ready for the direct startup sequence
  
  vector<AutoPoser*> auto_poser_container_;         ///! Object containing all Auto Poser objects
  PosingState auto_posing_state_ = POSING_COMPLETE; ///! The state of auto posing
  int pose_phase_ = 0;                              ///! The current phase used in auto posing if not synced to walk
  double pose_frequency_ = 0.0;                     ///! The pose frequency used in determining auto pose phase length
  int pose_phase_length_ = 0;                       ///! The phase length of the auto posing cycle
  int normaliser_ = 1;                              ///! The value used to scale the base posing cycle start/end ratios
  
  Vector3d rotation_absement_error_;  ///! Difference between current and desired rotation absement for IMU posing PID
  Vector3d rotation_position_error_;  ///! Difference between current and desired rotation position for IMU posing PID
  Vector3d rotation_velocity_error_;  ///! Difference between current and desired rotation velocity for IMU posing PID
  
  map<int, Leg*>::iterator leg_it_;     ///! Leg iteration member variable used to minimise code
  map<int, Joint*>::iterator joint_it_; ///! Joint iteration member variable used to minimise code
};

/*******************************************************************************************************************//**
 * This class handles the custom auto posing cycles, which when combined create the sub-pose 'auto pose'. Each Auto 
 * Poser is not tied to an individual leg but instead is seperate and is defined by user parameters in the
 * auto_pose.yaml config file. Each Auto Poser has its own characteristic posing cycle and outputs a pose based on an
 * input phase which defines the position along this posing cycle.
 * @see config/auto_pose.yaml
***********************************************************************************************************************/
class AutoPoser
{
public:
  /**
   * Auto poser contructor.
   * @param[in] poser Pointer to the Pose Controller object
   * @param[in] id Int defining the id number of the created Auto Poser object
   */
  AutoPoser(PoseController* poser, int id);
  
  /** Accessor for id number of Auto Poser object */
  inline int getIDNumber(void) { return id_number_; };
  
  /** Returns true is Auto Poser is allowed to update its pose (i.e. has not completed posing) */
  inline bool isPosing(void) { return allow_posing_; };
  
  /** Modifier for pose cycle start phase */
  inline void setStartPhase(int start_phase) { start_phase_ = start_phase; };
  
  /** Modifier for pose cycle end  phase*/
  inline void setEndPhase(int end_phase) { end_phase_ = end_phase; };
  
  /** Modifier for amplitude of x axis component of linear translation in auto pose */
  inline void setXAmplitude(double x) { x_amplitude_ = x; };
  
  /** Modifier for amplitude of y axis component of linear translation in auto pose */
  inline void setYAmplitude(double y) { y_amplitude_ = y; };
  
  /** Modifier for amplitude of z axis component of linear translation in auto pose */
  inline void setZAmplitude(double z) { z_amplitude_ = z; };
  
  /** Modifier for amplitude of roll component of angular rotation in auto pose */
  inline void setRollAmplitude(double roll) { roll_amplitude_ = roll; };
  
  /** Modifier for amplitude of pitch component of angular rotation in auto pose */
  inline void setPitchAmplitude(double pitch) { pitch_amplitude_ = pitch; };
  
  /** Modifier for amplitude of yaw component of angular rotation in auto pose */
  inline void setYawAmplitude(double yaw) { yaw_amplitude_ = yaw; };
  
  /** Resets checks used for defining completion of auto posing cycle */
  inline void resetChecks(void) 
  { 
    start_check_ = false;
    end_check_ = pair<bool, bool>(false, false);
  }
  
  /**
   * Returns a pose which contributes to the auto pose applied to the robot body. The resultant pose is defined by a 4th
   * order bezier curve for both linear position and angular rotation and iterated along using the phase input argument.
   * The characteristics of each bezier curves are defined by the user parameters in the auto_pose.yaml config file.
   * @param[in] phase The phase is the input value which is used to determine the progression along the bezier curve 
   * which defines the output pose.
   * @return The component of auto pose contributed by this Auto Poser object's posing cycle defined by user parameters.
   * @see config/auto_pose.yaml
   */
  Pose updatePose(int phase);
  
private:
  PoseController* poser_;       ///! Pointer to pose controller object
  int id_number_;               ///! Identification number of Auto Poser object
  int start_phase_;             ///! Phase value denoting start of posing cycle
  int end_phase_;               ///! Phase value denoting end of posing cycle
  bool start_check_;            ///! Flag denoting if the posing cycle should start / has started
  pair<bool, bool> end_check_;  ///! Pair of flags which together denote is the posing cycle should end / has ended
  bool allow_posing_ = false;   ///! Flag denoting if the Auto Poser is allowed to continue updating the pose output
  
  double x_amplitude_;      ///! Amplitude of x axis component of linear translation in auto pose
  double y_amplitude_;      ///! Amplitude of y axis component of linear translation in auto pose
  double z_amplitude_;      ///! Amplitude of z axis component of linear translation in auto pose
  double roll_amplitude_;   ///! Amplitude of roll component of angular rotation in auto pose
  double pitch_amplitude_;  ///! Amplitude of pitch component of angular rotation in auto pose
  double yaw_amplitude_;    ///! Amplitude of yaw component of angular rotation in auto pose
};

/*******************************************************************************************************************//**
 * This class handles the lower-level mechanisms for moving a leg to defined target tip or joint positions and stores
 * several leg specific variables including the current tip position of the leg (according to the pose controller) and
 * saved transition target tip positions for generated robot motion sequences. This class also handles the negation of
 * applied auto posing according to leg specific auto pose negation cycles defined in auto_pose.yaml.
 * @see config/auto_pose.yaml
***********************************************************************************************************************/
class LegPoser
{
public:
  /**
   * Leg poser contructor.
   * @param[in] poser Pointer to the Pose Controller object.
   * @param[in] leg Pointer to the parent leg object associated with the create Leg Poser object.
   */
  LegPoser(PoseController* poser, Leg* leg);
  
  /** Accessor for current tip position according to the Leg Poser object. */
  inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
  
  /** Accessor for target tip position. */
  inline Vector3d getTargetTipPosition(void) { return target_tip_position_; };
  
  /** Accessor for auto pose. */
  inline Pose getAutoPose(void) { return auto_pose_; };
  
  /** Accessor for phase start of auto pose negation cycle. */
  inline int getPoseNegationPhaseStart() { return pose_negation_phase_start_; };
  
  /** Accessor for phase end of auto pose negation cycle. */
  inline int getPoseNegationPhaseEnd() { return pose_negation_phase_end_; };
  
  /** Returns true if leg has completed its required step in a sequence. */
  inline bool getLegCompletedStep(void) { return leg_completed_step_; };

  /** Modifier for current tip position according to the Leg Poser object. */
  inline void setCurrentTipPosition(Vector3d current) { current_tip_position_ = current; };
  
  /** Modifier for target tip position. */
  inline void setTargetTipPosition(Vector3d target) { target_tip_position_ = target; };
  
  /** Modifier for auto pose. */
  inline void setAutoPose(Pose auto_pose) { auto_pose_ = auto_pose; };
  
  /** Modifier for phase start of auto pose negation cycle. */
  inline void setPoseNegationPhaseStart(int start) { pose_negation_phase_start_ = start; };
  
  /** Modifier for phase end of auto pose negation cycle. */
  inline void setPoseNegationPhaseEnd(int end) { pose_negation_phase_end_ = end; };
  
  /** Modifier for the flag which denotes if leg has completed its required step in a sequence. */
  inline void setLegCompletedStep(bool complete) { leg_completed_step_ = complete; };
  
  /** Accessor to the transition tip position at the requested index. */
  inline Vector3d getTransitionPosition(int index) { return transition_positions_[index]; }
  
  /** Returns true if the transition position, of the requested index, exists. */
  inline bool hasTransitionPosition(int index) { return int(transition_positions_.size()) > index; };
  
  /** Adds tip position to vector of transition tip positions. */
  inline void addTransitionPosition(Vector3d transition) { transition_positions_.push_back(transition); };
  
  /** Clears all tip positions from the vector of transition tip positions. */
  inline void resetTransitionSequence(void) { transition_positions_.clear(); };
  
  /** Reset the key variables of stepToPosition() ready for new stepping maneuver. */
  inline int resetStepToPosition(void) 
  { 
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }

  /**
   * Uses a bezier curve to smoothly update (over many iterations) the desired joint position of each joint in the leg 
   * associated with this Leg Poser object, from the original joint position at the first iteration of this function to 
   * the target joint position defined by the input argument. This maneuver completes after a time period defined by the 
   * input argument.
   * @param[in] target_joint_positions A vector of doubles defining the target joint positions of each joint of the 
   * parent leg of this Leg Poser object in asscending joint id number order.
   * @param[in] time_to_move The time period in which to complete this maneuver.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int moveToJointPosition(vector<double> target_joint_positions, double time_to_move);
  
  /**
   * Uses bezier curves to smoothly update (over many iterations) the desired tip position of the leg associated with 
   * this Leg Poser object, from the original tip position at the first iteration of this function to the target tip 
   * position defined by the input argument.
   * @param[in] target_tip_position A 3d vector defining the target tip position in reference to the body centre frame
   * @param[in] target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
   * @param[in] lift_height The height which the stepping leg trajectory should reach at its peak.
   * @param[in] time_to_step The time period to complete this maneuver.
   * @param[in] apply_delta_z A bool defining if a vertical offset value (generated by the impedance controller) should 
   * be applied to the target tip position.
   * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
   */
  int stepToPosition(Vector3d target_tip_position, Pose target_pose,
                      double lift_height, double time_to_step, bool apply_delta_z = true);
  
  /**
   * Sets a pose for this Leg Poser which negates the default auto pose applied to the robot body. The negation pose is
   * defined by a 4th order bezier curve for both linear position and angular rotation and iterated along using the 
   * phase input argument. The characteristics of each bezier curve are defined by the user parameters in the
   * auto_pose.yaml config file.
   * @param[in] phase The phase is the input value which is used to determine the progression along the bezier curves
   * which define the output pose.
   * @see config/auto_pose.yaml
   */
  void updateAutoPose(int phase);

private:
  PoseController* poser_; ///! Pointer to pose controller object.
  Leg* leg_;              ///! Pointer to the parent leg object.

  Pose auto_pose_;                    ///! Leg specific auto pose (based off body auto pose negated where required).
  int pose_negation_phase_start_ = 0; ///! Phase start of auto pose negation cycle.
  int pose_negation_phase_end_ = 0;   ///! Phase end of auto pose negation cycle.
  bool stop_negation_ = false;        ///! Flag denoting if negation of auto pose is to stop.
  
  bool first_iteration_ = true;       ///! Flag denoting if an iterating function is on it's first iteration.
  int master_iteration_count_ = 0;    ///! Master iteration count used in generating time input for bezier curves.

  vector<double> origin_joint_positions_; ///! Vector containing joint positions of leg joints at the first iteration.

  Vector3d origin_tip_position_;  ///! Origin tip position used in bezier curve equations.
  Vector3d current_tip_position_; ///! Current tip position according to the pose controller.
  Vector3d target_tip_position_;  ///! Target tip position used in bezier curve equations.

  vector<Vector3d> transition_positions_; ///! Vector of tip positions to target during sequence transitions.

  bool leg_completed_step_ = false; ///! Flag denoting if leg has completed its required step in a sequence.
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H */
