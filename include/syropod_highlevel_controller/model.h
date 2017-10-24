#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
/*******************************************************************************************************************//**
 *  @file    model.h
 *  @brief   Describes the robot model including all legs, joints and links.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    October 2017
 *  @version 0.5.7
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
#include "pose.h"
#include "syropod_highlevel_controller/LegState.h"

#define IK_TOLERANCE 0.005 ///< Tolerance between desired and resultant tip position from inverse/forward kinematics (m)
#define HALF_BODY_DEPTH 0.05 ///< Threshold used to estimate if leg tip has broken the plane of the robot body. (m)
#define DLS_COEFFICIENT 0.02 ///< Coefficient used in Damped Least Squares method for inverse kinematics.
#define JOINT_LIMIT_COST_WEIGHT 0.1 ///< Gain used in determining cost weight for joints approaching limits

class Leg;
class Joint;
class Link;
class Tip;

class WalkController;
class LegStepper;
class PoseController;
class LegPoser;

/*******************************************************************************************************************//**
 * This class serves as the top-level parent of each leg object and associated tip/joint/link objects. It contains data
 * which is relevant to the robot body or the robot as a whole rather than leg dependent data.
***********************************************************************************************************************/
typedef map<int, shared_ptr<Leg>, less<int>, aligned_allocator<pair<const int, shared_ptr<Leg>>>> LegContainer;
class Model : public enable_shared_from_this<Model>
{
public:
  /**
    * Contructor for robot model object - initialises member variables from parameters.
    * @param[in] params A pointer to the parameter data structure.
    */
  Model(const Parameters& params);

  /** Accessor for leg object container.*/
  inline LegContainer* getLegContainer(void) { return &leg_container_; };

  /** Accessor for leg count (number of legs in robot model).*/
  inline int getLegCount(void) { return leg_count_; };

  /** Accessor for current pose of the robot model body.*/
  inline Pose getCurrentPose(void) { return current_pose_; };

  /** Accessor for the time delta value which defines the period of the ros cycle.*/
  inline double getTimeDelta(void) { return time_delta_; }

  /**
   * Modifier for the current pose of the robot model body.
   * @param[in] pose The input pose to be set as the current robot model body pose.
   */
  inline void setCurrentPose(const Pose& pose)  { current_pose_ = pose; };

  /**
   * Generates child leg objects. Separated from constructor due to shared_from_this() constraints.
   */
  void generate(void);

  /**
   * Iterate through legs in robot model and have them run their initialisation.
   * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position
   * values for any joint with unknown current position values.
   */
  void initLegs(const bool& use_default_joint_positions);
  
  /**
   * Sets tip contact range offset used in calibrating the input range data such that values are zeroed when tip in
   * contact with walk surface. Function is run assuming all tips are in contact with walk surface.
   */
  void initTipRanges(void);

  /**
   * Estimates if the robot is bearing its load on its legs. Estimates the average distance between body and leg tips
   * and checks if the average breaks the plane of the robot body underside, if so, assume that at least one leg is
   * bearing load.
   * @todo Make more robust by estimating body height above the ground.
   * @todo Parameterise HALF_BODY_DEPTH
   */
  bool legsBearingLoad(void);

  /**
   * Returns pointer to leg requested via identification number input.
   * @param[in] leg_id_num The identification number of the requested leg object pointer.
   */
  inline shared_ptr<Leg> getLegByIDNumber(const int& leg_id_num) { return leg_container_[leg_id_num]; };

  /**
   * Returns pointer to leg requsted via identification name string input.
   * @param[in] leg_id_name The identification name of the requested leg object pointer.
   */
  shared_ptr<Leg> getLegByIDName(const string& leg_id_name);

private:
  const Parameters& params_;     ///< Pointer to parameter data structure for storing parameter variables.
  LegContainer leg_container_;   ///< The container map for all robot model leg objects.
  int leg_count_;                ///< The number of leg objects within the robot model.
  double time_delta_;            ///< The time period of the ros cycle.
  Pose current_pose_;            ///< Current pose of robot model body.
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This class handles data for each 'leg' object of the parent robot model and contains functions which allow the
 * application of both forward and inverse kinematics. This class contains all child Joint, Link and Tip objects
 * associated with the leg.
***********************************************************************************************************************/
typedef vector<double> state_type; //Impedance state used in admittance controller
typedef map<int, shared_ptr<Joint>, less<int>, aligned_allocator<pair<const int, shared_ptr<Joint>>>> JointContainer;
typedef map<int, shared_ptr<Link>, less<int>, aligned_allocator<pair<const int, shared_ptr<Link>>>> LinkContainer;
class Leg : public enable_shared_from_this<Leg>
{
public:
  /**
    * Constructor for a robot model leg object. Initialises member variables from parameters.
    * @param[in] model A pointer to the parent robot model.
    * @param[in] id_number An identification number for this leg object.
    * @param[in] params A pointer to the parameter data structure.
    */
  Leg(shared_ptr<Model> model, const int& id_number, const Parameters& params);

  /** Accessor for identification name of this leg object. */
  inline string getIDName(void) { return id_name_; };

  /** Accessor for identification number of this leg object. */
  inline int getIDNumber(void) { return id_number_; };

  /** Accessor for the number of child joint objects for this leg. */
  inline int getJointCount(void) { return joint_count_; };

  /** Accessor for the step coordination group of this leg. */
  inline int getGroup(void) { return group_; };

  /** Accessor for the cuurent state of this leg. */
  inline LegState getLegState(void) { return leg_state_; };

  /** Accessor for the current estimated force vector on the tip of this leg. */
  inline Vector3d getTipForce(void) { return tip_force_; };
  
  /** Accessor for the current estimated torque vector on the tip of this leg */
  inline Vector3d getTipTorque(void) { return tip_torque_; };
  
  /** Accessor for the current estimated distance from tip to walk surface contact. */
  inline double getTipContactRange(void) { return tip_contact_range_; };
  
  /** Accessor for the calibration offset for the distance from tip to walk surface contact. */
  inline double getTipContactOffset(void) { return tip_contact_offset_; };

  /** Accessor for the current admittance control vertical position offset (delta_z) for this leg.*/
  inline double getDeltaZ(void) { return delta_z_; };

  /** Accessor for the virtual mass value used in the admittance control model of this leg.*/
  inline double getVirtualMass(void) { return virtual_mass_; };

  /** Accessor for the current virtual stiffness value used in the admittance control model of this leg. */
  inline double getVirtualStiffness(void) { return virtual_stiffness_; };

  /** Accessor for the virtual damping ratio value used in the admittance control model of this leg. */
  inline double getVirtualDampingRatio(void) { return virtual_damping_ratio_; };

  /** Accessor for the admittance state object for this leg. */
  inline state_type* getAdmittanceState(void) { return &admittance_state_; };

  /** Accessor for the container of Joint objects associated with this leg. */
  inline JointContainer* getJointContainer(void) { return &joint_container_; };

  /** Accessor for the container of Link objects associated with this leg. */
  inline LinkContainer* getLinkContainer(void) { return &link_container_; };

  /** Accessor for the Tip object associated with this leg. */
  inline shared_ptr<Tip> getTip(void) { return tip_; };

  /** Accessor for the LegStepper object associated with this leg. */
  inline shared_ptr<LegStepper> getLegStepper(void) { return leg_stepper_; };

  /** Accessor for the LegPoser object associated with this leg. */
  inline shared_ptr<LegPoser> getLegPoser(void) { return leg_poser_; };
  
  /** Accessor for the current tip position of this leg. */
  inline Vector3d getDesiredTipPosition(void) { return desired_tip_pose_.position_; };
  
  /** Accessor for the current tip rotation of this leg. */
  inline Quaterniond getDesiredTipRotation(void) { return desired_tip_pose_.rotation_; };

  /** Accessor for the current tip velocity of this leg. */
  inline Vector3d getDesiredTipVelocity(void) { return desired_tip_velocity_; };

  /** Accessor for the current tip position of this leg. */
  inline Vector3d getCurrentTipPosition(void) { return current_tip_pose_.position_; };
  
  /** Accessor for the current tip rotation of this leg. */
  inline Quaterniond getCurrentTipRotation(void) { return current_tip_pose_.rotation_; };

  /** Accessor for the current tip velocity of this leg. */
  inline Vector3d getCurrentTipVelocity(void) { return current_tip_velocity_; };

  /**
    * Modifier for the curent state of this leg.
    * @param[in] leg_state The new state of this leg.
    */
  inline void setLegState(const LegState& leg_state) { leg_state_ = leg_state; };

  /**
    * Modifier for the publisher of leg state messages.
    * @param[in] publisher The new ros publisher to publish leg state messages.
    */
  inline void setStatePublisher(const ros::Publisher& publisher) { leg_state_publisher_ = publisher; };

  /**
    * Modifier for the publisher of ASC state messages.
    * @param[in] publisher The new ros publisher to publish ASC state messages.
    */
  inline void setASCStatePublisher(const ros::Publisher& publisher) { asc_leg_state_publisher_ = publisher; };

  /**
    * Modifier for the LegStepper object associated with this leg.
    * @param[in] leg_stepper A pointer to the new LegStepper object for this leg.
    */
  inline void setLegStepper(shared_ptr<LegStepper> leg_stepper) { leg_stepper_ = leg_stepper; };

  /**
    * Modifier for the LegPoser object associated with this leg.
    * @param[in] leg_poser A pointer to the new LegPoser object for this leg.
    */
  inline void setLegPoser(shared_ptr<LegPoser> leg_poser) { leg_poser_ = leg_poser; };

  /**
    * Modifier for the current estimated force vector on the tip of this leg.
    * @param[in] tip_force The new tip force estimate for this leg.
    */
  inline void setTipForce(const Vector3d& tip_force) { tip_force_ = tip_force; };
  
  /**
    * Modifier for the current estimated torque vector on the tip of this leg.
    * @param[in] tip_torque The new tip torque estimate for this leg.
    */
  inline void setTipTorque(const Vector3d& tip_torque) { tip_torque_ = tip_torque; };
  
  /**
    * Modifier for the current estimated distance from tip to walk surface contact.
    * @param[in] contact_range The new contact range estimate for this leg.
    */
  inline void setTipContactRange(const double& contact_range) { tip_contact_range_ = contact_range; };
  
  /**
    * Modifier for the calibration offset for the current estimated distance from tip to walk surface contact.
    * @param[in] offset The new contact range offset for this leg.
    */
  inline void setTipContactOffset(const double& offset) { tip_contact_offset_ = offset; };
  
  /**
    * Modifier for the current admittance control vertical position offset (delta_z) for this leg.
    * @param[in] delta_z The new delta_z value for this leg.
    */
  inline void setDeltaZ(const double& delta_z) { delta_z_ = delta_z; };

  /**
    * Modifier for the virtual mass value used in the admittance control model of this leg.
    * @param[in] mass The new virtual mass value.
    */
  inline void setVirtualMass(const double& mass) { virtual_mass_ = mass; };

  /**
    * Modifier for the current virtual stiffness value used in the admittance control model of this leg.
    * @param[in] stiffness The new virtual stiffness value.
    */
  inline void setVirtualStiffness(const double& stiffness) { virtual_stiffness_ = stiffness; };

  /**
    * Modifier for the virtual damping ratio value used in the admittance control model of this leg.
    * @param[in] damping_ratio The new virtual damping ratio value.
    */
  inline void setVirtualDampingRatio(const double& damping_ratio) { virtual_damping_ratio_ = damping_ratio; };

  /**
    * Publishes the given message via the leg state pubisher object.
    * @param[in] msg The leg state message to be published.
    */
  inline void publishState(const syropod_highlevel_controller::LegState& msg) { leg_state_publisher_.publish(msg); };

  /**
    * Publishes the given message via the ASC leg state pubisher object.
    * @param[in] msg The ASC leg state message to be published.
    */
  inline void publishASCState(const std_msgs::Bool& msg) { asc_leg_state_publisher_.publish(msg); };

  /**
   * Generates child joint/link/tip objects. Separated from constructor due to shared_from_this() constraints.
   */
  void generate(void);

  /**
    * Initialises leg object by setting desired joint state to default values or to current position (from encoders)
    * and running forward kinematics for tip position.
    * @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position
    * values for any joint with unknown current position values.
    */
  void init(const bool& use_default_joint_positions);

  /**
    * Re-Initialises leg object by setting desired joint state to values from JointState message input and running
    * forward kinematics for tip position.
    * @param[in] desired_joint_states JointState message containing desired joint states values for leg object.
    */
  void reInit(const sensor_msgs::JointState& desired_joint_states);

  /**
    * Generates a JointState message from the desired state of the joints of the leg object.
    * @param[out] joint_state_msg The output JointState mesage to fill with the state of joints within this leg object.
    */
  void generateDesiredJointStateMsg(sensor_msgs::JointState* joint_state_msg);

  /**
    * Returns pointer to joint requested via identification number input.
    * @param[in] joint_id_number The identification name of the requested joint object pointer.
    */
  inline shared_ptr<Joint> getJointByIDNumber(const int& joint_id_number) { return joint_container_[joint_id_number]; };

  /**
    * Returns pointer to joint requested via identification name string input.
    * @param[in] joint_id_name The identification name of the requested joint object pointer.
    */
  shared_ptr<Joint> getJointByIDName(const string& joint_id_name);

  /**
    * Returns pointer to link requested via identification number input.
    * @param[in] link_id_number The identification number of the requested link object pointer.
    */
  inline shared_ptr<Link> getLinkByIDNumber(const int& link_id_number) { return link_container_[link_id_number]; };

  /**
    * Returns pointer to link requested via identification name string input.
    * @param[in] link_id_name The identification name of the requested link object pointer.
    */
  shared_ptr<Link> getLinkByIDName(const string& link_id_name);

  /**
    * Sets desired tip position to the input, applying admittance controller vertical offset (delta z) if requested.
    * @param[in] tip_position The input desired tip position.
    * @param[in] apply_delta_z Flag denoting if 'delta_z' should be applied to desired tip position.
    */
  void setDesiredTipPosition(const Vector3d& tip_position, bool apply_delta_z = true);
  
  /**
    * Modifier for the desired tip rotation of this leg.
    * @param[in] tip_rotation The desired rotation of the tip.
    */
  inline void setDesiredTipRotation(const Quaterniond& tip_rotation) { desired_tip_pose_.rotation_ = tip_rotation; };

  /**
    * Modifier for the desired tip velocity of the tip of this leg object.
    * @param[in] tip_velocity The new tip velocity of this leg object.
    */
  inline void setDesiredTipVelocity(const Vector3d& tip_velocity) { desired_tip_velocity_ = tip_velocity; };
  
  inline void setOriginTipRotation(const Quaterniond& tip_rotation) { origin_tip_rotation_ = tip_rotation; };
  inline Quaterniond getOriginTipRotation(void) { return origin_tip_rotation_; };
  
  /**
    * Calculates an estimate for the tip force vector acting on this leg, using the calculated state jacobian and 
    * values for the torque on each joint in the leg.
    */
  void calculateTipForce(void);

  /**
    * Applies inverse kinematics to calculate required joint positions to achieve desired tip position. Inverse
    * kinematics is generated via the calculation of a jacobian for the current state of the leg, which is used as per
    * the Damped Least Squares method to generate a change in joint position for each joint. Returns a ratio of the
    * joint closest to position limits.
    * @param[in] simulation_run Flag denoting if this execution is for simulation purposes rather than normal use.
    * @param[in] ignore_tip_orientation Flag denoting if specific orientation of tip is desired or can be ignored
    * @todo Calculate optimal DLS coefficient (this value currently works sufficiently).
    * @todo Modify return value to output zero on IK failure to achieve desired tip position
    */
  double applyIK(const bool& simulation_run = false, const bool& ignore_tip_orientation = true);

  /**
    * Updates joint transforms and applies forward kinematics to calculate a new tip pose. 
    * Sets leg current tip pose to new pose if requested.
    * @param[in] set_current Flag denoting of the calculated tip pose should be set as the current tip pose.
    */
  Pose applyFK(const bool& set_current = true);

  /**
    * Calls jocobian creation function for requested degrees of freedom.
    * @param[in] dh A vector containing a map of DH parameter strings and values for each degree of freedom.
    */
  MatrixXd createJacobian(const vector<map<string, double>>& dh);

private:
  shared_ptr<Model> model_;        ///< A pointer to the parent robot model object.
  const Parameters& params_;       ///< Pointer to parameter data structure for storing parameter variables.
  JointContainer joint_container_; ///< The container object for all child Joint objects.
  LinkContainer link_container_;   ///< The container object for all child Link objects.
  shared_ptr<Tip> tip_;            ///< A pointer to the child Tip object.

  shared_ptr<LegStepper> leg_stepper_;  ///< A pointer to the LegStepper object associated with this leg.
  shared_ptr<LegPoser> leg_poser_;      ///< A pointer to the LegPoser object associated with this leg.

  const int id_number_;         ///< The identification number for this leg.
  const std::string id_name_;   ///< The identification name for this leg.
  const int joint_count_;       ///< The number of child Joint objects associated with this leg.
  LegState leg_state_;          ///< The current state of this leg.

  ros::Publisher leg_state_publisher_;     ///< The ros publisher object that publishes state messages for this leg.
  ros::Publisher asc_leg_state_publisher_; ///< The ros publisher object that publishes ASC state messages for this leg.

  double delta_z_ = 0.0;         ///< The admittance controller vertical tip position offset value.
  double virtual_mass_;          ///< The virtual mass of the admittance controller virtual model of this leg.
  double virtual_stiffness_;     ///< The virtual stiffness of the admittance controller virtual model of this leg.
  double virtual_damping_ratio_; ///< The virtual damping ratio of the admittance controller virtual model of this leg.
  state_type admittance_state_;  ///< The admittance state of the admittance controller virtual model of this leg.

  Pose desired_tip_pose_; ///< Desired tip pose before applying Inverse/Forward kinematics
  Pose current_tip_pose_; ///< Current tip pose according to the model
  
  Vector3d desired_tip_velocity_; ///< Desired linear tip velocity before applying Inverse/Forward kinematics
  Vector3d current_tip_velocity_; ///< Current linear tip velocity according to the model
  
  Quaterniond origin_tip_rotation_;

  int group_; ///< Leg stepping coordination group (Either 0 or 1).

  Vector3d tip_force_;        ///< Force estimation on the tip.
  Vector3d tip_torque_;       ///< Torque estimation on the tip.
  double tip_contact_range_;  ///< Estimation of the distance from the tip to contacting the walking surface.
  double tip_contact_offset_; ///< Calibration offset set once assumed all legs are on walking surface.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This class handles data for each 'link' of a parent leg object. This data includes the DH parameters which define
 * the transformation between the actuating joint at the beginning of this link (in the kinematic chain) and the next
 * joint for which this link is the reference.
***********************************************************************************************************************/
class Link
{
public:
  /**
    * Constructor for Link object. Initialises member variables from parameters.
    * @param[in] leg A pointer to the parent leg object.
    * @param[in] actuating_joint A pointer to the actuating joint object, from which this link is moved.
    * @param[in] id_number The identification number for this link.
    * @param[in] params A pointer to the parameter data structure.
    */
  Link(shared_ptr<Leg> leg, shared_ptr<Joint> actuating_joint, const int& id_number, const Parameters& params);

  const shared_ptr<Leg> parent_leg_;        ///< A pointer to the parent leg object associated with this link.
  const shared_ptr<Joint> actuating_joint_; ///< A pointer to the actuating Joint object associated with this link.
  const int id_number_;                     ///< The identification number for this link.
  const string id_name_;                    ///< The identification name for this link.
  const double dh_parameter_r_;             ///< The DH parameter 'r' associated with this link.
  const double dh_parameter_theta_;         ///< The DH parameter 'theta' associated with this link.
  const double dh_parameter_d_;             ///< The DH parameter 'd' associated with this link.
  const double dh_parameter_alpha_;         ///< The DH parameter 'alpha' associated with this link.
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This class handles data for each 'joint' of a parent leg object and contains functions which allow the transformation
 * of positions between the robot frame and the joint frame along the kinematic chain.
***********************************************************************************************************************/
class Joint
{
public:
  /**
    * Constructor for Joint object. Initialises member variables from parameters and generates initial transform.
    * @param[in] leg A pointer to the parent leg object.
    * @param[in] reference_link A pointer to the reference link object, from which this joint actuates.
    * @param[in] id_number The identification number for this joint.
    * @param[in] params A pointer to the parameter data structure.
    */
  Joint(shared_ptr<Leg> leg, shared_ptr<Link> reference_link, const int& id_number, const Parameters& params);
  
  /**
   * Constructor for null joint object. Acts as a null joint object for use in ending kinematic chains.
   */
  Joint(void);

  /**
    * Returns the transformation matrix from the specified target joint of the robot model to this joint. 
    * Target joint defaults to the origin of the kinematic chain.
    * @param[in] target_joint_id ID number of joint object defining the target joint for the transformation.
    * @return The transformation matrix from target joint to this joint.
    */
  inline Matrix4d getTransformFromJoint(const int& target_joint_id = 0) const
  {
    shared_ptr<Joint> next_joint = reference_link_->actuating_joint_;
    bool at_target = (target_joint_id == next_joint->id_number_);
    return at_target ? current_transform_ : next_joint->getTransformFromJoint(target_joint_id) * current_transform_;
  };

  /**
    * Returns the pose of (or a pose relative to) the origin of this joint in the frame of the robot model.
    * @param[in] joint_frame_pose The pose relative to the joint frame that is requested in the robot frame.
    * @return The input pose transformed into the robot frame.
    */
  inline Pose getPoseRobotFrame(const Pose& joint_frame_pose = Pose::identity()) const
  {
    Matrix4d transform = getTransformFromJoint();
    return joint_frame_pose.transform(transform);
  };

  /**
    * Returns the pose of (or a pose relative to) the origin of the robot model in the frame of this joint.
    * @param[in] robot_frame_pose The position relative to the robot frame that is requested in the joint frame.
    * @return The input pose transformed into the frame of this joint.
    */
  inline Pose getPoseJointFrame(const Pose& robot_frame_pose = Pose::identity()) const
  {
    MatrixXd transform = getTransformFromJoint();
    return robot_frame_pose.transform(transform.inverse());
  };

  const shared_ptr<Leg> parent_leg_;      ///< A pointer to the parent leg object associated with this joint.
  const shared_ptr<Link> reference_link_; ///< A pointer to the reference Link object associated with this joint.
  const int id_number_;                   ///< The identification number for this joint.
  const string id_name_;                  ///< The identification name for this joint.
  Matrix4d current_transform_;            ///< The current transformation matrix between previous joint and this joint.
  Matrix4d identity_transform_;           ///< The identity transformation matrix between previous joint and this joint.

  ros::Publisher desired_position_publisher_; ///< The ros publisher for publishing desired position values.

  const double min_position_ = 0.0;      ///< The minimum position allowed for this joint.
  const double max_position_ = 0.0;      ///< The maximum position allowed for this joint.
  const double packed_position_ = 0.0;   ///< The defined position of this joint in a 'packed' state.
  const double unpacked_position_ = 0.0; ///< The defined position of this joint in an 'unpacked' state.
  const double max_angular_speed_ = 0.0; ///< The maximum angular speed of this joint.

  double desired_position_ = 0.0;      ///< The desired angular position of this joint.
  double desired_velocity_ = 0.0;      ///< The desired angular velocity of this joint.
  double desired_effort_ = 0.0;        ///< The desired angular effort of this joint.
  double prev_desired_position_ = 0.0; ///< The desired angular position of this joint at the previous iteration.
  double prev_desired_velocity_ = 0.0; ///< The desired angular velocity of this joint at the previous iteration.
  double prev_desired_effort_ = 0.0;   ///< The desired angular effort of this joint at the previous iteration.

  double current_position_ = UNASSIGNED_VALUE; ///< The current position of this joint according to hardware.
  double current_velocity_ = 0.0;              ///< The current velocity of this joint according to hardware.
  double current_effort_ = 0.0;                ///< The current effort of this joint according to hardware.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*******************************************************************************************************************//**
 * This class handles data for the 'tip' of a parent leg object and contains functions which allow the transformation
 * of positions between the robot frame and the tip frame along the kinematic chain.
***********************************************************************************************************************/
class Tip
{
public:
  /**
    * Constructor for Tip object. Initialises member variables from parameters and generates initial transform.
    * @param[in] leg A pointer to the parent leg object.
    * @param[in] reference_link A pointer to the reference link object, which is attached to this tip object.
    */
  Tip(shared_ptr<Leg> leg, shared_ptr<Link> reference_link);

  /**
    * Returns the transformation matrix from the specified target joint  of the robot model to the tip. 
    * Target joint defaults to the origin of the kinematic chain.
    * @param[in] target_joint_id ID number of joint object defining the target joint for the transformation.
    * @return The transformation matrix from target joint to the tip.
    */
  inline Matrix4d getTransformFromJoint(const int& target_joint_id = 0) const
  {
    shared_ptr<Joint> next_joint = reference_link_->actuating_joint_;
    bool at_target = (target_joint_id == next_joint->id_number_);
    return at_target ? current_transform_ : next_joint->getTransformFromJoint(target_joint_id) * current_transform_;
  };

  /**
    * Returns the pose of (or a pose relative to) the origin of the tip in the frame of the robot model.
    * @param[in] tip_frame_pose The pose relative to the tip frame that is requested in the robot frame.
    * @return The input pose transformed into the robot frame.
    */
  inline Pose getPoseRobotFrame(const Pose& tip_frame_pose = Pose::identity()) const
  {
    Matrix4d transform = getTransformFromJoint();
    return tip_frame_pose.transform(transform);
  };
  
  /**
  * Returns the pose of (or a pose relative to) the origin of the robot model in the frame of the tip.
  * @param[in] robot_frame_pose The pose relative to the robot frame that is requested in the tip frame.
  * @return The input pose transformed into the tip frame.
  */
  inline Pose getPoseTipFrame(const Pose& robot_frame_pose = Pose::identity()) const
  {
    Matrix4d transform = getTransformFromJoint();
    return robot_frame_pose.transform(transform.inverse());
  };

  const shared_ptr<Leg> parent_leg_;       ///< A pointer to the parent leg object associated with the tip.
  const shared_ptr<Link> reference_link_;  ///< A pointer to the reference Link object associated with the tip.
  const string id_name_;                   ///< The identification name for the tip.
  Matrix4d current_transform_;             ///< The current transformation matrix between previous joint and the tip.
  Matrix4d identity_transform_;            ///< The identity transformation matrix between previous joint and the tip.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H */
