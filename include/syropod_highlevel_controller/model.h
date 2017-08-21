#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
/*******************************************************************************************************************//**
 *  @file    model.h
 *  @brief   Describes the robot model including all legs, joints and links.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    August 2017
 *  @version 0.5.2
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
#include "syropod_highlevel_controller/LegState.h"

#define IK_TOLERANCE 0.005 ///< Tolerance between desired and resultant tip position from inverse/forward kinematics (m)
#define HALF_BODY_DEPTH 0.05 ///< Threshold used to estimate if leg tip has broken the plane of the robot body. (m)
#define DLS_COEFFICIENT 0.02 ///< Coefficient used in Damped Least Squares method for inverse kinematics.

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
typedef map<int, shared_ptr<Leg>> LegContainer;
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
};

/*******************************************************************************************************************//**
 * This class handles data for each 'leg' object of the parent robot model and contains functions which allow the
 * application of both forward and inverse kinematics. This class contains all child Joint, Link and Tip objects
 * associated with the leg.
***********************************************************************************************************************/
typedef vector<double> state_type; //Impedance state used in impedance controller
typedef map<int, shared_ptr<Joint>> JointContainer;
typedef map<int, shared_ptr<Link>> LinkContainer;
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

  /** Accessor for the current estimated vertical force on the tip of this leg. */
  inline double getTipForce(void) { return tip_force_; };

  /** Accessor for the current impedance control vertical position offset (delta_z) for this leg.*/
  inline double getDeltaZ(void) { return delta_z_; };

  /** Accessor for the virtual mass value used in the impedance control model of this leg.*/
  inline double getVirtualMass(void) { return virtual_mass_; };

  /** Accessor for the current virtual stiffness value used in the impedance control model of this leg. */
  inline double getVirtualStiffness(void) { return virtual_stiffness_; };

  /** Accessor for the virtual damping ratio value used in the impedance control model of this leg. */
  inline double getVirtualDampingRatio(void) { return virtual_damping_ratio_; };

  /** Accessor for the impedance state object for this leg. */
  inline state_type* getImpedanceState(void) { return &impedance_state_; };

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
  inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };

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
    * Modifier for the current estimated vertical force on the tip of this leg.
    * @param[in] tip_force The new vertical tip force estimate for this leg.
    */
  inline void setTipForce(const double& tip_force) { tip_force_ = tip_force; };

  /**
    * Modifier for the current impedance control vertical position offset (delta_z) for this leg.
    * @param[in] delta_z The new delta_z value for this leg.
    */
  inline void setDeltaZ(const double& delta_z) { delta_z_ = delta_z; };

  /**
    * Modifier for the virtual mass value used in the impedance control model of this leg.
    * @param[in] mass The new virtual mass value.
    */
  inline void setVirtualMass(const double& mass) { virtual_mass_ = mass; };

  /**
    * Modifier for the current virtual stiffness value used in the impedance control model of this leg.
    * @param[in] stiffness The new virtual stiffness value.
    */
  inline void setVirtualStiffness(const double& stiffness) { virtual_stiffness_ = stiffness; };

  /**
    * Modifier for the virtual damping ratio value used in the impedance control model of this leg.
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
   * @todo Refactor use of null pointer.
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
    * Sets desired tip position to the input, applying impedance controller vertical offset (delta z) if requested.
    * @param[in] tip_position The input desired tip position.
    * @param[in] apply_delta_z Flag denoting if 'delta_z' should be applied to desired tip position.
    */
  void setDesiredTipPosition(const Vector3d& tip_position, bool apply_delta_z = true);

  /**
    * Modifier for the desired tip velocity of the tip of this leg object.
    * @param[in] tip_velocity The new tip velocity of this leg object.
    */
  inline void setDesiredTipVelocity(const Vector3d& tip_velocity) { desired_tip_velocity_ = tip_velocity; };

  /**
    * Applies inverse kinematics to calculate required joint positions to achieve desired tip position. Inverse
    * kinematics is generated via the calculation of a jacobian for the current state of the leg, which is used as per
    * the Damped Least Squares method to generate a change in joint position for each joint. Returns a ratio of the
    * joint closest to position limits.
    * @param[in] simulation_run Flag denoting if this execution is for simulation purposes rather than normal use.
    * @param[in] ignore_tip_orientation Flag denoting if specific orientation of tip is desired or can be ignored
    * @todo Calculate optimal DLS coefficient (this value currently works sufficiently).
    */
  double applyIK(const bool& simulation_run = false, const bool& ignore_tip_orientation = true);

  /**
    * Updates joint transforms and applies forward kinematics to calculate a new tip position. Sets leg current tip
    * position to new position if requested.
    * @param[in] set_current Flag denoting of the calculated tip position should be set as the current tip position.
    */
  Vector3d applyFK(const bool& set_current = true);

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

  double delta_z_ = 0.0;         ///< The impedance controller vertical tip position offset value.
  double virtual_mass_;          ///< The virtual mass of the impedance controller virtual model of this leg.
  double virtual_stiffness_;     ///< The virtual stiffness of the impedance controller virtual model of this leg.
  double virtual_damping_ratio_; ///< The virtual damping ratio of the impedance controller virtual model of this leg.
  state_type impedance_state_;   ///< The impedance state of the impedance controller virtual model of this leg.

  Vector3d desired_tip_position_; ///< Desired tip position before applying Inverse/Forward kinematics
  Vector3d desired_tip_velocity_; ///< Desired tip velocity before applying Inverse/Forward kinematics
  Vector3d current_tip_position_; ///< Current tip position according to the model
  Vector3d current_tip_velocity_; ///< Current tip velocity according to the model
  double workspace_radius_;

  int group_; ///< Leg stepping coordination group (Either 0 or 1).

  double tip_force_ = 0.0; ///< Vertical force estimation on tip.

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
    * Returns the transformation matrix from the origin of the robot model to this joint.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    */
  inline Matrix4d getBaseTransform(const bool& zero = false) const
  {
    Matrix4d transform = zero ? identity_transform_ : current_transform_;
    return (id_number_ == 1) ? transform : reference_link_->actuating_joint_->getBaseTransform(zero) * transform;
  };

  /**
    * Returns the transformation matrix from the specified target joint of the robot model to this joint.
    * @param[in] target_joint Shared pointer to the Joint object defining the target joint for the transformation.
    */
  inline Matrix4d getTransformFrom(shared_ptr<Joint> target_joint) const
  {
    Matrix4d transform = current_transform_;
    bool at_target = (id_number_ <= 1 || (target_joint->id_number_ == reference_link_->actuating_joint_->id_number_));
    return at_target ? transform : reference_link_->actuating_joint_->getTransformFrom(target_joint) * transform;
  };

  /**
    * Returns the position of (or a position relative to) the origin of this joint in the frame of the robot model.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    * @param[in] joint_frame_position The position relative to the joint frame that is requested in the robot frame.
    */
  inline Vector3d getPositionRobotFrame(const bool& zero = false,
                                        const Vector3d& joint_frame_position = Vector3d(0, 0, 0)) const
  {
    Vector4d result(joint_frame_position[0], joint_frame_position[1], joint_frame_position[2], 1);
    result = getBaseTransform(zero) * result;
    return Vector3d(result[0], result[1], result[2]);
  };

  /**
    * Returns the position of (or a position relative to) the origin of the robot model in the frame of this joint.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    * @param[in] robot_frame_position The position relative to the robot frame that is requested in the joint frame.
    */
  inline Vector3d getPositionJointFrame(const bool& zero = false,
                                        const Vector3d& robot_frame_position = Vector3d(0, 0, 0)) const
  {
    MatrixXd transform = getBaseTransform(zero);
    Vector4d result(robot_frame_position[0], robot_frame_position[1], robot_frame_position[2], 1);
    result = transform.inverse() * result;
    return Vector3d(result[0], result[1], result[2]);
  };

  const shared_ptr<Leg> parent_leg_;      ///< A pointer to the parent leg object associated with this joint.
  const shared_ptr<Link> reference_link_; ///< A pointer to the reference Link object associated with this joint.
  const int id_number_;                   ///< The identification number for this joint.
  const string id_name_;                  ///< The identification name for this joint.
  Matrix4d current_transform_;            ///< The current transformation matrix between previous joint and this joint.
  Matrix4d identity_transform_;           ///< The identity transformation matrix between previous joint and this joint.

  ros::Publisher desired_position_publisher_; ///< The ros publisher for publishing desired position values.

  const double position_offset_;   ///< The offset between the zero positions of the robot model and actuating motor.
  const double min_position_;      ///< The minimum position allowed for this joint.
  const double max_position_;      ///< The maximum position allowed for this joint.
  const double packed_position_;   ///< The defined position of this joint in a 'packed' state.
  const double unpacked_position_; ///< The defined position of this joint in an 'unpacked' state.
  const double max_angular_speed_; ///< The maximum angular speed of this joint.

  double desired_position_ = 0.0;      ///< The desired angular position of this joint.
  double desired_velocity_ = 0.0;      ///< The desired angular velocity of this joint.
  double desired_effort_ = 0.0;        ///< The desired angular effort of this joint.
  double prev_desired_position_ = 0.0; ///< The desired angular position of this joint at the previous iteration.
  double prev_desired_velocity_ = 0.0; ///< The desired angular velocity of this joint at the previous iteration.
  double prev_desired_effort_ = 0.0;   ///< The desired angular effort of this joint at the previous iteration.

  double current_position_ = UNASSIGNED_VALUE; ///< The current position of this joint according to hardware.
  double current_velocity_ = UNASSIGNED_VALUE; ///< The current velocity of this joint according to hardware.
  double current_effort_ = UNASSIGNED_VALUE;   ///< The current effort of this joint according to hardware.

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
    * Returns the transformation matrix from the origin of the robot model to the tip.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    */
  inline Matrix4d getBaseTransform(const bool& zero = false) const
  {
    MatrixXd transform = zero ? identity_transform_ : current_transform_;
    return reference_link_->actuating_joint_->getBaseTransform() * transform;
  };

  /**
    * Returns the transformation matrix from the specified target joint of the robot model to the tip.
    * @param[in] target_joint Shared pointer to the Joint object defining the target joint for the transformation.
    */
  inline Matrix4d getTransformFrom(shared_ptr<Joint> target_joint) const
  {
    Matrix4d transform = current_transform_;
    bool at_target = (target_joint->id_number_ == reference_link_->actuating_joint_->id_number_);
    return at_target ? transform : reference_link_->actuating_joint_->getTransformFrom(target_joint) * transform;
  };

  /**
    * Returns the position of (or a position relative to) the origin of the tip in the frame of the robot model.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    * @param[in] tip_frame_position The position relative to the tip frame that is requested in the robot frame.
    */
  inline Vector3d getPositionRobotFrame(const bool& zero = false,
                                        const Vector3d& tip_frame_position = Vector3d(0, 0, 0)) const
  {
    Vector4d result(tip_frame_position[0], tip_frame_position[1], tip_frame_position[2], 1);
    result = getBaseTransform(zero) * result;
    return Vector3d(result[0], result[1], result[2]);
  };

  /**
    * Returns the position of (or a position relative to) the origin of the tip in the frame of the specified joint.
    * @param[in] tip_frame_position The position relative to the tip frame that is requested in the robot frame.
    * @param[in] target_joint Shared pointer to the Joint object defining the target frame for relative position.
    */
  inline Vector3d getPositionFromFrame(const Vector3d& tip_frame_position,
                                       shared_ptr<Joint> target_joint) const
  {
    Vector4d result(tip_frame_position[0], tip_frame_position[1], tip_frame_position[2], 1);
    result = getTransformFrom(target_joint) * result;
    return Vector3d(result[0], result[1], result[2]);
  };

  /**
    * Returns the position of (or a position relative to) the origin of the robot model in the frame of the tip.
    * @param[in] zero Flag determining if the transform is calculated using the current joint positions OR using the
    * zero position for all joints in the kinematic chain.
    * @param[in] robot_frame_position The position relative to the robot frame that is requested in the tip frame.
    */
  inline Vector3d getPositionTipFrame(const bool& zero = false,
                                      const Vector3d robot_frame_position = Vector3d(0, 0, 0)) const
  {
    MatrixXd transform = getBaseTransform(zero);
    Vector4d result(robot_frame_position[0], robot_frame_position[1], robot_frame_position[2], 1);
    result = transform.inverse() * result;
    return Vector3d(result[0], result[1], result[2]);
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
