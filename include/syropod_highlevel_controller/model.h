////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "pose.h"
#include "syropod_highlevel_controller/LegState.h"

#define IK_TOLERANCE 0.005 ///< Tolerance between desired and resultant tip position from inverse/forward kinematics (m)
#define HALF_BODY_DEPTH 0.05 ///< Threshold used to estimate if leg tip has broken the plane of the robot body (m)
#define DLS_COEFFICIENT 0.02          ///< Coefficient used in Damped Least Squares method for inverse kinematics
#define JOINT_LIMIT_COST_WEIGHT 0.1   ///< Gain used in determining cost weight for joints approaching limits

#define BEARING_STEP 45          ///< Step to increment bearing in workspace generation algorithm (deg)
#define MAX_POSITION_DELTA 0.002 ///< Position delta to increment search position in workspace generation algorithm (m)
#define MAX_WORKSPACE_RADIUS 1.0 ///< Maximum radius allowed in workspace polygedron plane (m)
#define WORKSPACE_LAYERS 10      ///< Number of planes in workspace polyhedron

class Leg;
class Joint;
class Link;
class Tip;

class WalkController;
class LegStepper;
class PoseController;
class LegPoser;

class DebugVisualiser;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This struct contains data from IMU hardware.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct ImuData
{
public:
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class serves as the top-level parent of each leg object and associated tip/joint/link objects. It contains data
/// which is relevant to the robot body or the robot as a whole rather than leg dependent data.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef std::map<int, std::shared_ptr<Leg>, std::less<int>, Eigen::aligned_allocator<std::pair<const int, std::shared_ptr<Leg>>>> LegContainer;
class Model : public std::enable_shared_from_this<Model>
{
public:
  /// Contructor for robot model object - initialises member variables from parameters.
  /// @param[in] params A pointer to the parameter data structure
  /// @param[in] debug_visualiser A pointer to debug visualiser object
  Model(const Parameters& params, std::shared_ptr<DebugVisualiser> debug_visualiser);
  
  /// Copy Constructor for a robot model object. Initialises member variables from existing Model object.
  /// @param[in] model A pointer to a existing reference robot model object
  Model(std::shared_ptr<Model> model);

  /// Accessor for leg object container.
  /// @return Pointer to leg container object
  inline LegContainer* getLegContainer(void) { return &leg_container_; };
  
  /// Accessor for debug visualiser pointer.
  /// @return Pointer to debug visualiser object
  inline std::shared_ptr<DebugVisualiser> getDebugVisualiser(void) { return debug_visualiser_; };

  /// Accessor for leg count (number of legs in robot model).
  /// @return Number of legs in the robot model
  inline int getLegCount(void) { return leg_count_; };

  /// Accessor for current pose of the robot model body.
  /// @return The current pose of the robot model body
  inline Pose getCurrentPose(void) { return current_pose_; };
  
  /// Accessor for default pose of the robot model body.
  /// @return The default pose of the robot model body
  inline Pose getDefaultPose(void) { return default_pose_; };

  /// Accessor for the time delta value which defines the period of the ros cycle.
  /// @return The time delta value which define the period of the ros cycle
  inline double getTimeDelta(void) { return time_delta_; }

  /// Modifier for the current pose of the robot model body.
  /// @param[in] pose The input pose to be set as the current robot model body pose
  inline void setCurrentPose(const Pose& pose)  { current_pose_ = pose; };
  
  /// Modifier for the default pose of the robot model body.
  /// @param[in] pose The input pose to be set as the default robot model body pose
  inline void setDefaultPose(const Pose& pose)  { default_pose_ = pose; };

  /// Generates child leg objects and copies state from reference model if provided.
  /// Separated from constructor due to shared_from_this() constraints.
  /// @param[in] model A pointer to a existing reference robot model object
  void generate(std::shared_ptr<Model> model = NULL);

  /// Iterate through legs in robot model and have them run their initialisation.
  /// @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position
  /// values for any joint with unknown current position values
  void initLegs(const bool& use_default_joint_positions);

  /// Estimates if the robot is bearing its load on its legs. Estimates the average distance between body and leg tips
  /// and checks if the average breaks the plane of the robot body underside, if so, assume that at least one leg is
  /// bearing load.
  /// @return Flag denoting whether the robot is bearing its load on its legs
  /// @todo Make more robust by estimating body height above the ground
  /// @todo Parameterise HALF_BODY_DEPTH
  bool legsBearingLoad(void);

  /// Returns pointer to leg requested via identification number input.
  /// @param[in] leg_id_num The identification number of the requested leg object pointer
  /// @return The Pointer to leg requested via identification number input
  inline std::shared_ptr<Leg> getLegByIDNumber(const int& leg_id_num) { return leg_container_[leg_id_num]; };

  /// Returns pointer to leg requsted via identification name string input.
  /// @param[in] leg_id_name The identification name of the requested leg object pointer
  /// @return The pointer to leg requested via identification name input
  std::shared_ptr<Leg> getLegByIDName(const std::string& leg_id_name);
  
  /// Accessor for imu data.
  /// @return The imu data structure of the robot model
  inline ImuData getImuData(void) 
  { 
    ImuData imu_data(imu_data_);
    if (imu_data_.orientation.isApprox(UNDEFINED_ROTATION))
    {
      imu_data.orientation = Eigen::Quaterniond::Identity();
    }
    return imu_data;
  };
  
  /// Modifier for imu data.
  /// @param[in] orientation The orientation to be set as the orientation of the imu
  /// @param[in] linear_acceleration The linear acceleration to be set as the linear acceleration of the imu
  /// @param[in] angular_velocity The angular velocity to be set as the angular velocity of the imu
  inline void setImuData(const Eigen::Quaterniond& orientation, 
                         const Eigen::Vector3d& linear_acceleration, 
                         const Eigen::Vector3d& angular_velocity)
  {
    imu_data_.orientation = orientation.normalized();
    imu_data_.linear_acceleration = linear_acceleration;
    imu_data_.angular_velocity = angular_velocity;
  }
  
  /// Updates joint default positions for each leg according to current joint positions of each leg.
  void updateDefaultConfiguration(void);
  
  /// Generates workspace polyhedron for each leg in model.
  void generateWorkspaces(void);
  
  /// Updates model configuration by applying inverse kinematics to solve desired tip poses generated from walk/pose
  /// controllers.
  void updateModel(void);
  
  /// Estimates the acceleration vector due to gravity from pitch and roll orientations from IMU data
  /// @return The estimated acceleration vector due to gravity.
  Eigen::Vector3d estimateGravity(void);

private:
  const Parameters& params_;                     ///< Pointer to parameter structure for storing parameter variables
  std::shared_ptr<DebugVisualiser> debug_visualiser_; ///< Pointer to debug visualiser object
  LegContainer leg_container_;                   ///< The container map for all robot model leg objects
  
  int leg_count_;                ///< The number of leg objects within the robot model
  double time_delta_;            ///< The time period of the ros cycle
  Pose current_pose_;            ///< Current pose of robot model body (i.e. walk_plane -> base_link)
  Pose default_pose_;            ///< Default pose of robot model body (i.e. only body clearance above walk plane)
  ImuData imu_data_;             ///< Imu data structure
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles data for each 'leg' object of the parent robot model and contains functions which allow the
/// application of both forward and inverse kinematics. This class contains all child Joint, Link and Tip objects
/// associated with the leg.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef std::vector<double> state_type; // Impedance state used in admittance controller
typedef std::map<int, double> Workplane;
typedef std::map<double, Workplane> Workspace;
typedef std::map<int, std::shared_ptr<Joint>, std::less<int>, Eigen::aligned_allocator<std::pair<const int, std::shared_ptr<Joint>>>> JointContainer;
typedef std::map<int, std::shared_ptr<Link>, std::less<int>, Eigen::aligned_allocator<std::pair<const int, std::shared_ptr<Link>>>> LinkContainer;
class Leg : public std::enable_shared_from_this<Leg>
{
public:
  /// Constructor for a robot model leg object. Initialises member variables from parameters.
  /// @param[in] model A pointer to the parent robot model
  /// @param[in] id_number An identification number for this leg object
  /// @param[in] params A pointer to the parameter data structure
  Leg(std::shared_ptr<Model> model, const int& id_number, const Parameters& params);
  
  /// Copy Constructor for a robot model leg object. Initialises member variables from existing Leg object.
  /// @param[in] leg A pointer to the parent robot model
  /// @param[in] model A pointer to the parent model of the leg
  Leg(std::shared_ptr<Leg> leg, std::shared_ptr<Model> model = NULL);

  /// Accessor for identification name of this leg object.
  /// @return The identification name of the leg object
  inline std::string getIDName(void) { return id_name_; };

  /// Accessor for identification number of this leg object.
  /// @return The identification number of the leg object
  inline int getIDNumber(void) { return id_number_; };

  /// Accessor for the number of child joint objects for this leg.
  /// @return The number of child joint objects of the leg
  inline int getJointCount(void) { return joint_count_; };

  /// Accessor for the step coordination group of this leg.
  /// @return the step coordination group of the leg
  inline int getGroup(void) { return group_; };
  
  /// Accessor for the workspace polyhedron.
  /// @return the workspace polyhedron of the leg
  inline Workspace getWorkspace(void) { return workspace_; };

  /// Accessor for the cuurent state of this leg.
  /// @return The current state of the leg
  inline LegState getLegState(void) { return leg_state_; };

  /// Accessor for the current calculated force vector on the tip of this leg.
  /// @return The current calculated force vector on the tip of the leg
  inline Eigen::Vector3d getTipForceCalculated(void) { return tip_force_calculated_; };
  
  /// Accessor for the current measured force vector on the tip of this leg. 
  /// @return The current measured force vector on the tip of the leg
  inline Eigen::Vector3d getTipForceMeasured(void) { return tip_force_measured_; };
  
  /// Accessor for the current calculated torque vector on the tip of this leg.
  /// @return The current calculated torque vector on the tip of the leg
  inline Eigen::Vector3d getTipTorqueCalculated(void) { return tip_torque_calculated_; };
  
  /// Accessor for the current measured torque vector on the tip of this leg.
  /// @return The current measured torque vector on the tip of the leg
  inline Eigen::Vector3d getTipTorqueMeasured(void) { return tip_torque_measured_; };
  
  /// Accessor for the current estimated pose of the stepping surface plane.
  /// @return The current estimated pose of the steppping surface plane
  inline Pose getStepPlanePose(void) { return step_plane_pose_; };

  /// Accessor for the current admittance control position offset for this leg.
  /// @return The current admittance control position offset for the leg
  inline Eigen::Vector3d getAdmittanceDelta(void) { return admittance_delta_; };

  /// Accessor for the virtual mass value used in the admittance control model of this leg.
  /// @return The virtual mass value used in the admittance control model of the leg
  inline double getVirtualMass(void) { return virtual_mass_; };

  /// Accessor for the current virtual stiffness value used in the admittance control model of this leg.
  /// @return The current virtual stiffness value used in the admittance control model of the leg
  inline double getVirtualStiffness(void) { return virtual_stiffness_; };

  /// Accessor for the virtual damping ratio value used in the admittance control model of this leg.
  /// @return The virtual damping ratio value used in the admittance control model of this leg
  inline double getVirtualDampingRatio(void) { return virtual_damping_ratio_; };

  /// Accessor for the admittance state object for this leg.
  /// @return The admittance state object for the leg
  inline state_type* getAdmittanceState(void) { return &admittance_state_; };

  /// Accessor for the container of Joint objects associated with this leg.
  /// @return The container of Joint objects associated with the leg
  inline JointContainer* getJointContainer(void) { return &joint_container_; };

  /// Accessor for the container of Link objects associated with this leg.
  /// @return The container of Link objects associated with the leg
  inline LinkContainer* getLinkContainer(void) { return &link_container_; };

  /// Accessor for the Tip object associated with this leg.
  /// @return The Tip object associated with the leg
  inline std::shared_ptr<Tip> getTip(void) { return tip_; };

  /// Accessor for the LegStepper object associated with this leg.
  /// @return The LegStepper object associated with the leg
  inline std::shared_ptr<LegStepper> getLegStepper(void) { return leg_stepper_; };

  /// Accessor for the LegPoser object associated with this leg.
  /// @return The LegPoser object associated with the leg
  inline std::shared_ptr<LegPoser> getLegPoser(void) { return leg_poser_; };
  
  /// Accessor for the desired tip pose of this leg.
  /// @return The desired tip pose of the leg
  inline Pose getDesiredTipPose(void) { return desired_tip_pose_; };

  /// Accessor for the desired tip velocity of this leg.
  /// @return The desired tip velocity of the leg
  inline Eigen::Vector3d getDesiredTipVelocity(void) { return desired_tip_velocity_; };

  /// Accessor for the current tip pose of this leg.
  /// @return The current tip pose of the leg
  inline Pose getCurrentTipPose(void) { return current_tip_pose_; };

  /// Accessor for the current tip velocity of this leg.
  /// @return The current tip velocity of the leg
  inline Eigen::Vector3d getCurrentTipVelocity(void) { return current_tip_velocity_; };
  
  /// Accessor for the current pose of the robot body.
  /// @return The current pose of the robot body
  inline Pose getCurrentBodyPose(void) { return model_->getCurrentPose(); };
  
  /// Accessor for the default pose of the robot body.
  /// @return The default pose of the robot body
  inline Pose getDefaultBodyPose(void) { return model_->getDefaultPose(); };
  
  /// Modifier for the workspace of the leg.
  /// @param[in] workspace The new leg workspace
  inline void setWorkspace(const Workspace& workspace) { workspace_ = workspace; };

  /// Modifier for the curent state of this leg.
  /// @param[in] leg_state The new state of this leg
  inline void setLegState(const LegState& leg_state) { leg_state_ = leg_state; };

  /// Modifier for the publisher of leg state messages.
  /// @param[in] publisher The new ros publisher to publish leg state messages
  inline void setStatePublisher(const ros::Publisher& publisher) { leg_state_publisher_ = publisher; };

  /// Modifier for the publisher of ASC state messages.
  /// @param[in] publisher The new ros publisher to publish ASC state messages
  inline void setASCStatePublisher(const ros::Publisher& publisher) { asc_leg_state_publisher_ = publisher; };

  /// Modifier for the LegStepper object associated with this leg.
  /// @param[in] leg_stepper A pointer to the new LegStepper object for this leg
  inline void setLegStepper(std::shared_ptr<LegStepper> leg_stepper) { leg_stepper_ = leg_stepper; };

  /// Modifier for the LegPoser object associated with this leg.
  /// @param[in] leg_poser A pointer to the new LegPoser object for this leg
  inline void setLegPoser(std::shared_ptr<LegPoser> leg_poser) { leg_poser_ = leg_poser; };

  /// Modifier for the current estimated force vector on the tip of this leg (from calculation of joint torques).
  /// @param[in] tip_force The new tip force estimate for this leg
  inline void setTipForceCalculated(const Eigen::Vector3d& tip_force) { tip_force_calculated_ = tip_force; };
  
  /// Modifier for the current estimated torque vector on the tip of this leg (from calculation of joint torques).
  /// @param[in] tip_torque The new tip torque estimate for this leg
  inline void setTipTorqueCalculated(const Eigen::Vector3d& tip_torque) { tip_torque_calculated_ = tip_torque; };
  
  /// Modifier for the current estimated force vector on the tip of this leg (from direct measurement).
  /// @param[in] tip_force The new tip force estimate for this leg
  inline void setTipForceMeasured(const Eigen::Vector3d& tip_force) { tip_force_measured_ = tip_force; };
  
  /// Modifier for the current estimated torque vector on the tip of this leg (from direct measurement).
  /// @param[in] tip_torque The new tip torque estimate for this leg
  inline void setTipTorqueMeasured(const Eigen::Vector3d& tip_torque) { tip_torque_measured_ = tip_torque; };
  
  /// Modifier for the current estimated pose of the stepping surface plane.
  /// @param[in] step_plane_pose The new estimate of the pose of the stepping surface plane for this leg
  inline void setStepPlanePose(const Pose& step_plane_pose) { step_plane_pose_ = step_plane_pose; };
  
  /// Modifier for the current admittance control position offset for this leg. Only sets component of input vector 
  /// which aligns with direction of tip.
  /// @param[in] delta The calculated delta vector of this leg
  inline void setAdmittanceDelta(const Eigen::Vector3d& delta) 
  { 
    admittance_delta_ = getProjection(delta, current_tip_pose_.rotation_._transformVector(Eigen::Vector3d::UnitX()));
  };

  /// Modifier for the virtual mass value used in the admittance control model of this leg.
  /// @param[in] mass The new virtual mass value
  inline void setVirtualMass(const double& mass) { virtual_mass_ = mass; };

  /// Modifier for the current virtual stiffness value used in the admittance control model of this leg.
  /// @param[in] stiffness The new virtual stiffness value
  inline void setVirtualStiffness(const double& stiffness) { virtual_stiffness_ = stiffness; };

  /// Modifier for the virtual damping ratio value used in the admittance control model of this leg.
  /// @param[in] damping_ratio The new virtual damping ratio value
  inline void setVirtualDampingRatio(const double& damping_ratio) { virtual_damping_ratio_ = damping_ratio; };

  /// Publishes the given message via the leg state pubisher object.
  /// @param[in] msg The leg state message to be published
  inline void publishState(const syropod_highlevel_controller::LegState& msg) { leg_state_publisher_.publish(msg); };

  /// Publishes the given message via the ASC leg state pubisher object.
  /// @param[in] msg The ASC leg state message to be published
  inline void publishASCState(const std_msgs::Bool& msg) { asc_leg_state_publisher_.publish(msg); };

  /// Generates child joint/link/tip objects and copies state from reference leg if provided.
  /// Separated from constructor due to shared_from_this() constraints.
  /// @param[in] leg A pointer to an existing reference robot model leg object
  void generate(std::shared_ptr<Leg> leg = NULL);

  /// Initialises leg object by setting desired joint state to default values or to current position (from encoders)
  /// and running forward kinematics for tip position.
  /// @param[in] use_default_joint_positions Flag denoting if the leg should initialise using default joint position
  /// values for any joint with unknown current position values
  void init(const bool& use_default_joint_positions);
  
  /// Generates workspace polyhedron for this leg by searching for kinematic limitations.
  /// @return The generated workspace object
  Workspace generateWorkspace(void);
  
  /// Generates interpolated workplane within workspace from given height above workspace origin.
  /// @param[in] height The desired workplane height from workspace origin
  /// @return The interpolated workplane at input height
  Workplane getWorkplane(const double& height);
  
  /// Generates a reachable tip position from an input test tip position within the workspace of this leg.
  /// @param[in] reference_tip_position The tip position to use as reference to generate a reachable tip position
  /// @return A reachable tip position which lies within the leg workspace based on the input reference tip position
  Eigen::Vector3d makeReachable(const Eigen::Vector3d& reference_tip_position);
  
  /// Updates joint default positions according to current joint positions.
  void updateDefaultConfiguration(void);

  /// Generates a JointState message from the desired state of the joints of the leg object.
  /// @param[out] joint_state_msg The output JointState mesage to fill with the state of joints within this leg object
  void generateDesiredJointStateMsg(sensor_msgs::JointState* joint_state_msg);

  /// Returns pointer to joint requested via identification number input.
  /// @param[in] joint_id_number The identification name of the requested joint object pointer
  /// @return Pointer to joint requested via identification number input
  inline std::shared_ptr<Joint> getJointByIDNumber(const int& joint_id_number) { return joint_container_[joint_id_number]; };

  /// Returns pointer to joint requested via identification name string input.
  /// @param[in] joint_id_name The identification name of the requested joint object pointer
  /// @return Pointer to joint requested via identification name string input
  std::shared_ptr<Joint> getJointByIDName(const std::string& joint_id_name);

  /// Returns pointer to link requested via identification number input.
  /// @param[in] link_id_number The identification number of the requested link object pointer
  /// @return Pointer to link requested via identification number input
  inline std::shared_ptr<Link> getLinkByIDNumber(const int& link_id_number) { return link_container_[link_id_number]; };

  /// Returns pointer to link requested via identification name string input.
  /// @param[in] link_id_name The identification name of the requested link object pointer
  /// @return Pointer to link requested via identification name string input
  std::shared_ptr<Link> getLinkByIDName(const std::string& link_id_name);

  /// Sets desired tip pose to the input, applying admittance controller vertical offset (delta z) if requested.
  /// @param[in] tip_pose The input desired tip pose
  /// @param[in] apply_delta Flag denoting if the admittance position offset should be applied to desired tip pose
  void setDesiredTipPose(const Pose& tip_pose = Pose::Undefined(), bool apply_delta = true);

  /// Modifier for the desired tip velocity of the tip of this leg object.
  /// @param[in] tip_velocity The new tip velocity of this leg object
  inline void setDesiredTipVelocity(const Eigen::Vector3d& tip_velocity) { desired_tip_velocity_ = tip_velocity; };
  
  /// Calculates an estimate for the tip force vector acting on this leg, using the calculated state jacobian and 
  /// values for the torque on each joint in the leg.
  /// @todo Implement rotation to tip frame
  void calculateTipForce(void);
  
  /// Checks tip force magnitude against touchdown/liftoff thresholds to instantaneously define the location of the 
  /// step plane.
  void touchdownDetection(void);
  
  /// Applies inverse kinematics to calculate required joint positions to achieve desired tip pose. Inverse
  /// kinematics is generated via the calculation of a jacobian for the current state of the leg, which is used as per
  /// the Damped Least Squares method to generate a change in joint position for each joint.
  /// @param[in] delta The iterative change in tip position and rotation
  /// @param[in] solve_rotation Flag denoting if IK should solve for rotation as well rather than just position
  /// @return The position delta for each joint in the model to achieve desired tip position delta. 
  /// @todo Calculate optimal DLS coefficient (this value currently works sufficiently)
  Eigen::VectorXd solveIK(const Eigen::MatrixXd& delta, const bool& solve_rotation);
  
  /// Updates the joint positions of each joint in this leg based on the input vector. Clamps joint velocities and
  /// positions based on limits and calculates a ratio of proximity of joint position to limits.
  /// @param[in] delta The iterative change in joint position for each joint
  /// @param[in] simulation Flag denoting if this execution is for simulation purposes rather than normal use
  /// @return The ratio of the proximity of the joint position to it's limits (i.e. 0.0 = at limit, 1.0 = furthest away)
  double updateJointPositions(const Eigen::VectorXd& delta, const bool& simulation);

  /// Applies inverse kinematics solution to achieve desired tip position. Clamps joint positions and velocities
  /// within limits and applies forward kinematics to update tip position. Returns an estimate of the chance of solving
  /// IK within thresholds on the next iteration. 0.0 denotes failure on THIS iteration.
  /// @param[in] simulation Flag denoting if this execution is for simulation purposes rather than normal use
  /// @return A double between 0.0 and 1.0 which estimates the chance of solving IK within thresholds on the next 
  /// iteration. 0.0 denotes failure on THIS iteration.
  double applyIK(const bool& simulation = false);

  /// Updates joint transforms and applies forward kinematics to calculate a new tip pose. 
  /// Sets leg current tip pose to new pose if requested.
  /// @param[in] set_current Flag denoting of the calculated tip pose should be set as the current tip pose
  /// @param[in] use_actual Flag denoting if the joint position values used for FK come from actual motor outputs
  /// @return Calculated new tip pose by applying forward kinematics
  Pose applyFK(const bool& set_current = true, const bool& use_actual = false);

private:
  std::shared_ptr<Model> model_;        ///< A pointer to the parent robot model object
  const Parameters& params_;       ///< Pointer to parameter data structure for storing parameter variables
  JointContainer joint_container_; ///< The container object for all child Joint objects
  LinkContainer link_container_;   ///< The container object for all child Link objects
  std::shared_ptr<Tip> tip_;            ///< A pointer to the child Tip object

  std::shared_ptr<LegStepper> leg_stepper_;  ///< A pointer to the LegStepper object associated with this leg
  std::shared_ptr<LegPoser> leg_poser_;      ///< A pointer to the LegPoser object associated with this leg

  const int id_number_;         ///< The identification number for this leg
  const std::string id_name_;   ///< The identification name for this leg
  const int joint_count_;       ///< The number of child Joint objects associated with this leg
  LegState leg_state_;          ///< The current state of this leg
  
  Workspace workspace_;         ///< Polyhedron (planes of radii) representing workspace of this leg

  ros::Publisher leg_state_publisher_;     ///< The ros publisher object that publishes state messages for this leg
  ros::Publisher asc_leg_state_publisher_; ///< The ros publisher object that publishes ASC state messages for this leg

  Eigen::Vector3d admittance_delta_;    ///< The admittance controller tip position offset vector
  double virtual_mass_;          ///< The virtual mass of the admittance controller virtual model of this leg
  double virtual_stiffness_;     ///< The virtual stiffness of the admittance controller virtual model of this leg
  double virtual_damping_ratio_; ///< The virtual damping ratio of the admittance controller virtual model of this leg
  state_type admittance_state_;  ///< The admittance state of the admittance controller virtual model of this leg

  Pose desired_tip_pose_;        ///< Desired tip pose before applying Inverse/Forward kinematics
  Pose current_tip_pose_;        ///< Current tip pose according to the model
  
  Eigen::Vector3d desired_tip_velocity_; ///< Desired linear tip velocity before applying Inverse/Forward kinematics
  Eigen::Vector3d current_tip_velocity_; ///< Current linear tip velocity according to the model

  int group_; ///< Leg stepping coordination group (Either 0 or 1)

  Eigen::Vector3d tip_force_calculated_;  ///< Calculated force estimation on the tip
  Eigen::Vector3d tip_torque_calculated_; ///< Calculated torque estimation on the tip
  Eigen::Vector3d tip_force_measured_;    ///< Measured force estimation on the tip
  Eigen::Vector3d tip_torque_measured_;   ///< Measured torque estimation on the tip
  Pose step_plane_pose_;           ///< Estimation of the pose of the stepping surface plane
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles data for each 'link' of a parent leg object. This data includes the DH parameters which define
/// the transformation between the actuating joint at the beginning of this link (in the kinematic chain) and the next
/// joint for which this link is the reference.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Link
{
public:
  /// Constructor for Link object. Initialises member variables from parameters.
  /// @param[in] leg A pointer to the parent leg object
  /// @param[in] actuating_joint A pointer to the actuating joint object, from which this link is moved
  /// @param[in] id_number The identification number for this link
  /// @param[in] params A pointer to the parameter data structure
  Link(std::shared_ptr<Leg> leg, std::shared_ptr<Joint> actuating_joint, const int& id_number, const Parameters& params);
  
  /// Copy Constructor for Link object. Initialises member variables from existing Link object.
  /// @param[in] link A pointer to an existing link object
  Link(std::shared_ptr<Link> link);

  const std::shared_ptr<Leg> parent_leg_;        ///< A pointer to the parent leg object associated with this link
  const std::shared_ptr<Joint> actuating_joint_; ///< A pointer to the actuating Joint object associated with this link
  const int id_number_;                     ///< The identification number for this link
  const std::string id_name_;                    ///< The identification name for this link
  const double dh_parameter_r_;             ///< The DH parameter 'r' associated with this link
  const double dh_parameter_theta_;         ///< The DH parameter 'theta' associated with this link
  const double dh_parameter_d_;             ///< The DH parameter 'd' associated with this link
  const double dh_parameter_alpha_;         ///< The DH parameter 'alpha' associated with this link
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles data for each 'joint' of a parent leg object and contains functions which allow the 
/// transformation of positions between the robot frame and the joint frame along the kinematic chain.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Joint
{
public:
  /// Constructor for Joint object. Initialises member variables from parameters and generates initial transform.
  /// @param[in] leg A pointer to the parent leg object
  /// @param[in] reference_link A pointer to the reference link object, from which this joint actuates
  /// @param[in] id_number The identification number for this joint
  /// @param[in] params A pointer to the parameter data structure
  Joint(std::shared_ptr<Leg> leg, std::shared_ptr<Link> reference_link, const int& id_number, const Parameters& params);
  
  /// Copy Constructor for Joint object. Initialises member variables from existing Joint object.
  /// @param[in] joint A pointer to an existing Joint object
  Joint(std::shared_ptr<Joint> joint);
  
  /// Constructor for null joint object. Acts as a null joint object for use in ending kinematic chains.
  Joint(void);

  /// Returns the transformation matrix from the specified target joint of the robot model to this joint. 
  /// Target joint defaults to the origin of the kinematic chain.
  /// @param[in] target_joint_id ID number of joint object defining the target joint for the transformation
  /// @return The transformation matrix from target joint to this joint
  inline Eigen::Matrix4d getTransformFromJoint(const int& target_joint_id = 0) const
  {
    std::shared_ptr<Joint> next_joint = reference_link_->actuating_joint_;
    bool at_target = (target_joint_id == next_joint->id_number_);
    return at_target ? current_transform_ : next_joint->getTransformFromJoint(target_joint_id) * current_transform_;
  };

  /// Returns the pose of (or a pose relative to) the origin of this joint in the frame of the robot model.
  /// @param[in] joint_frame_pose The pose relative to the joint frame that is requested in the robot frame
  /// @return The input pose transformed into the robot frame
  inline Pose getPoseRobotFrame(const Pose& joint_frame_pose = Pose::Identity()) const
  {
    Eigen::Matrix4d transform = getTransformFromJoint();
    return joint_frame_pose.transform(transform);
  };

  /// Returns the pose of (or a pose relative to) the origin of the robot model in the frame of this joint.
  /// @param[in] robot_frame_pose The position relative to the robot frame that is requested in the joint frame
  /// @return The input pose transformed into the frame of this joint
  inline Pose getPoseJointFrame(const Pose& robot_frame_pose = Pose::Identity()) const
  {
    Eigen::MatrixXd transform = getTransformFromJoint();
    return robot_frame_pose.transform(transform.inverse());
  };

  const std::shared_ptr<Leg> parent_leg_;      ///< A pointer to the parent leg object associated with this joint
  const std::shared_ptr<Link> reference_link_; ///< A pointer to the reference Link object associated with this joint
  const int id_number_;                   ///< The identification number for this joint
  const std::string id_name_;                  ///< The identification name for this joint
  Eigen::Matrix4d current_transform_;            ///< The current transformation matrix between previous joint and this joint
  Eigen::Matrix4d identity_transform_;           ///< The identity transformation matrix between previous joint and this joint

  ros::Publisher desired_position_publisher_; ///< The ros publisher for publishing desired position values

  const double min_position_ = 0.0;       ///< The minimum position allowed for this joint
  const double max_position_ = 0.0;       ///< The maximum position allowed for this joint
  std::vector<double> packed_positions_ ;      ///< The defined position of this joint in a 'packed' state
  const double unpacked_position_ = 0.0;  ///< The defined position of this joint in an 'unpacked' state
  const double max_angular_speed_ = 0.0;  ///< The maximum angular speed of this joint

  double desired_position_ = 0.0;      ///< The desired angular position of this joint
  double desired_velocity_ = 0.0;      ///< The desired angular velocity of this joint
  double desired_effort_ = 0.0;        ///< The desired angular effort of this joint
  double prev_desired_position_ = 0.0; ///< The desired angular position of this joint at the previous iteration
  double prev_desired_velocity_ = 0.0; ///< The desired angular velocity of this joint at the previous iteration
  double prev_desired_effort_ = 0.0;   ///< The desired angular effort of this joint at the previous iteration

  double current_position_ = UNASSIGNED_VALUE; ///< The current position of this joint according to hardware
  double current_velocity_ = 0.0;              ///< The current velocity of this joint according to hardware
  double current_effort_ = 0.0;                ///< The current effort of this joint according to hardware
  
  double default_position_ = UNASSIGNED_VALUE; ///< The default position of this joint
  double default_velocity_ = 0.0;              ///< The default velocity of this joint
  double default_effort_ = 0.0;                ///< The default effort of this joint

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles data for the 'tip' of a parent leg object and contains functions which allow the transformation
/// of positions between the robot frame and the tip frame along the kinematic chain.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Tip
{
public:
  /// Constructor for Tip object. Initialises member variables from parameters and generates initial transform.
  /// @param[in] leg A pointer to the parent leg object
  /// @param[in] reference_link A pointer to the reference link object, which is attached to this tip object
  Tip(std::shared_ptr<Leg> leg, std::shared_ptr<Link> reference_link);
  
  /// Copy Constructor for Tip object. Initialises member variables from existing Tip object.
  /// @param[in] tip A pointer to an existing tip object
  Tip(std::shared_ptr<Tip> tip);

  /// Returns the transformation matrix from the specified target joint  of the robot model to the tip. 
  /// Target joint defaults to the origin of the kinematic chain.
  /// @param[in] target_joint_id ID number of joint object defining the target joint for the transformation
  /// @return The transformation matrix from target joint to the tip
  inline Eigen::Matrix4d getTransformFromJoint(const int& target_joint_id = 0) const
  {
    std::shared_ptr<Joint> next_joint = reference_link_->actuating_joint_;
    bool at_target = (target_joint_id == next_joint->id_number_);
    return at_target ? current_transform_ : next_joint->getTransformFromJoint(target_joint_id) * current_transform_;
  };

  /// Returns the pose of (or a pose relative to) the origin of the tip in the frame of the robot model.
  /// @param[in] tip_frame_pose The pose relative to the tip frame that is requested in the robot frame
  /// @return The input pose transformed into the robot frame
  inline Pose getPoseRobotFrame(const Pose& tip_frame_pose = Pose::Identity()) const
  {
    Eigen::Matrix4d transform = getTransformFromJoint();
    return tip_frame_pose.transform(transform);
  };
  
  /// Returns the pose of (or a pose relative to) the origin of the robot model in the frame of the tip.
  /// @param[in] robot_frame_pose The pose relative to the robot frame that is requested in the tip frame
  /// @return The input pose transformed into the tip frame
  inline Pose getPoseTipFrame(const Pose& robot_frame_pose = Pose::Identity()) const
  {
    Eigen::Matrix4d transform = getTransformFromJoint();
    return robot_frame_pose.transform(transform.inverse());
  };

  const std::shared_ptr<Leg> parent_leg_;       ///< A pointer to the parent leg object associated with the tip
  const std::shared_ptr<Link> reference_link_;  ///< A pointer to the reference Link object associated with the tip
  const std::string id_name_;                   ///< The identification name for the tip
  Eigen::Matrix4d current_transform_;             ///< The current transformation matrix between previous joint and the tip
  Eigen::Matrix4d identity_transform_;            ///< The identity transformation matrix between previous joint and the tip

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_MODEL_H
