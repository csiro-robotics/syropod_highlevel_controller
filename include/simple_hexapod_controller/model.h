#ifndef SIMPLE_HEXAPOD_CONTROLLER_MODEL_H
#define SIMPLE_HEXAPOD_CONTROLLER_MODEL_H
/*******************************************************************************************************************//**
 *  \file    model.h
 *  \brief   Describes the hexapod model including all legs, joints and links. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   January 2017
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
 */

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "quat.h"
#include "pose.h"
#include "simple_hexapod_controller/legState.h"

#define JOINT_TOLERANCE 0.01 // Tolerance allowing assumption that joints are in correct position (metres)

struct Leg;
struct Joint;
struct Link;
struct Tip;

class WalkController;
class LegStepper;
class PoseController;
class LegPoser;

// defines the hexapod model
class Model
{
  public:
    Model(Parameters* parameters);
    
    inline std::map<int, Leg*>* getLegContainer(void) { return &leg_container_;};
    inline int getLegCount(void) { return leg_count_; };
    inline Pose getCurrentPose(void) { return current_pose_; };
    inline double getTimeDelta(void) { return time_delta_;}
    inline Vector2d getLinearVelocity(void) { return linear_velocity_; };
    inline double getAngularVelocity(void) { return angular_velocity_; };
    
    inline void setCurrentPose(Pose pose) { current_pose_ = pose; };
    inline void setLinearVelocity(Vector2d velocity) { linear_velocity_ = velocity; };
    inline void setAngularVelocity(double velocity) { angular_velocity_ = velocity; };
    
    void initLegs(bool use_default_joint_positions);
    bool legsBearingLoad(void);
    vector<Vector3d> getJointPositions(const Pose &pose);
    
    Leg* getLegByIDNumber(int leg_id_num) { return leg_container_[leg_id_num]; };
    Leg* getLegByIDName(std::string leg_id_name);
    
  private:
    std::map<int, Leg*> leg_container_;
		int leg_count_;
    double time_delta_;
    Pose current_pose_;  // Current pose of body including all compensation posing
    Vector2d linear_velocity_;
    double angular_velocity_ = 0.0;
};

//Base leg class
typedef std::vector<double> state_type; //Impedance state used in impedance controller
class Leg
{
public:
	Leg(Model* model, int id_number, Parameters* params);

	inline std::string getIDName(void) { return id_name_; };
	inline int getIDNumber(void) { return id_number_; };
	inline int getNumJoints(void) { return num_joints_; };
	inline int getGroup(void) { return group_; };
	inline LegState getLegState(void) { return leg_state_; };
	inline double getMirrorDir(void) { return mirror_dir_; }; 
	inline double getStanceLegYaw(void) { return stance_leg_yaw_; };
	inline double getTipForce(void) { return tip_force_; };
	inline double getDeltaZ(void) { return delta_z_; };
	inline double getVirtualMass(void) { return virtual_mass_; };
	inline double getVirtualStiffness(void) { return virtual_stiffness_; };
	inline double getVirtualDampingRatio(void) { return virtual_damping_ratio_; };
	inline state_type* getImpedanceState(void) { return &impedance_state_; };
	inline std::map<int, Joint*>* getJointContainer(void) { return &joint_container_; };
	inline std::map<int, Link*>* getLinkContainer(void) { return &link_container_; };
	inline Tip* getTip(void) { return tip_; };
	inline LegStepper* getLegStepper(void) { return leg_stepper_ ;};    
	inline LegPoser* getLegPoser(void) { return leg_poser_ ;};
	inline Vector3d getLocalTipPosition(void) { return local_tip_position_; };
	inline Vector3d getDesiredTipVelocity(void) { return desired_tip_velocity_; };
	inline double getWorkspaceRadius(void) { return workspace_radius_; };

	inline void setLegState(LegState leg_state) { leg_state_ = leg_state; };  
	inline void setStatePublisher(ros::Publisher publisher) { leg_state_publisher_ = publisher; };
	inline void setASCStatePublisher(ros::Publisher publisher) { asc_leg_state_publisher_ = publisher; };
	inline void setLegStepper(LegStepper* leg_stepper) { leg_stepper_ = leg_stepper; };
	inline void setLegPoser(LegPoser* leg_poser) { leg_poser_ = leg_poser; };
	inline void setTipForce(double tip_force) { tip_force_ = tip_force; };
	inline void setDeltaZ(double dz) { delta_z_ = dz; };
	inline void setVirtualMass(double mass) { virtual_mass_ = mass; };
	inline void setVirtualStiffness(double stiffness) { virtual_stiffness_ = stiffness; };
	inline void setVirtualDampingRatio(double damping_ratio) { virtual_damping_ratio_ = damping_ratio; };
	inline void setDesiredTipVelocity(Vector3d tip_velocity) { desired_tip_velocity_ = tip_velocity; };
	inline void setWorkspaceRadius(double radius) { workspace_radius_ = radius; };

	void init(bool use_default_joint_positions);
	void setDesiredTipPosition(Vector3d tip_position, bool apply_delta_z = true);
	bool updateTipForce(bool debug);
  Vector3d applyFK(bool set_local = true);
	double applyIK(bool debug = false,
                 bool ignore_warnings = false,
                 bool clamp_positions = true,
                 bool clamp_velocities = true);
	

	void publishState(simple_hexapod_controller::legState msg) { leg_state_publisher_.publish(msg); };
	void publishASCState(std_msgs::Bool msg) { asc_leg_state_publisher_.publish(msg); };

	Joint* getJointByIDNumber(int joint_id_num) { return joint_container_[joint_id_num]; };
	Link* getLinkByIDNumber(int link_id_num) { return link_container_[link_id_num]; };
	Joint* getJointByIDName(std::string joint_id_name);    
	Link* getLinkByIDName(std::string link_name);

protected:
	Model* model_;
	std::map<int, Joint*> joint_container_;
	std::map<int, Link*> link_container_;
	Tip* tip_;

	const int id_number_;
	const std::string id_name_;

	const int num_joints_;
	const double mirror_dir_;  // 1 or -1 for mirrored
	const double stance_leg_yaw_;

	LegState leg_state_;
	ros::Publisher leg_state_publisher_;
	ros::Publisher asc_leg_state_publisher_;

	LegStepper* leg_stepper_;
	LegPoser* leg_poser_;

	double delta_z_ = 0.0;
	double virtual_mass_;
	double virtual_stiffness_;
	double virtual_damping_ratio_;
	state_type impedance_state_;

	Vector3d desired_tip_position_;
	Vector3d local_tip_position_;  // actual tip position relative to root
	Vector3d desired_tip_velocity_;
	double workspace_radius_;

	int group_;

	double tip_force_ = 0.0; // Vertical force estimation on tip
};

struct Link
{
  public:
    Link(Leg* leg, Joint* actuating_joint, int id, Parameters* params);      
    const Leg* parent_leg;
    const Joint* actuating_joint;
    const int id_number;
    const std::string name;    
    const double length; //DH parameter 'r'   
    const double angle; //DH parameter 'theta'
    const double offset; //DH parameter 'd'
    const double twist; //DH parameter 'alpha'    
};

struct Joint
{
public:
	Joint(Leg* leg, Link* reference_link, int id_number, Parameters* params);
	
	inline Matrix4d getBaseTransform(bool zero = false) const
	{
		Matrix4d transform = zero ? identity_transform : current_transform;
		return (id_number == 1) ? transform : reference_link->actuating_joint->getBaseTransform(zero)*transform;
	};
	inline Vector3d getPositionWorldFrame(bool zero = false, Vector3d joint_frame_position = Vector3d(0,0,0)) const
	{
		Vector4d result(joint_frame_position[0],joint_frame_position[1],joint_frame_position[2],1);
		result = getBaseTransform(zero)*result;
		return Vector3d(result[0], result[1], result[2]);
	};
	inline Vector3d getPositionJointFrame(bool zero = false, Vector3d world_frame_position = Vector3d(0,0,0)) const
	{
		MatrixXd transform = getBaseTransform(zero);
		Vector4d result(world_frame_position[0], world_frame_position[1], world_frame_position[2], 1);
		result = transform.inverse()*result;
		return Vector3d(result[0], result[1], result[2]);
	};
	
	const Leg* parent_leg;
	const Link* reference_link;
	const int id_number;
	const std::string name;
	Matrix4d current_transform;
	Matrix4d identity_transform;
	ros::Publisher desired_position_publisher;
	
	const double position_offset;
	const double min_position;
	const double max_position;
	const double packed_position;
	const double unpacked_position;
	const double max_angular_speed;     
	
	double desired_position = 0.0;
	double desired_velocity = 0.0;
	double desired_effort = 0.0;    
	double prev_desired_position = 0.0;   
	double prev_desired_velocity = 0.0;    
	double prev_desired_effort = 0.0;     
	
	// Current joint state from joint state publisher
	double current_position = UNASSIGNED_VALUE;
	double current_velocity = UNASSIGNED_VALUE;
	double current_effort = UNASSIGNED_VALUE;
};

struct Tip
{
public:
	Tip(Leg* leg, Link* reference_link);
	inline Matrix4d getBaseTransform(bool zero = false) const
	{ 
		MatrixXd transform = zero ? identity_transform : current_transform;
		return reference_link->actuating_joint->getBaseTransform()*transform; 
	};    
	inline Vector3d getPositionWorldFrame(bool zero = false, Vector3d tip_frame_position = Vector3d(0,0,0)) const
	{
		Vector4d result(tip_frame_position[0],tip_frame_position[1],tip_frame_position[2],1);
		result = getBaseTransform(zero)*result;
		return Vector3d(result[0], result[1], result[2]);
	};
	inline Vector3d getPositionTipFrame(bool zero = false, Vector3d world_frame_position = Vector3d(0,0,0)) const
	{
		MatrixXd transform = getBaseTransform(zero);
		Vector4d result(world_frame_position[0], world_frame_position[1], world_frame_position[2], 1);
		result = transform.inverse()*result;
		return Vector3d(result[0], result[1], result[2]);
	};
	
	const Leg* parent_leg;
	const Link* reference_link;
	const std::string name;
	Matrix4d current_transform;
	Matrix4d identity_transform;
	Vector3d relative_body_position;
};
#endif /* SIMPLE_HEXAPOD_CONTROLLER_MODEL_H */




