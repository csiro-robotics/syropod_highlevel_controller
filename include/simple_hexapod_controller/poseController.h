#ifndef SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H
/** 
 *  \file    pose_controller.h
 *  \brief   Handles control of hexapod body posing. Part of simple hexapod controller.
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

#include "standardIncludes.h"
#include "parametersAndStates.h"
#include "quat.h"
#include "pose.h"

#include "model.h"
#include "walkController.h"

//#include "debugOutput.h"

struct ImuData
{
  Quat orientation;
  Vector3d linear_acceleration;
  Vector3d angular_velocity;
};

class AutoPoser;

class PoseController
{      
  public:
    PoseController(Model* model, Parameters* params);
    void setAutoPoseParams(void);
    inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };
		inline PosingState getAutoPoseState(void) { return auto_posing_state_; };
    inline ImuData getImuData(void) { return imu_data_; };
    inline Parameters* getParameters(void) { return params_; };
		inline Pose getAutoPose(void) { return auto_pose_; };
		inline int getPhaseLength(void) { return pose_phase_length_; };
		inline int getNormaliser(void) { return normaliser_; };
		inline double getPoseFrequency(void) { return pose_frequency_; };
    inline Vector3d getRotationAbsementError(void) { return rotation_absement_error_; };
    inline Vector3d getRotationPositionError(void) { return rotation_position_error_; };
    inline Vector3d getRotationVelocityError(void) { return rotation_velocity_error_; };
    inline Vector3d getTranslationAbsementError(void) { return translation_absement_error_; };
    inline Vector3d getTranslationPositionError(void) { return translation_position_error_; };
    inline Vector3d getTranslationVelocityError(void) { return translation_velocity_error_; };
    inline Vector3d getTranslationAccelerationError(void) { return translation_acceleration_error_; };
    
		inline void setPhaseLength(int phase_length) { pose_phase_length_ = phase_length; };
		inline void setNormaliser(int normaliser) { normaliser_ = normaliser; };
    inline void setPoseResetMode(PoseResetMode mode) { pose_reset_mode_ = mode; };
    inline void setImuData(Quat orientation, Vector3d linear_acceleration, Vector3d angular_velocity)
    {
      imu_data_.orientation = orientation;
      imu_data_.linear_acceleration = linear_acceleration;
      imu_data_.angular_velocity = angular_velocity;
    }
    inline void setManualPoseInput(Vector3d translation, Vector3d rotation) 
    {
      translation_velocity_input_ = translation;
      rotation_velocity_input_ = rotation;
    }
    
    // Updates tip positions based on current pose
    void updateStance(void);

    //Coordinated leg movements - tip position
    int directStartup(void);
    int stepToNewStance(void);
    int poseForLegManipulation(void);
    int executeSequence(SequenceSelection sequence);
    
    //Coordinated leg movements - joint position
    int packLegs(double time_to_pack);
    int unpackLegs(double time_to_unpack);
    
    // Compensation functions
    void updateCurrentPose(double body_height, WalkState walk_state);
    Pose manualCompensation(void);
    Pose autoCompensation(void);
		Quat imuCompensation(void);
    Vector3d inclinationCompensation(double body_height);
    double impedanceControllerCompensation(void);
    
    void calculateDefaultPose(void);
    
    void resetAllPosing(void)
    {
      manual_pose_ = Pose::identity();
      auto_pose_ = Pose::identity();
      default_pose_ = Pose::identity();
      inclination_compensation_offset_ = Vector3d(0,0,0);
    }
    
  private:    
    Model* model_;
    ImuData imu_data_; 
    Parameters* params_;
    map<int, Leg*>::iterator leg_it_;
    map<int, Joint*>::iterator joint_it_;
    
    PoseResetMode pose_reset_mode_;
    Vector3d translation_velocity_input_;
    Vector3d rotation_velocity_input_;
    
    //Tripod coordination variables
    int legs_completed_step_ = 0;
    int current_group_ = 0;
    
    bool recalculate_offset_ = true;
    
    //Tracked compensation variables
    Pose manual_pose_;  // Current pose of body only using manual compensation
    Pose auto_pose_;  // Current pose of body only using auto compensation
    Pose default_pose_; // Default pose calculated for different loading patterns
    Vector3d inclination_compensation_offset_;
    
    //Shutdown/Startup state variables
    int transition_step_ = 0;
    int transition_step_count_ = 0;
    bool set_target_ = true;
    bool proximity_alert_ = false;
    bool horizontal_transition_complete_ = false;
    bool vertical_transition_complete_ = false;
    bool first_sequence_execution_ = true;
    
    //Direct startup state variables
    bool move_joints_complete = false;
    bool direct_startup_complete = false;
		
		//Auto Compensation cycle variables
		vector<AutoPoser*> auto_poser_container_;
		PosingState auto_posing_state_;
		int pose_phase_ = 0;
		double pose_frequency_ = 0.0;
		int pose_phase_length_ = 0;
		int normaliser_ = 1;
    
    //DEBUGGING
    // Imu compensation PID error vectors
    Vector3d rotation_absement_error_;
    Vector3d rotation_position_error_;
    Vector3d rotation_velocity_error_;

    Vector3d translation_absement_error_;
    Vector3d translation_position_error_;
    Vector3d translation_velocity_error_;
    Vector3d translation_acceleration_error_;
};

class AutoPoser
{
	public:
		AutoPoser(PoseController* poser, int id);
		Pose updatePose(int phase);
		inline int getID(void) { return id_; };
		inline void setStartPhase(int start_phase) { start_phase_ = start_phase; };
		inline void setEndPhase(int end_phase) { end_phase_ = end_phase; };
		inline void setXAmplitude(double x) { x_amplitude_ = x; };
		inline void setYAmplitude(double y) { y_amplitude_ = y; };
		inline void setZAmplitude(double z) { z_amplitude_ = z; };
		inline void setRollAmplitude(double roll) { roll_amplitude_ = roll; };
		inline void setPitchAmplitude(double pitch) { pitch_amplitude_ = pitch; };
		inline void setYawAmplitude(double yaw) { yaw_amplitude_ = yaw; };
		inline void resetChecks(void) 
		{ 
			start_check_ = false;
			end_check_ = pair<bool, bool>(false, false);
		}
		
	private:
		PoseController* poser_;
		int id_;
		int start_phase_;
		int end_phase_;
		bool start_check_;
		pair<bool, bool> end_check_;
		bool allow_posing_ = false;
		
		double x_amplitude_;
		double y_amplitude_;
		double z_amplitude_;
		double roll_amplitude_;
		double pitch_amplitude_;
		double yaw_amplitude_;
};

class LegPoser
{
  public:
    LegPoser(PoseController* poser, Leg* leg);
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getTargetTipPosition(void) { return target_tip_position_; };
    inline Vector3d getUnpackedTipPosition(void) { return unpacked_tip_position_; };
    inline Vector3d getTransitionPosition(int index) { return transition_positions_[index]; }
    inline bool hasTransitionPosition(int index) { return int(transition_positions_.size()) > index; };
		inline Pose getOriginPose(void) { return origin_pose_; };
		inline Pose getAutoPose(void) { return auto_pose_; };
		inline int getPoseNegationPhaseStart() { return pose_negation_phase_start_; };
		inline int getPoseNegationPhaseEnd() { return pose_negation_phase_end_; };
		inline bool getStopNegation(void) { return stop_negation_; };
    inline bool getLegCompletedStep(void) { return leg_completed_step_; };
    
    inline void setCurrentTipPosition(Vector3d current) { current_tip_position_ = current; };
    inline void setTargetTipPosition(Vector3d target) { target_tip_position_ = target; };
    inline void setUnpackedTipPosition(Vector3d unpacked) { unpacked_tip_position_ = unpacked; };
    inline void addTransitionPosition(Vector3d transition) { transition_positions_.push_back(transition); };
		inline void setAutoPose(Pose auto_pose) { auto_pose_ = auto_pose; };
		inline void setOriginPose(Pose origin_pose) { origin_pose_ = origin_pose; };
		inline void setPoseNegationPhaseStart(int start) { pose_negation_phase_start_ = start; };
		inline void setPoseNegationPhaseEnd(int end) { pose_negation_phase_end_ = end; };
		inline void setStopNegation(bool stop_negation) { stop_negation_ = stop_negation; };
    inline void setLegCompletedStep(bool complete) { leg_completed_step_ = complete; };
    inline int resetStepToPosition(void) 
    { 
      first_iteration_ = true;
      int progress = 100;
      return progress;
    }

    //updatePosition(void); //apply current pose to generate new tip position
    int moveToJointPosition(vector<double> targetJointPositions, double speed = 2.0); //move leg joints directly to target postion
    int stepToPosition(Vector3d targetTipPosition, Pose targetPose, double lift_height, double time_to_step);
		void updateAutoPose(int phase);
    
  private:
    PoseController* poser_;
    Leg* leg_;
		
		Pose origin_pose_; // Set pose used as origin of bezier curve in calculating per leg negation pose 
		Pose auto_pose_; // Leg specific auto pose (based off body auto pose negating where required)
		int pose_negation_phase_start_ = 0; //Auto pose negation phase start
		int pose_negation_phase_end_ = 0; //Auto pose negation phase end
    
    bool stop_negation_ = false;
		bool first_iteration_ = true;
    int master_iteration_count_ = 0;
    
    vector<double> origin_joint_positions_;
    
    Vector3d origin_tip_position_;
    Vector3d current_tip_position_;
    Vector3d target_tip_position_;
    
    Vector3d stance_tip_position_;
    Vector3d unpacked_tip_position_;
    vector<Vector3d> transition_positions_;
    
    bool leg_completed_step_ = false;
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H */
