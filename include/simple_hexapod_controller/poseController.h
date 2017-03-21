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

class PoseController
{      
  public:
    PoseController(Model* model, Parameters* params);
    
    inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };
    inline ImuData getImuData(void) { return imu_data_; };
    inline Parameters* getParameters(void) { return params_; };
    inline Vector3d getRotationAbsementError(void) { return rotation_absement_error_; };
    inline Vector3d getRotationPositionError(void) { return rotation_position_error_; };
    inline Vector3d getRotationVelocityError(void) { return rotation_velocity_error_; };
    inline Vector3d getTranslationAbsementError(void) { return translation_absement_error_; };
    inline Vector3d getTranslationPositionError(void) { return translation_position_error_; };
    inline Vector3d getTranslationVelocityError(void) { return translation_velocity_error_; };
    inline Vector3d getTranslationAccelerationError(void) { return translation_acceleration_error_; };
    
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
    int shutDownSequence(void);
    int startUpSequence(void);    
    
    //Coordinated leg movements - joint position
    int packLegs(double time_to_pack);
    int unpackLegs(double time_to_unpack);
    
    // Compensation functions
    void updateCurrentPose(double body_height);
    Pose manualCompensation(void);
    Pose autoCompensation(void);    
    Quat imuCompensation(void);
    Vector3d inclinationCompensation(double body_height);
    double impedanceControllerCompensation(void);
    
    void calculateDefaultPose(void);
    
  private:    
    Model* model_;
    ImuData imu_data_; 
    Parameters* params_;
    std::map<int, Leg*>::iterator leg_it_;
    
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
    bool set_target_ = true;
    bool lower_complete_ = false;
    bool step_complete_ = false;
    bool raise_complete_ = false;
    
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

class LegPoser
{
  public:
    LegPoser(PoseController* poser, Leg* leg);
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getTargetTipPosition(void) { return target_tip_position_; };
		inline Pose getSwingAutoPose(void) { return swing_auto_pose_; };
    
    inline void setCurrentTipPosition(Vector3d current) { current_tip_position_ = current; };
    inline void setTargetTipPosition(Vector3d target) { target_tip_position_ = target; };
		inline void setSwingAutoPose(Pose pose) { swing_auto_pose_ = pose; };
    
    //updatePosition(void); //apply current pose to generate new tip position
    int moveToJointPosition(vector<double> targetJointPositions, double speed = 2.0); //move leg joints directly to target postion
    int stepToPosition(Vector3d targetTipPosition, Pose targetPose, double lift_height, double time_to_step);
    
  private:
    PoseController* poser_;
    Leg* leg_;
    
    bool first_iteration_ = true;
    int master_iteration_count_ = 0;
    
    vector<double> origin_joint_positions_;
		
		Pose swing_auto_pose_;
    
    Vector3d origin_tip_position_;
    Vector3d current_tip_position_;
    Vector3d target_tip_position_;
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_POSE_CONTROLLER_H */
