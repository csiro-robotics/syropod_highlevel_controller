// Controller that handles changes in body pose
#pragma once
#include "parametersAndStates.h"
#include "model.h"
#include "debugOutput.h"
#include "walkController.h"
#include "imu.h"
#include "geometry_msgs/Twist.h"

class PoseController
{      
  public:
    //Constructor
    PoseController(Model *model, Parameters* params);
    
    //Accessors
    inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };
    
    //Mutators
    inline void setPoseResetMode(PoseResetMode mode) { pose_reset_mode_ = mode; };
    inline void setManualPoseInput(Vector3d translation, Vector3d rotation) 
    {
      translation_velocity_input_ = translation;
      rotation_velocity_input_ = rotation;
    }
    
    // Updates tip positions based on current pose
    void updateStance(void);

    //Coordinated leg movements - tip position
    double directStartup();
    double stepToNewStance();
    double poseForLegManipulation();
    bool shutDownSequence();
    bool startUpSequence();    
    
    //Coordinated leg movements - joint position
    bool packLegs(double time_to_pack);
    bool unpackLegs(double time_to_unpack);
    
    // Compensation functions
    void autoCompensation(void);
    void manualCompensation(void);
    void imuCompensation(ImuData imu_data);
    void inclinationCompensation(WalkController* walker, ImuData imu_data);
    void impedanceControllerCompensation();
    
    void calculateDefaultPose();
    
  private:    
    Model* model_;
    Parameters* params_;
    std::map<std::string, Leg*>::iterator leg_it_;
    
    PoseResetMode pose_reset_mode_;
    Vector3d translation_velocity_input_;
    Vector3d rotation_velocity_input_;
    
    //Tripod coordination variables
    int legs_completed_task_ = 0;
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
    Vector3d rotation_absement_error;
    Vector3d rotation_position_error;
    Vector3d rotation_velocity_error;

    Vector3d translationAbsementError;
    Vector3d translationPositionError;
    Vector3d translationVelocityError;
    Vector3d translationAccelerationError;
};

class LegPoser
{
  public:
    LegPoser(PoseController* poser, Vector3d packed_joint_positions, Vector3d unpacked_joint_positions);
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getTargetTipPosition(void) { return target_tip_position_; };
    inline Vector3d getPackedJointPositions(void) { return packed_joint_positions_; };
    inline Vector3d getUnpackedJointPositions(void) { return unpacked_joint_positions_; };
    
    inline void setCurrentTipPosition(Vector3d current) { current_tip_position_ = current; };
    inline void setTargetTipPosition(Vector3d target) { target_tip_position_ = target; };
    
    //updatePosition(void); //apply current pose to generate new tip position
    bool moveToJointPosition(Vector3d targetJointPositions, double speed = 2.0); //move leg joints directly to target postion
    double stepToPosition(Vector3d targetTipPosition, Pose targetPose);
    
  private:
    PoserController* poser_;
    Leg3DOF* leg_;
    
    bool first_iteration_ = true;
    int master_iteration_count_ = 0;
    
    Vector3d origin_joint_positions_;
    Vector3d packed_joint_positions_;
    Vector3d unpacked_joint_positions_;
    
    Vector3d origin_tip_position_;
    Vector3d current_tip_position_;
    Vector3d target_tip_position_;
};
