// Controller that handles changes in body pose
#pragma once
#include "parametersAndStates.h"
#include "model.h"
#include "debugOutput.h"
#include "walkController.h"
#include "imu.h"

class PoseController
{      
  public:
    //Constructor
    PoseController(Model *model, Parameters params);
    
    //Accessors
    inline PoseResetMode getPoseResetMode(void) { return pose_reset_mode_; };
    
    //Mutators
    inline void setPoseResetMode(PoseResetMode mode) { pose_reset_mode_ = mode; };
    
    //Other functions
    void updateStance(Vector3d targetTipPositions[3][2], bool excludeSwingingLegs = false);
    
    bool startUpSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
			bool forceSequentialMode);
    bool shutDownSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
			  bool forceSequentialMode);
    double createSequence(Vector3d targetTipPositions[3][2]);

    //Coordinated leg movements - tip position
    double directStartup(Model* model);
    double stepToNewStance(Model* model);
    double poseForLegManipulation(Model* model);
    
    //Coordinated leg movements - joint position
    bool packLegs(Model* model, double time_to_pack);
    bool unpackLegs(Model* model, double time_to_unpack);
    bool moveLegsToJointPositions(map<Leg3DOF*, Vector3d> leg_target_map, 
				  LegCoordinationMode mode, double time_to_move);
    
    // Compensation functions
    void autoCompensation(void);
    void manualCompensation(Vector3d translation_velocity_input, Vector3d rotation_velocity_input);
    void imuCompensation(ImuData imu_data);
    void inclinationCompensation(WalkController* walker, ImuData imu_data);
    void impedanceControllerCompensation(Model* model);
    
    void calculateDefaultPose();
    
  private:    
    Parameters params_;
    PoseResetMode pose_reset_mode_;
    std::map<std::string, Leg*>::iterator leg_it_;
    
    int legs_completed_task_ = 0;
    int current_group_ = 0;
    
    bool recalculate_offset_ = true;
    
    //Tracked compensation variables
    Pose manual_pose_;  // Current pose of body only using manual compensation
    Pose auto_pose_;  // Current pose of body only using auto compensation
    Pose default_pose_; // Default pose calculated for different loading patterns
    Vector3d inclination_compensation_offset_;
    
    //DEBUGGING
    // Imu compensation PID error vectors
    Vector3d rotation_absement_error;
    Vector3d rotation_position_error;
    Vector3d rotation_velocity_error;

    Vector3d translationAbsementError;
    Vector3d translationPositionError;
    Vector3d translationVelocityError;
    Vector3d translationAccelerationError;

    // Used in startup and shutdown sequences
    int sequence_step_ = 0;
    double start_height_ratio_;
    Vector3d phase1TipPositions[3][2];
    Vector3d phase2TipPositions[3][2];
    Vector3d phase3TipPositions[3][2];
    Vector3d phase4TipPositions[3][2];
    Vector3d phase5TipPositions[3][2];
    Vector3d phase6TipPositions[3][2];
    Vector3d phase7TipPositions[3][2];
    Vector3d phase8TipPositions[3][2];   
};

class LegPoser
{
  public:
    LegPoser(PoseController* poser, Vector3d packed_joint_positions, Vector3d unpacked_joint_positions);
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getPackedJointPositions(void) { return packed_joint_positions_; };
    inline Vector3d getUnpackedJointPositions(void) { return unpacked_joint_positions_; };
    
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
};
