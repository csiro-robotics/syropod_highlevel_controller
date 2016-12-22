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
    
    //Accessors & Mutators
    //inline Vector3d getTipPositions(int leg_ref, int side_ref) { return this->tipPositions[leg_ref][side_ref];};
    //inline void setParameters(Parameters parameters) { this->parameters = parameters;};
    
    //Other functions
    void updateStance(Vector3d targetTipPositions[3][2], bool excludeSwingingLegs = false);
    double stepLegsToPosition(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
			  StepToPositionModes mode = NO_STEP_MODE, double stepHeight = 0.0, double forceTimeToStep = 0.0);
    bool moveToJointPosition(Vector3d targetJointPositions[3][2], double speed = 2.0);
    bool startUpSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
			bool forceSequentialMode);
    bool shutDownSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
			  bool forceSequentialMode);
    double createSequence(Vector3d targetTipPositions[3][2]);
    void resetSequence(void);

    double poseForLegManipulation(LegState state, int l, int s, double deltaZ[3][2]);

    void calculateDefaultPose();

    //Coordinated leg movements
    void unpackLegs(Model* model, LegCoordinationMode mode, double time_to_unpack);
    bool moveLegsToJointPositions(map<Leg3DOF*, Vector3d> leg_target_map, 
				  LegCoordinationMode mode, double time_to_move);
    
    // Compensation functions
    void autoCompensation(void);
    void manualCompensation(Vector3d translationVelocityInput, Vector3d rotationVelocityInput,
			    PoseResetMode poseResetMode, Pose defaultPose = Pose::identity());
    void imuCompensation(ImuData imuData, Quat targetRotation);
    void inclinationCompensation(ImuData imuData);
    void impedanceControllerCompensation(double deltaZ[3][2]);
    
  private:    
    Parameters params_;
    std::map<std::string, Leg*>::iterator leg_it_;
    
    int legs_completed_task_ = 0;
    int current_group_ = 0;

    bool firstIteration = true;
    int masterIterationCount = 0;

    double moveToPoseTime = 0.0;

    //Vector3d originTipPositions[3][2];
    Vector3d midTipPositions[3][2];
    bool hasStepped[3][2];

    Vector3d originJointPositions[3][2];

    // Used in startup and shutdown sequences
    int sequenceStep = 0;
    double startHeightRatio;
    Vector3d phase1TipPositions[3][2];
    Vector3d phase2TipPositions[3][2];
    Vector3d phase3TipPositions[3][2];
    Vector3d phase4TipPositions[3][2];
    Vector3d phase5TipPositions[3][2];
    Vector3d phase6TipPositions[3][2];
    Vector3d phase7TipPositions[3][2];
    Vector3d phase8TipPositions[3][2];

    Pose targetPose;  // Target pose of body for use in manual posing bezier curve
    Pose originPose;  // Origin pos of body for use in manual posing bezier curve

    Pose manualPose;  // Current pose of body only using manual compensation
    Pose imuPose;  // Current pose of body only using imu compensation
    Pose inclinationPose;  // Current pose of body only using inclination compensation
    Pose deltaZPose;  // Current pose of body only using impedance control body height compensation

    Pose defaultPose;

    Pose autoPoseDefault;  // Current pose of body only using auto compensation
    Pose autoPose[3][2];  // Leg specific auto compensation - equal to default but with zero pose compensation on leg
			  // swing phase

    bool recalculateOffset = true;

    // Imu compensation PID error vectors
    Vector3d rotationAbsementError;
    Vector3d rotationPositionError;
    Vector3d rotationVelocityError;

    Vector3d translationAbsementError;
    Vector3d translationPositionError;
    Vector3d translationVelocityError;
    Vector3d translationAccelerationError;

    bool correctPose = false;  // Flag for correcting pose to a zero roll/pitch pose used for auto compensation
    double debugTime = 0.0;
};

class LegPoser
{
  public:
    LegPoser(PoseController* poser, Vector3d packed_joint_positions, Vector3d unpacked_joint_positions);
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getPackedJointPositions(void) { return packed_joint_positions_; };
    inline Vector3d getUnpackedJointPositions(void) { return unpacked_joint_positions_; };
    
    updatePosition(void); //apply current pose to generate new tip position
    moveToJointPosition(Vector3d targetJointPositions, double speed = 2.0); //move leg joints directly to target postion
    stepToPosition(Vector3d targetTipPosition, Pose targetPose);
    
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
