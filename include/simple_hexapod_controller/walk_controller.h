#ifndef SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H
/*******************************************************************************************************************//**
 *  \file    walk_controller.h
 *  \brief   Handles control of hexapod walking. Part of simple hexapod controller.
 *
 *  \author  Fletcher Talbot
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
 **********************************************************************************************************************/

#include "standard_includes.h"
#include "parameters_and_states.h"
#include "pose.h"

#include "model.h"
#include "pose_controller.h"
#include "debug_output.h"

#define JOINT_POSITION_ITERATION 0.001 // Joint position iteration value used to find optimal angle (rad)

/***********************************************************************************************************************
 * Top level controller that calculates walk characteristics and coordinates leg specific walk controllers
***********************************************************************************************************************/
class WalkController
{  
public:
  WalkController(Model* model, Parameters* params);
  
  inline Parameters* getParameters(void) { return params_; };
  inline int getPhaseLength(void) { return phase_length_; };
  inline int getSwingStart(void) { return swing_start_; };
  inline int getSwingEnd(void) { return swing_end_; };
  inline int getStanceStart(void) { return stance_start_; };
  inline int getStanceEnd(void) { return stance_end_; };
  inline double getTimeDelta(void) { return time_delta_; };
  inline double getStepFrequency(void) { return step_frequency_; };
  inline double getStepClearance(void) { return step_clearance_; };
  inline double getMaxBodyHeight(void) { return maximum_body_height_; };
  inline double getStepDepth(void) { return step_depth_; };
  inline double getBodyHeight(void) { return body_clearance_; };
  inline double getWorkspaceRadius(void) { return workspace_radius_; };
  inline double getStanceRadius(void) { return stance_radius_; };
  inline Vector2d getDesiredLinearVelocity(void) { return desired_linear_velocity_; };
  inline double getDesiredAngularVelocity(void) { return desired_angular_velocity_; };
  inline double getStrideLength(void) { return stride_length_; };
  inline WalkState getWalkState(void) { return walk_state_; };
  inline void resetJointOrientationTracking(void) { joint_twist_ = 0.0, ground_bearing_ = 1.57; };
  inline void setPoseState(PosingState state) { pose_state_ = state; };

  void init(void);
  void setGaitParams(Parameters* p);
  void updateWalk(Vector2d linear_velocity_input, double angular_velocity_input);
  void updateManual(int primary_leg_selection_ID, Vector3d primary_tip_velocity_input,
                    int secondary_leg_selection_ID, Vector3d secondary_tip_velocity_input);

private:
  Model* model_;
  Parameters* params_;
  WalkState walk_state_;
  PosingState pose_state_ = POSING_COMPLETE;
  double time_delta_;
  double time_elapsed_=0;

  //Joint orientation tracking variables
  double joint_twist_;
  double ground_bearing_;

  // Walk parameters
  double step_frequency_;
  double step_clearance_;
  double step_depth_;
  double body_clearance_;

  // Gait cycle parameters
  int phase_length_;
  int swing_length_;
  int stance_length_;
  int stance_end_;
  int swing_start_;
  int swing_end_;
  int stance_start_;

  // Workspace variables
  double maximum_body_height_ = UNASSIGNED_VALUE;
  double workspace_radius_;
  double stride_length_ = 0.0;
  double stance_radius_;

  // Velocity/acceleration variables
  Vector2d desired_linear_velocity_;  // Desired linear body velocity
  double desired_angular_velocity_;  // Angular Body Velocity
  double max_linear_speed_ = 0.0;
  double max_angular_speed_ = 0.0;
  double max_linear_acceleration_ = 0.0;
  double max_angular_acceleration_ = 0.0;

  // Leg coordination variables
  int legs_at_correct_phase_ = 0;
  int legs_completed_first_step_ = 0; 

  // Iteration variables
  map<int, Leg*>::iterator leg_it_;
  map<int, Joint*>::iterator joint_it_;
  map<int, Link*>::iterator link_it_;
};

/***********************************************************************************************************************
 * Leg specific controller which walks leg through step cycle
***********************************************************************************************************************/
class LegStepper
{
  public:
    LegStepper(WalkController* walker, Leg* leg, Vector3d identity_tip_position);

    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };
    inline Vector3d getDefaultTipPosition(void) { return default_tip_position_;};
    inline WalkState getWalkState(void) { return walker_->getWalkState(); };
    inline StepState getStepState(void) { return step_state_; };
    inline int getPhase(void) { return phase_; };
    inline int getPhaseOffset(void) { return phase_offset_; };
    inline Vector2d getStrideVector(void) { return Vector2d(stride_vector_[0], stride_vector_[1]); };
    inline double getSwingHeight(void) { return swing_height_; };
    inline double getStanceDepth(void) { return stance_depth_; };
    inline double getSwingProgress(void) { return swing_progress_; };
    inline double getStanceProgress(void) { return stance_progress_; };
    inline bool hasCompletedFirstStep(void) { return completed_first_step_; };
    inline bool isAtCorrectPhase(void) { return at_correct_phase_; };

    inline Vector3d getSwing1ControlNode(int i) { return swing_1_nodes_[i]; };
    inline Vector3d getSwing2ControlNode(int i) { return swing_2_nodes_[i]; };
    inline Vector3d getStanceControlNode(int i) { return stance_nodes_[i]; };

    inline void setCurrentTipPosition(Vector3d current_tip_position) { current_tip_position_ = current_tip_position; };
    inline void setStepState(StepState stepState) { step_state_ = stepState; };
    inline void setPhase(int phase) { phase_ = phase; };
    inline void setPhaseOffset(int phase_offset) { phase_offset_ = phase_offset;};
    inline void setCompletedFirstStep(bool completed_first_step) { completed_first_step_ = completed_first_step; };
    inline void setAtCorrectPhase(bool at_correct_phase) { at_correct_phase_ = at_correct_phase; };
    inline void updateStride(Vector2d stride_vector)
    {
      stride_vector_ = Vector3d(stride_vector[0], stride_vector[1], 0.0);
    };

    void updatePosition(void);
    void generateSwingControlNodes(double bezier_scaler);
    void generateStanceControlNodes(Vector3d stride_vector);
    void iteratePhase(void);
    double calculateDeltaT(StepState step_state, int length);
  
  private:
    WalkController* walker_;
    Leg* leg_;

    bool at_correct_phase_ = false;
    bool completed_first_step_ = false;

    int phase_ = 0;
    int phase_offset_;

    double swing_progress_ = -1.0;
    double stance_progress_ = -1.0;

    StepState step_state_ = STANCE;

    Vector3d swing_1_nodes_[5]; // Primary swing bezier curve
    Vector3d swing_2_nodes_[5]; // Secondary swing bezier curve
    Vector3d stance_nodes_[5];  // Stance bezier curve

    Vector3d stride_vector_;
    double swing_height_;
    double stance_depth_;

    Vector3d default_tip_position_;
    Vector3d current_tip_position_; // Current tip position according to the walk controller
    Vector3d current_tip_velocity_; // Current tip velocity according to the walk controller 
    Vector3d swing_origin_tip_position_;
    Vector3d stance_origin_tip_position_;
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H */
