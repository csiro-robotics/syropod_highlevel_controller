#ifndef SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H
/** 
 *  \file    walk_controller.h
 *  \brief   Handles control of hexapod walking. Part of simple hexapod controller.
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
#include "pose.h"

#include "model.h"

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
    inline double getBodyHeight(void) { return body_clearance_ * maximum_body_height_; };
    inline WalkState getWalkState(void) { return walk_state_; };
    inline Vector2d getCurrentLinearVelocity(void) { return current_linear_velocity_; };
    inline double getCurrentAngularVelocity(void) { return current_angular_velocity_; };
    
    void init(void);
    void setGaitParams(Parameters* p);
    void updateWalk(Vector2d linear_velocity_input, double angular_velocity_input);
    void updateManual(int primary_leg_selection_ID, Vector3d primary_tip_velocity_input,
		      int secondary_leg_selection_ID, Vector3d secondary_tip_velocity_input);

  private:
    Model* model_;
    Parameters* params_;
    WalkState walk_state_;
    double time_delta_;

    // Walk parameters
    double step_frequency_;
    double step_clearance_;
    double step_depth_;
    double body_clearance_;

    int phase_length_;
    int stance_end_;
    int swing_start_;
    int swing_end_;
    int stance_start_;

    Vector3d foot_spread_distances_;
    double min_footprint_radius_;
    double stance_radius_;

    Vector2d current_linear_velocity_;  // Linear Body Velocity //TBD put in model?
    double current_angular_velocity_;  // Angular Body Velocity

    double maximum_body_height_;

    int legs_at_correct_phase_ = 0;
    int legs_completed_first_step_ = 0; 
};

/***********************************************************************************************************************
 * Leg specific controller which walks leg through step cycle
***********************************************************************************************************************/
class LegStepper
{
  public:
    LegStepper(WalkController* walker, Leg* leg, Vector3d identity_tip_position);    
    
    //Accessors
    inline Vector3d getCurrentTipPosition(void) { return current_tip_position_; };    
    inline Vector3d getDefaultTipPosition(void) { return default_tip_position_;};    
    inline StepState getStepState(void) { return step_state_; };
    inline int getPhase(void) { return phase_; };    
    inline int getPhaseOffset(void) { return phase_offset_; };    
    inline Vector2d getStrideVector(void) { return stride_vector_; };
    inline double getSwingHeight(void) { return swing_height_; };
    inline double getStanceDepth(void) { return stance_depth_; };  
    inline double getSwingProgress(void) { return swing_progress_; };
    inline double getStanceProgress(void) { return stance_progress_; };
    inline bool hasCompletedFirstStep(void) { return completed_first_step_; };    
    inline bool isAtCorrectPhase(void) { return at_correct_phase_; };    
    
    //Mutators
    inline void setCurrentTipPosition(Vector3d current_tip_position) { current_tip_position_ = current_tip_position; };
    inline void setStepState(StepState stepState) { step_state_ = stepState; };
    inline void setPhase(int phase) { phase_ = phase; };
    inline void setPhaseOffset(int phase_offset) { phase_offset_ = phase_offset;};
    inline void setStrideVector(Vector2d stride_vector) { stride_vector_ = stride_vector; };
    inline void setCompletedFirstStep(bool completed_first_step) { completed_first_step_ = completed_first_step; };
    inline void setAtCorrectPhase(bool at_correct_phase) { at_correct_phase_ = at_correct_phase; };
    
    void updatePosition(void);
    void generateSwingControlNodes(Vector3d stride_vector);
    void generateStanceControlNodes(Vector3d stride_vector);
    void iteratePhase(void);
    double calculateDeltaT(int length);
  
  private:
    WalkController* walker_;
    Leg* leg_;
    
    bool at_correct_phase_ = false;
    bool completed_first_step_ = false;

    int phase_ = 0;
    int phase_offset_;

    double swing_progress_;
    double stance_progress_;

    StepState step_state_ = STANCE;

    Vector3d swing_1_nodes_[5];  // Primary swing bezier curve
    Vector3d swing_2_nodes_[5];  // Secondary swing bezier curve
    Vector3d stance_nodes_[5];  // Stance bezier curve

    double swing_delta_t_;
    double stance_delta_t_;

    Vector2d stride_vector_;  // length gives stride length  
    double swing_height_;
    double stance_depth_;
    
    Vector3d default_tip_position_;
    Vector3d current_tip_position_;
    Vector3d current_tip_velocity_;    
    Vector3d swing_origin_tip_position_;
    Vector3d stance_origin_tip_position_;
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_WALK_CONTROLLER_H */