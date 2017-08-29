/*******************************************************************************************************************//**
 *  @file    impedance_controller.cpp
 *  @brief   Handles executiong of the impedance controller.
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

#include "../include/syropod_highlevel_controller/impedance_controller.h"

/*******************************************************************************************************************//**
 * ImpedanceController class constructor. Assigns pointers to robot model object and parameter data storage object.
 * @param[in] model Pointer to the robot model class object
 * @param[in] params Pointer to the parameter struct object
***********************************************************************************************************************/
ImpedanceController::ImpedanceController(shared_ptr<Model> model, const Parameters& params)
  : model_(model)
  , params_(params)
{
}

/*******************************************************************************************************************//**
 * Impdedance controller initialisation. For each leg sets virtual mass/stiffness/damping ratio from parameters.
***********************************************************************************************************************/
void ImpedanceController::init(void)
{
  LegContainer::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    leg->setVirtualMass(params_.virtual_mass.current_value);
    leg->setVirtualStiffness(params_.virtual_stiffness.current_value);
    leg->setVirtualDampingRatio(params_.virtual_damping_ratio.current_value);
  }
}

/*******************************************************************************************************************//**
 * Iterates through legs in the robot model and updates the vertical tip position offset value (delta_z) for each.
 * The calculation of delta_z is achieved through the use of a classical Runge-Kutta ODE solver with a force input
 * acquired from a tip force callback OR from a joint effort value.
 * @param[in] use_joint_effort Bool which determines whether the tip force input is derived from joint effort
 * @todo Implement impedance control in x/y axis
***********************************************************************************************************************/
void ImpedanceController::updateImpedance(const bool& use_joint_effort)
{
  // Get current force value on leg and run impedance calculations to get a vertical tip offset (deltaZ)
  LegContainer::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    if (use_joint_effort)
    {
      leg->calculateTipForce();
    }

    double force_input = min(leg->getTipForce()[2], 0.0); // Use vertical component of tip force vector //TODO
    double damping = leg->getVirtualDampingRatio();
    double stiffness = leg->getVirtualStiffness();
    double mass = leg->getVirtualMass();
    double step_time = params_.integrator_step_time.data;
    state_type* impedance_state = leg->getImpedanceState();
    double virtual_damping = damping * 2 * sqrt(mass * stiffness);
    boost::numeric::odeint::runge_kutta4<state_type> stepper;
    integrate_const(stepper,
                    [&](const state_type & x, state_type & dxdt, double t)
                    {
                      dxdt[0] = x[1];
                      dxdt[1] = -force_input / mass - virtual_damping / mass * x[1] - stiffness / mass * x[0];
                    }, 
                    *impedance_state,
                    0.0,
                    step_time,
                    step_time / 30);
    leg->setDeltaZ((*impedance_state)[0]);
  }
}

/*******************************************************************************************************************//**
 * Scales virtual stiffness of an input swinging leg and two 'adjacent legs' according to an input reference.
 * The input reference ranges between 0.0 and 1.0, and defines the characteristic of the curve as stiffness changes
 * from the default stiffness (i.e. scaled by 1.0) to the swing/load stiffness (i.e. scaled by swing/load scaling value)
 * The swing stiffness value is applied to the swinging leg and load stiffness value applied to the two adjacent legs.
 * @param[in] leg A pointer to the leg object associated with the stiffness value to be updated.
 * @param[in] scale_reference A double ranging from 0.0->1.0 which controls the scaling of the stiffnesses
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(shared_ptr<Leg> leg, const double& scale_reference)
{
  int leg_id = leg->getIDNumber();
  int adjacent_leg_1_id = mod(leg_id - 1, model_->getLegCount());
  int adjacent_leg_2_id = mod(leg_id + 1, model_->getLegCount());
  shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
  shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);

  //(X-1)+1 to change range from 0->1 to 1->scaler
  double virtual_stiffness = params_.virtual_stiffness.current_value;
  double swing_stiffness = virtual_stiffness * (scale_reference * (params_.swing_stiffness_scaler.data - 1) + 1);
  double load_stiffness = virtual_stiffness * (scale_reference * (params_.load_stiffness_scaler.data - 1) + 1);

  leg->setVirtualStiffness(swing_stiffness);

  if (adjacent_leg_1->getLegState() != MANUAL)
  {
    adjacent_leg_1->setVirtualStiffness(load_stiffness);
  }

  if (adjacent_leg_2->getLegState() != MANUAL)
  {
    adjacent_leg_2->setVirtualStiffness(load_stiffness);
  }
}

/*******************************************************************************************************************//**
 * Scales virtual stiffness of swinging leg and adjacent legs according to the walk cycle of the walk controller.
 * The percentage vertical position difference of the swinging leg tip from it's default position is used as a reference
 * value ranging from 0.0 (default tip height) to 1.0 (max swing height). This reference value defines the
 * characteristic of the curve as stiffness changes from the default stiffness (i.e. scaled by 1.0) to the swing/load
 * stiffness (i.e. scaled by swing/load scaling value). The swing stiffness value is applied to the swinging leg and
 * load stiffness value applied to the two adjacent legs. The reseting and addition of stiffness allows overlapping
 * step cycles to JOINTLY add stiffness to simultaneously adjacent legs.
 * @param[in] walker A pointer to the walk controller
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(shared_ptr<WalkController> walker)
{
  // Reset virtual Stiffness each cycle
  LegContainer::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    leg->setVirtualStiffness(params_.virtual_stiffness.current_value);
  }

  // Calculate dynamic virtual stiffness
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    shared_ptr<Leg> leg = leg_it->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg_stepper->getStepState() == SWING)
    {
      double z_diff = leg_stepper->getCurrentTipPosition()[2] - leg_stepper->getDefaultTipPosition()[2];
      double step_reference = 0;
      step_reference += abs(z_diff / walker->getStepClearance());

      int leg_id = leg->getIDNumber();
      int adjacent_leg_1_id = mod(leg_id - 1, model_->getLegCount());
      int adjacent_leg_2_id = mod(leg_id + 1, model_->getLegCount());
      shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
      shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);

      //(X-1)+1 to change range from 0->1 to 1->scaler
      double virtual_stiffness = params_.virtual_stiffness.current_value;
      double swing_stiffness = virtual_stiffness * (step_reference * (params_.swing_stiffness_scaler.data - 1) + 1);
      double load_stiffness = virtual_stiffness * (step_reference * (params_.load_stiffness_scaler.data - 1));
      double current_stiffness_1 = adjacent_leg_1->getVirtualStiffness();
      double current_stiffness_2 = adjacent_leg_2->getVirtualStiffness();
      leg->setVirtualStiffness(swing_stiffness);
      adjacent_leg_1->setVirtualStiffness(current_stiffness_1 + load_stiffness);
      adjacent_leg_2->setVirtualStiffness(current_stiffness_2 + load_stiffness);
    }
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
