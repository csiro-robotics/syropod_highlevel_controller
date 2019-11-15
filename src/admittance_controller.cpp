////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../include/syropod_highlevel_controller/admittance_controller.h"
#include "../include/syropod_highlevel_controller/walk_controller.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AdmittanceController::AdmittanceController(std::shared_ptr<Model> model, const Parameters& params)
  : model_(model)
  , params_(params)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AdmittanceController::updateAdmittance(void)
{
  // Get current force value on leg and run admittance calculations to get a vertical tip offset (deltaZ)
  LegContainer::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    Eigen::Vector3d admittance_delta = Eigen::Vector3d::Zero();
    Eigen::Vector3d tip_force = params_.use_joint_effort.data ? leg->getTipForceCalculated():leg->getTipForceMeasured();
    tip_force *= params_.force_gain.current_value;
    for (int i = 0; i < 3; ++i)
    {
      double force_input = std::max(tip_force[i], 0.0); // Use vertical component of tip force vector //TODO
      double damping = params_.virtual_damping_ratio.current_value;
      double stiffness = params_.virtual_stiffness.current_value;
      double mass = params_.virtual_mass.current_value;
      double step_time = params_.integrator_step_time.data;
      state_type* admittance_state = leg->getAdmittanceState();
      double virtual_damping = damping * 2 * sqrt(mass * stiffness);
      boost::numeric::odeint::runge_kutta4<state_type> stepper;
      integrate_const(stepper,
                      [&](const state_type & x, state_type & dxdt, double t)
                      {
                        dxdt[0] = x[1];
                        dxdt[1] = -force_input / mass - virtual_damping / mass * x[1] - stiffness / mass * x[0];
                      }, 
                      *admittance_state,
                      0.0,
                      step_time,
                      step_time / 30);
      
      // Deadbanding
      double delta = clamped(-(*admittance_state)[0], -0.2, 0.2);
      double delta_direction = delta / abs(delta);
      if (abs(delta) > ADMITTANCE_DEADBAND)
      {
        admittance_delta[i] = delta_direction * (abs(delta) - ADMITTANCE_DEADBAND) / (1 - ADMITTANCE_DEADBAND);
      }
    }
    leg->setAdmittanceDelta(admittance_delta);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AdmittanceController::updateStiffness(std::shared_ptr<Leg> leg, const double& scale_reference)
{
  int leg_id = leg->getIDNumber();
  int adjacent_leg_1_id = mod(leg_id - 1, model_->getLegCount());
  int adjacent_leg_2_id = mod(leg_id + 1, model_->getLegCount());
  std::shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
  std::shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);

  // (X-1)+1 to change range from 0->1 to 1->scaler
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AdmittanceController::updateStiffness(std::shared_ptr<WalkController> walker)
{
  // Reset virtual Stiffness each cycle
  LegContainer::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    leg->setVirtualStiffness(params_.virtual_stiffness.current_value);
  }

  // Calculate dynamic virtual stiffness
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    std::shared_ptr<Leg> leg = leg_it->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg_stepper->getStepState() == SWING)
    {
      double z_diff = leg_stepper->getCurrentTipPose().position_[2] - leg_stepper->getDefaultTipPose().position_[2];
      double step_reference = 0;
      step_reference += abs(z_diff / walker->getStepClearance());

      int leg_id = leg->getIDNumber();
      int adjacent_leg_1_id = mod(leg_id - 1, model_->getLegCount());
      int adjacent_leg_2_id = mod(leg_id + 1, model_->getLegCount());
      std::shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
      std::shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);

      // (X-1)+1 to change range from 0->1 to 1->scaler
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////