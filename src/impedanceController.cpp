#include "../include/simple_hexapod_controller/impedanceController.h"

/***********************************************************************************************************************
 * Constructor
***********************************************************************************************************************/
ImpedanceController::ImpedanceController(Model* model, Parameters* params)
  : model_(model)
  , params_(params)
{
  init();
}

/***********************************************************************************************************************
 * Initialisation
***********************************************************************************************************************/
void ImpedanceController::init(void)
{
  delta_t_ = params_->integrator_step_time.data;
  force_gain_ = params_->force_gain.current_value;
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    leg->setVirtualMass(params_->virtual_mass.current_value);
    leg->setVirtualStiffness(params_->virtual_stiffness.current_value);
    leg->setVirtualDampingRatio(params_->virtual_damping_ratio.current_value);
  }
}

/***********************************************************************************************************************
 * Calculates change in tip position in z direction (deltaZ) according to tip force value
***********************************************************************************************************************/
void ImpedanceController::updateImpedance(Leg* leg, bool use_joint_effort)
{
	if (use_joint_effort)
	{
		leg->setTipForce(leg->getJointByIDNumber(2)->current_effort * leg->getMirrorDir()); //TBD Refactor
	}
    
  double force_input = abs(max(leg->getTipForce(), 0.0));

	double damping = leg->getVirtualDampingRatio();
	double stiffness = leg->getVirtualStiffness();
	double mass = leg->getVirtualMass();
	state_type* impedance_state = leg->getImpedanceState();
	double virtual_damping = damping * 2 * sqrt(mass * stiffness);
	runge_kutta4<state_type> stepper;
	integrate_const(stepper,
	                [&](const state_type &x, state_type &dxdt, double t)
	                {
	                  dxdt[0] = x[1];
	                  dxdt[1] = -force_input/mass*force_gain_ - virtual_damping/mass*x[1] - stiffness/mass*x[0];
	                },
	                *impedance_state, 0.0, delta_t_, delta_t_ / 30);
	double delta_z = -(*impedance_state)[0];
	leg->setDeltaZ(delta_z);
}

/***********************************************************************************************************************
 * Scales virtual stiffness value of given and adjacent legs according to reference
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(Leg* leg, double step_reference)
{
  int leg_id = leg->getIDNumber();
  int adjacent_leg_1_id = mod(leg_id-1, model_->getLegCount()); 
  int adjacent_leg_2_id = mod(leg_id+1, model_->getLegCount());
  Leg* adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
  Leg* adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);  

  //(X-1)+1 to change range from 0->1 to 1->multiplier
  double virtual_stiffness = params_->virtual_stiffness.current_value;
  double swing_stiffness = virtual_stiffness*(step_reference*(params_->swing_stiffness_scaler.data-1)+1);
  double load_stiffness = virtual_stiffness*(step_reference*(params_->load_stiffness_scaler.data-1)+1);
  
  leg->setVirtualStiffness(swing_stiffness);
  adjacent_leg_1->setVirtualStiffness(load_stiffness);
  adjacent_leg_2->setVirtualStiffness(load_stiffness);
}

/***********************************************************************************************************************
 * Scales virtual stiffness value of legs adjacent to swinging leg according to swing cycle percentage
 * Note: The reseting and addition of stiffness allows overlapping step cycles to JOINTLY add stiffness to
 * simultaneously adjacent legs
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(WalkController *walker)
{
  // Reset virtual Stiffness each cycle
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    leg->setVirtualStiffness(params_->virtual_stiffness.current_value);
  }

  // Calculate dynamic virtual stiffness
  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    LegStepper* leg_stepper = leg->getLegStepper();
    if (leg_stepper->getStepState() == SWING)
    {
      double z_diff = leg_stepper->getCurrentTipPosition()[2] - leg_stepper->getDefaultTipPosition()[2];
      double step_reference = 0;
      step_reference += abs(z_diff / walker->getStepClearance());

      int leg_id = leg->getIDNumber();
      int adjacent_leg_1_id = mod(leg_id-1,model_->getLegCount()); 
      int adjacent_leg_2_id = mod(leg_id+1,model_->getLegCount());
      Leg* adjacent_leg_1 = model_->getLegByIDNumber(adjacent_leg_1_id);
      Leg* adjacent_leg_2 = model_->getLegByIDNumber(adjacent_leg_2_id);
      
      //(X-1)+1 to change range from 0->1 to 1->multiplier
      double virtual_stiffness = params_->virtual_stiffness.current_value;
      double swing_stiffness = virtual_stiffness*(step_reference*(params_->swing_stiffness_scaler.data-1)+1);
      double load_stiffness = virtual_stiffness*(step_reference*(params_->load_stiffness_scaler.data-1));
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
