#include "../include/simple_hexapod_controller/impedanceController.h"

ImpedanceController::ImpedanceController(const Parameters &p): 
  TIP_FORCE(3, std::vector<double >(2)), 
  IMPEDANCE_STATE(3, std::vector<std::vector<double > >(2, std::vector<double >(2))), 
  DELTA_Z(3, std::vector<double >(2))
{
  init(p);
}

ImpedanceController::~ImpedanceController()
{
}

void ImpedanceController::init(Parameters p)
{
  PARAMS = p;
  DELTA_T = PARAMS.integratorStepTime;
  VIRT_MASS = PARAMS.virtualMass;
  VIRT_STIFF = PARAMS.virtualStiffness;
  VIRT_DAMP = PARAMS.virtualDampingRatio*2*sqrt(VIRT_MASS*VIRT_STIFF);
  FORCE_GAIN = PARAMS.forceGain;
}

std::vector<std::vector<double> > &ImpedanceController::updateImpedance(const std::vector<std::vector<double> > &effort) 
{
  for (int s = 0; s<2; ++s)
  {
    for (int l = 0; l<3; ++l)
    {
      TIP_FORCE[l][s] = effort[l][s] ;
      calculateDeltaZ(l,s);
    }
  }
  return DELTA_Z;
}

void ImpedanceController::calculateDeltaZ(int leg, int side) 
{
  // Solving the ODE
  runge_kutta4< state_type > stepper;
  integrate_const( stepper , [&]( const state_type &x, state_type &dxdt , double t ) {
          dxdt[0] = x[1]; dxdt[1] = -this->TIP_FORCE[leg][side]/this->VIRT_MASS*FORCE_GAIN-this->VIRT_DAMP/this->VIRT_MASS*x[1]-this->VIRT_STIFF/this->VIRT_MASS*x[0]; }
        , IMPEDANCE_STATE[leg][side] , 0.0 , DELTA_T , DELTA_T/30 );
  DELTA_Z[leg][side] = IMPEDANCE_STATE[leg][side][0];
}

std::vector<std::vector<double> > ImpedanceController::returnZeroLegMatrix(void) const
{
  std::vector<std::vector<double> > zeroLegMatrix(3, std::vector<double >(2));
  zeroLegMatrix[0][0] = 0;
  zeroLegMatrix[0][1] = 0;
  zeroLegMatrix[1][0] = 0;
  zeroLegMatrix[1][1] = 0;
  zeroLegMatrix[2][0] = 0;
  zeroLegMatrix[2][1] = 0;
  return zeroLegMatrix;
}