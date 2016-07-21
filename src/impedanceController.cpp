#include "../include/simple_hexapod_controller/impedanceController.h"

ImpedanceController::ImpedanceController(const Parameters &p): 
  tipForces(3, std::vector<double>(2)), 
  impedanceState(3, std::vector<std::vector<double>>(2, std::vector<double>(2)))
{
  init(p);
}

ImpedanceController::~ImpedanceController()
{
}

void ImpedanceController::init(Parameters p)
{
  params = p;
  deltaT = params.integratorStepTime;
  forceGain = params.forceGain;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      virtualMass[l][s] = params.virtualMass;
      virtualStiffness[l][s] = params.virtualStiffness;
      virtualDampingRatio[l][s] = params.virtualDampingRatio;
    }
  }  
}

void ImpedanceController::updateImpedance(int l, int s, double effort[3][2], double deltaZ[3][2])
{
  tipForces[l][s] = effort[l][s];
  deltaZ[l][s] = calculateDeltaZ(l,s);
}

double ImpedanceController::calculateDeltaZ(int l, int s)
{
  double virtualDamping = virtualDampingRatio[l][s]*2*sqrt(virtualMass[l][s]*virtualStiffness[l][s]);
  runge_kutta4<state_type> stepper;  
  integrate_const(stepper, 
		  [&]( const state_type &x, state_type &dxdt , double t ) 
		  {
		    dxdt[0] = x[1]; 
		    dxdt[1] = -tipForces[l][s]/virtualMass[l][s]*forceGain - virtualDamping/virtualMass[l][s]*x[1] - virtualStiffness[l][s]/virtualMass[l][s]*x[0];
		  },
		  impedanceState[l][s], 0.0, deltaT, deltaT/30);
  return impedanceState[l][s][0];
}

void ImpedanceController::zeroLegMatrix(double inputMatrix[3][2])
{
  inputMatrix[0][0] = 0;
  inputMatrix[0][1] = 0;
  inputMatrix[1][0] = 0;
  inputMatrix[1][1] = 0;
  inputMatrix[2][0] = 0;
  inputMatrix[2][1] = 0;
}

void ImpedanceController::adjustStiffness(double stiffnessMultiplierArray[3][2])
{
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      virtualStiffness[l][s] = params.virtualStiffness*stiffnessMultiplierArray[l][s];
    }
  }  
}
