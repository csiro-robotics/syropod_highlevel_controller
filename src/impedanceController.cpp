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

void ImpedanceController::updateStiffness(WalkController *walker)
{  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      //For more future control of stiffness - use bezier curves with leg specific control points
      int basePhaselength = params.unloadedPhase + params.loadedPhase;
      int normaliser = walker->phaseLength/basePhaselength;
      int index = 2*l+s;
      int offset = params.stiffnessPhaseOffset*params.stiffnessOffsetMultiplier[index];
      
      int loadPhaseStart = params.unloadedPhase*0.5*normaliser;
      int loadPhaseEnd = loadPhaseStart + params.loadedPhase*normaliser;
      
      int phase = (walker->legSteppers[0][0].phase + offset*normaliser)%walker->phaseLength;
      /*
      if (s==1 && l==1)
	cout << "\t11: " << walker->legSteppers[0][0].phase << " - " << phase;
      
      if (s==0 && l==1)
	cout << " \t10: " << walker->legSteppers[0][0].phase << " - " << phase;
      */
      if (walker->state != STOPPED && params.stiffnessOffsetMultiplier[index] >= 0)
      {	
	if (phase > loadPhaseStart && phase < loadPhaseEnd)
	{
	  virtualStiffness[l][s] = params.virtualStiffness * params.stiffnessMultiplier;
	}
	else
	{
	  virtualStiffness[l][s] = params.virtualStiffness;
	}
      }
      else
      {
	virtualStiffness[l][s] = params.virtualStiffness;
      }
    }
  }
}
