#include "../include/simple_hexapod_controller/impedanceController.h"

/***********************************************************************************************************************
 * Constructor
***********************************************************************************************************************/
ImpedanceController::ImpedanceController(const Parameters &p): 
  tipForces(3, std::vector<double>(2)), 
  impedanceState(3, std::vector<std::vector<double>>(2, std::vector<double>(2)))
{
  init(p);
}

/***********************************************************************************************************************
 * Destructor
***********************************************************************************************************************/
ImpedanceController::~ImpedanceController()
{
}

/***********************************************************************************************************************
 * Initialisation
***********************************************************************************************************************/
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
      
      zTipPositionError[l][s] = 0;
      zTipVelocityError[l][s] = 0;
      zTipAbsementError[l][s] = 0;
    }
  }  
}

/***********************************************************************************************************************
 * Updates change in tip position in z direction (deltaZ) according to tip force value
***********************************************************************************************************************/
void ImpedanceController::updateImpedance(int l, int s, double effort[3][2], double deltaZ[3][2])
{
  tipForces[l][s] = effort[l][s];
  deltaZ[l][s] = calculateDeltaZ(l,s);
}

/***********************************************************************************************************************
 * Calculates change in tip position in z direction (deltaZ) according to tip force value
***********************************************************************************************************************/
double ImpedanceController::calculateDeltaZ(int l, int s)
{
  double virtualDamping = virtualDampingRatio[l][s]*2*sqrt(virtualMass[l][s]*virtualStiffness[l][s]);
  runge_kutta4<state_type> stepper;  
  integrate_const(stepper, 
		  [&]( const state_type &x, state_type &dxdt , double t ) 
		  {
		    dxdt[0] = x[1]; 
		    dxdt[1] = -tipForces[l][s]/virtualMass[l][s]*forceGain - 
			      virtualDamping/virtualMass[l][s]*x[1] - 
			      virtualStiffness[l][s]/virtualMass[l][s]*x[0];
		  },
		  impedanceState[l][s], 0.0, deltaT, deltaT/30);
  return impedanceState[l][s][0];
}

/***********************************************************************************************************************
 * Sets input matrix to zero
***********************************************************************************************************************/
void ImpedanceController::zeroLegMatrix(double inputMatrix[3][2])
{
  inputMatrix[0][0] = 0;
  inputMatrix[0][1] = 0;
  inputMatrix[1][0] = 0;
  inputMatrix[1][1] = 0;
  inputMatrix[2][0] = 0;
  inputMatrix[2][1] = 0;
}

/***********************************************************************************************************************
 * Scales virtual stiffness value of legs adjacent to swinging leg according to swing cycle percentage
 * Note: The reseting and addition of stiffness allows overlapping step cycles to JOINTLY add stiffness to 
 * simultaneously adjacent legs
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(WalkController *walker)
{  
  //Reset virtual Stiffness each cycle
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      virtualStiffness[l][s] = params.virtualStiffness;
    }
  }
  
  //Calculate dynamic virtual stiffness
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      if (walker->legSteppers[l][s].state == SWING)
      {
	double zDiff = walker->legSteppers[l][s].currentTipPosition[2] - walker->legSteppers[l][s].defaultTipPosition[2];
	double stiffnessMultiplier = 0;
	stiffnessMultiplier += abs(zDiff/(walker->stepClearance*walker->maximumBodyHeight))*(params.stiffnessMultiplier-1); //Change range from 0->1 to 1->multiplier
	
	//Get adjacent leg references
	int adjacent1Leg = (l==1) ? (l+1):l;
	int adjacent1Side = (l==1) ? s:!s;
	int adjacent2Leg = (l==0) ? (l+1):(l-1);
	int adjacent2Side = s;
	
	virtualStiffness[l][s] = params.virtualStiffness/(stiffnessMultiplier+1);
	
	virtualStiffness[adjacent1Leg][adjacent1Side] += params.virtualStiffness*stiffnessMultiplier;
	virtualStiffness[adjacent2Leg][adjacent2Side] += params.virtualStiffness*stiffnessMultiplier;
      }
    }
  }
}

/***********************************************************************************************************************
 * Scales virtual stiffness value according to current pose offset calculated from IMU readings
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(Pose currentPose, Vector3d identityTipPositions[3][2])
{  
  Vector3d posedTipPositions[3][2];
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {  
      double proportionalGain = 1000; //TBD Parameterise
      double integralGain = 0; //TBD Parameterise
      double derivativeGain = 0; //TBD Parameterise
      
      posedTipPositions[l][s] = currentPose.inverseTransformVector(identityTipPositions[l][s]);
      
      double oldTipPositionError = zTipPositionError[l][s];
      zTipPositionError[l][s] = identityTipPositions[l][s][2] - posedTipPositions[l][s][2];
      zTipAbsementError[l][s] += zTipPositionError[l][s]*params.timeDelta;
      
      //Low pass filter of IMU angular velocity data
      double smoothingFactor = 0.15;
      zTipVelocityError[l][s] = smoothingFactor*(zTipPositionError[l][s] - oldTipPositionError)/params.timeDelta + (1-smoothingFactor)*zTipVelocityError[l][s];
      
      double minStiffness = params.virtualStiffness; 
      
      virtualStiffness[l][s] = max(zTipPositionError[l][s]*proportionalGain + zTipAbsementError[l][s]*integralGain + zTipVelocityError[l][s]*derivativeGain, minStiffness);
    }
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
