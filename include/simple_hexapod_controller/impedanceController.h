#ifndef IMPEDANCECONTROLLER_H_
#define IMPEDANCECONTROLLER_H_

#include "standardIncludes.h"
#include <boost/numeric/odeint.hpp>
#include "walkController.h"
#include "imuCompensation.h"

using namespace boost::numeric::odeint;

typedef std::vector< double > state_type;

class ImpedanceController
{
public:
	ImpedanceController(const Parameters &p);
	~ImpedanceController();
	
	Parameters params;
	
	void init(Parameters p);
	
	// Calculate and return adapted position of the feet in z-direction
	void updateImpedance(int l, int s, double effort[3][2], double deltaZ[3][2]);
	
	// Zero leg position for inititialzing
	void zeroLegMatrix(double inputMatrix[3][2]);
	
	void updateStiffness(WalkController *walker);
	void updateStiffness(Imu *imu, Vector3d identityTipPositions[3][2]);
	
	double virtualStiffness[3][2];	
	
	double zTipPositionError[3][2];
	double zTipVelocityError[3][2];
	double zTipAbsementError[3][2];
	
private:
	std::vector<std::vector<double> > tipForces;
	std::vector<std::vector<state_type> > impedanceState;
	
	int loadPhaseStart;
	int loadPhaseEnd;
	
	double deltaT;
	double virtualMass[3][2];
	
	double virtualDampingRatio[3][2];
	double forceGain;
	
	int phase[3][2];
	
	// Solve ODE of the impedance controller
	double calculateDeltaZ(int side, int leg);
};

#endif /* IMPEDANCECONTROLLER_H_ */