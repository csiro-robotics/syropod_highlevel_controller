#ifndef IMPEDANCECONTROLLER_H_
#define IMPEDANCECONTROLLER_H_

#include "standardIncludes.h"
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector< double > state_type;

class ImpedanceController
{
public:
	ImpedanceController(const Parameters &p);
	~ImpedanceController();
	
	void init(Parameters p);
	
	// Calculate and return adapted position of the feet in z-direction
	void updateImpedance(int l, int s, double effort[3][2], double deltaZ[3][2]);
	
	// Zero leg position for inititialzing
	void zeroLegMatrix(double inputMatrix[3][2]);
	
	void adjustStiffness(double stiffnessMultiplierArray[3][2]);
	
private:
	Parameters params;
	
	std::vector<std::vector<double> > tipForces;
	std::vector<std::vector<state_type> > impedanceState;
	
	double deltaT;
	double virtualMass[3][2];
	double virtualStiffness[3][2];
	double virtualDampingRatio[3][2];
	double forceGain;
	
	// Solve ODE of the impedance controller
	double calculateDeltaZ(int side, int leg);
};

#endif /* IMPEDANCECONTROLLER_H_ */