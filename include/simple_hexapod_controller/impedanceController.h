#ifndef IMPEDANCECONTROLLER_H_
#define IMPEDANCECONTROLLER_H_

#include "standardIncludes.h"
#include "parametersAndStates.h"
#include <boost/numeric/odeint.hpp>
#include "walkController.h"

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

class ImpedanceController
{
public:
  ImpedanceController(const Parameters &p);
  ~ImpedanceController();

  Parameters parameters_;

  void init(Parameters p);

  // Calculate and return adapted position of the feet in z-direction
  void updateImpedance(int l, int s, double effort[3][2], double delta_z[3][2]);

  // Zero leg position for inititialzing
  void zeroLegMatrix(double input_matrix[3][2]);

  void updateStiffness(double step_reference, int l, int s);
  void updateStiffness(WalkController *walker);
  void updateStiffness(Pose current_pose, Vector3d identity_tip_positions[3][2]);

  double virtualStiffness[3][2];

  double zTipPositionError[3][2];
  double zTipVelocityError[3][2];
  double zTipAbsementError[3][2];

private:
  std::vector<std::vector<double> > tip_forces_;
  std::vector<std::vector<state_type> > impedance_state_;

  int load_phase_start_;
  int load_phase_end_;

  double delta_t_;
  double virtual_mass_[3][2];

  double virtual_damping_ratio_[3][2];
  double force_gain_;

  int phase_[3][2];

  // Solve ODE of the impedance controller
  double calculateDeltaZ(int side, int leg);
};

#endif /* IMPEDANCECONTROLLER_H_ */