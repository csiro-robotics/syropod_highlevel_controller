#include "../include/simple_hexapod_controller/impedanceController.h"

/***********************************************************************************************************************
 * Constructor
***********************************************************************************************************************/
ImpedanceController::ImpedanceController(const Parameters &p)
  : tip_forces_(3, std::vector<double>(2)), impedance_state_(3, std::vector<std::vector<double>>(2, std::vector<double>(2)))
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
  parameters_ = p;
  delta_t_ = parameters_.integrator_step_time;
  force_gain_ = parameters_.force_gain;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      virtual_mass_[l][s] = parameters_.virtual_mass;
      virtualStiffness[l][s] = parameters_.virtual_stiffness;
      virtual_damping_ratio_[l][s] = parameters_.virtual_damping_ratio;

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
  tip_forces_[l][s] = effort[l][s];
  deltaZ[l][s] = calculateDeltaZ(l, s);
}

/***********************************************************************************************************************
 * Calculates change in tip position in z direction (deltaZ) according to tip force value
***********************************************************************************************************************/
double ImpedanceController::calculateDeltaZ(int l, int s)
{
  double virtualDamping = virtual_damping_ratio_[l][s] * 2 * sqrt(virtual_mass_[l][s] * virtualStiffness[l][s]);
  runge_kutta4<state_type> stepper;
  integrate_const(stepper,
                  [&](const state_type &x, state_type &dxdt, double t)
                  {
                    dxdt[0] = x[1];
                    dxdt[1] = -tip_forces_[l][s] / virtual_mass_[l][s] * force_gain_ -
                              virtualDamping / virtual_mass_[l][s] * x[1] -
                              virtualStiffness[l][s] / virtual_mass_[l][s] * x[0];
                  },
                  impedance_state_[l][s], 0.0, delta_t_, delta_t_ / 30);
  return impedance_state_[l][s][0];
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
 * Scales virtual stiffness value of given and adjacent legs according to reference
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(double stepReference, int l, int s)
{
  // Get adjacent leg references
  int adjacent1Leg = (l == 1) ? (l + 1) : l;
  int adjacent1Side = (l == 1) ? s : !s;
  int adjacent2Leg = (l == 0) ? (l + 1) : (l - 1);
  int adjacent2Side = s;

  //(X-1)+1 to change range from 0->1 to 1->multiplier
  virtualStiffness[l][s] = parameters_.virtual_stiffness * (stepReference * (parameters_.swing_stiffness_scaler - 1) + 1);

  virtualStiffness[adjacent1Leg][adjacent1Side] =
      parameters_.virtual_stiffness * (stepReference * (parameters_.load_stiffness_scaler - 1) + 1);
  virtualStiffness[adjacent2Leg][adjacent2Side] =
      parameters_.virtual_stiffness * (stepReference * (parameters_.load_stiffness_scaler - 1) + 1);
}

/***********************************************************************************************************************
 * Scales virtual stiffness value of legs adjacent to swinging leg according to swing cycle percentage
 * Note: The reseting and addition of stiffness allows overlapping step cycles to JOINTLY add stiffness to
 * simultaneously adjacent legs
***********************************************************************************************************************/
void ImpedanceController::updateStiffness(WalkController *walker)
{
  // Reset virtual Stiffness each cycle
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      virtualStiffness[l][s] = parameters_.virtual_stiffness;
    }
  }

  // Calculate dynamic virtual stiffness
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      if (walker->legSteppers[l][s].step_state_ == SWING)
      {
        // Get adjacent leg references
        int adjacent1Leg = (l == 1) ? (l + 1) : l;
        int adjacent1Side = (l == 1) ? s : !s;
        int adjacent2Leg = (l == 0) ? (l + 1) : (l - 1);
        int adjacent2Side = s;

        double zDiff =
            walker->legSteppers[l][s].current_tip_position_[2] - walker->legSteppers[l][s].default_tip_position_[2];
        double stepReference = 0;
        stepReference += abs(zDiff / (walker->step_clearance_ * walker->maximum_body_height_));

        //(X-1)+1 to change range from 0->1 to 1->multiplier
        virtualStiffness[l][s] = parameters_.virtual_stiffness * (stepReference * (parameters_.swing_stiffness_scaler - 1) + 1);

        virtualStiffness[adjacent1Leg][adjacent1Side] +=
            parameters_.virtual_stiffness * ((parameters_.load_stiffness_scaler - 1) * stepReference);
        virtualStiffness[adjacent2Leg][adjacent2Side] +=
            parameters_.virtual_stiffness * ((parameters_.load_stiffness_scaler - 1) * stepReference);
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
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      double proportionalGain = 1000;  // TBD Parameterise
      double integralGain = 0;  // TBD Parameterise
      double derivativeGain = 0;  // TBD Parameterise

      posedTipPositions[l][s] = currentPose.inverseTransformVector(identityTipPositions[l][s]);

      double oldTipPositionError = zTipPositionError[l][s];
      zTipPositionError[l][s] = identityTipPositions[l][s][2] - posedTipPositions[l][s][2];
      zTipAbsementError[l][s] += zTipPositionError[l][s] * parameters_.time_delta;

      // Low pass filter of IMU angular velocity data
      double smoothingFactor = 0.15;
      zTipVelocityError[l][s] = smoothingFactor * (zTipPositionError[l][s] - oldTipPositionError) / parameters_.time_delta +
                                (1 - smoothingFactor) * zTipVelocityError[l][s];

      double minStiffness = parameters_.virtual_stiffness;

      virtualStiffness[l][s] = max(zTipPositionError[l][s] * proportionalGain + zTipAbsementError[l][s] * integralGain +
                                       zTipVelocityError[l][s] * derivativeGain,
                                   minStiffness);
    }
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
