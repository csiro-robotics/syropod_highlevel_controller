////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_ADMITTANCE_CONTROLLER_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_ADMITTANCE_CONTROLLER_H

#include "standard_includes.h"
#include "parameters_and_states.h"
#include <boost/numeric/odeint.hpp>

#include "model.h"

#define ADMITTANCE_DEADBAND 0.0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// This class handles the application of an admittance controller to the robot model. Specifically it calculates a
/// tip position offset value for each leg of the robot which are modelled as mass/spring/damper
/// systems. The system for each leg is defined by parameterised characteristics (system mass, spring stiffness and
/// damping ratio) and the input to the system is a tip force value scaled by a parameterised force gain. As well as
/// handling calculation of admittance this class handles a dynamic stiffness system which dynamically modifies the
/// stiffness input for the system of each leg depending on the state of the leg and robot.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class AdmittanceController
{
public:
  /// AdmittanceController class constructor. Assigns pointers to robot model object and parameter data storage object
  /// and calls initialisation.
  /// @param[in] model Pointer to the robot model class object
  /// @param[in] params Pointer to the parameter struct object  
  AdmittanceController(std::shared_ptr<Model> model, const Parameters& params);
  
  /// Iterates through legs in the robot model and updates the tip position offset value for each.
  /// The calculation is achieved through the use of a classical Runge-Kutta ODE solver with a force input
  /// acquired from a tip force callback OR from estimation from joint effort values.
  /// @todo Implement admittance control in x/y axis
  void updateAdmittance(void);
  
  /// Scales virtual stiffness of an input swinging leg and two 'adjacent legs' according to an input reference.
  /// The input reference ranges between 0.0 and 1.0, and defines the characteristic of the curve as stiffness changes
  /// from the default stiffness (i.e. scaled by 1.0) to the swing/load stiffness (i.e. scaled by swing/load scaling
  /// value). The swing stiffness value is applied to the swinging leg and load stiffness value applied to the two
  /// adjacent legs.
  /// @param[in] leg A pointer to the leg object associated with the stiffness value to be updated
  /// @param[in] scale_reference A double ranging from 0.0->1.0 which controls the scaling of the stiffnesses
  void updateStiffness(std::shared_ptr<Leg> leg, const double& scale_reference);

  /// Scales virtual stiffness of swinging leg and adjacent legs according to the walk cycle of the walk controller.
  /// The percentage vertical position difference of the swinging leg tip from it's default position is used as a
  /// reference value ranging from 0.0 (default tip height) to 1.0 (max swing height). This reference value defines the
  /// characteristic of the curve as stiffness changes from the default stiffness (i.e. scaled by 1.0) to the swing/load
  /// stiffness (i.e. scaled by swing/load scaling value). The swing stiffness value is applied to the swinging leg and
  /// load stiffness value applied to the two adjacent legs. The reseting and addition of stiffness allows overlapping
  /// step cycles to JOINTLY add stiffness to simultaneously adjacent legs.
  /// @param[in] walker A pointer to the walk controller
  void updateStiffness(std::shared_ptr<WalkController> walker);

private:
  std::shared_ptr<Model> model_;  ///< Pointer to the robot model object
  const Parameters& params_; ///< Pointer to parameter data structure for storing parameter variables
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_ADMITTANCE_CONTROLLER_H