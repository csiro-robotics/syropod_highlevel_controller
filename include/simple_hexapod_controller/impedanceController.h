#ifndef SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H
/** 
 *  \file    impedance_controller.h
 *  \brief   Impedance controller. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   January 2017
 *  \version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
 */

#include "standardIncludes.h"
#include "parametersAndStates.h"
#include <boost/numeric/odeint.hpp>

#include "model.h"
#include "walkController.h"

using namespace boost::numeric::odeint;

class ImpedanceController
{
public:
  ImpedanceController(Model* model, Parameters* params);
  void init(void);

  // Solve ODE of the impedance controller and update delta_z value for leg
  void updateImpedance(Leg* leg, bool use_joint_effort);
  
  void updateStiffness(Leg*, double step_reference);
  void updateStiffness(WalkController *walker);

private:
  Model* model_;
  Parameters* params_;
  double delta_t_;
  double force_gain_;
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H */