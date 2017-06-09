#ifndef SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H
#define SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H
/*******************************************************************************************************************//**
 *  @file    impedance_controller.h
 *  @brief   Impedance controller. Part of simple hexapod controller.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    June 2017
 *  @version 0.5.0
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
***********************************************************************************************************************/

#include "standard_includes.h"
#include "parameters_and_states.h"
#include <boost/numeric/odeint.hpp>

#include "model.h"
#include "walk_controller.h"

/***********************************************************************************************************************
 * This class handles the application of an impedance controller to the robot model. Specifically it calculates a
 * vertical tip position offset value (delta_z) for each leg of the robot which are modelled as mass/spring/damper
 * systems. The system for each leg is defined by parameterised characteristics (system mass, spring stiffness and
 * damping ratio) and the input to the system is a tip force value scaled by a parameterised force gain. As well as
 * handling calculation of impedance this class handles a dynamic stiffness system which dynamically modifies the
 * stiffness input for the system of each leg depending on the state of the leg and robot.
***********************************************************************************************************************/
class ImpedanceController
{
public:
  /**
   * ImpedanceController class constructor. Assigns pointers to robot model object and parameter data storage object 
   * and calls initialisation.
   * @param[in] model Pointer to the robot model class object
   * @param[in] params Pointer to the parameter struct object
   */
  ImpedanceController(Model* model, Parameters* params);
  
  /**
   * Impdedance controller initialisation. Assigns integrator step time and force gain for robot model and for each leg
   * sets virtual mass/stiffness/damping ratio from parameters.
   */
  void init(void);

  /**
   * Iterates through legs in the robot model and updates the vertical tip position offset value (delta_z) for each.
   * The calculation of delta_z is achieved through the use of a classical Runge-Kutta ODE solver with a force input
   * acquired from a tip force callback OR from a joint effort value.
   * @input[in] use_joint_effort Bool which determines whether the tip force input is derived from joint effort
   * @todo Refactor the method of generating tip force from joint effort/s and method of determining effort direction.
   */
  void updateImpedance(bool use_joint_effort);
  
  /**
   * Scales virtual stiffness of an input swinging leg and two 'adjacent legs' according to an input reference.
   * The input reference ranges between 0.0 and 1.0, and defines the characteristic of the curve as stiffness changes
   * from the default stiffness (i.e. scaled by 1.0) to the swing/load stiffness (i.e. scaled by swing/load scaling 
   * value). The swing stiffness value is applied to the swinging leg and load stiffness value applied to the two 
   * adjacent legs.
   * @input[in] leg A pointer to the leg object associated with the stiffness value to be updated.
   * @input[in] scale_reference A double ranging from 0.0->1.0 which controls the scaling of the stiffnesses
   */
  void updateStiffness(Leg*, double scale_reference);
  
  /**
   * Scales virtual stiffness of swinging leg and adjacent legs according to the walk cycle of the walk controller.
   * The percentage vertical position difference of the swinging leg tip from it's default position is used as a 
   * reference value ranging from 0.0 (default tip height) to 1.0 (max swing height). This reference value defines the 
   * characteristic of the curve as stiffness changes from the default stiffness (i.e. scaled by 1.0) to the swing/load
   * stiffness (i.e. scaled by swing/load scaling value). The swing stiffness value is applied to the swinging leg and 
   * load stiffness value applied to the two adjacent legs. The reseting and addition of stiffness allows overlapping 
   * step cycles to JOINTLY add stiffness to simultaneously adjacent legs.
   * @input[in] walker A pointer to the walk controller
   */
  void updateStiffness(WalkController *walker);

private:
  Model* model_;        ///! Pointer to the robot model object
  Parameters* params_;  ///! Pointer to parameter data structure for storing parameter variables.
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_IMPEDANCE_CONTROLLER_H */
