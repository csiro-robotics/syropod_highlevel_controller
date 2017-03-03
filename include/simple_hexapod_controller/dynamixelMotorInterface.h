#ifndef SIMPLE_HEXAPOD_CONTROLLER_DYNAMIXEL_MOTOR_INTERFACE_H
#define SIMPLE_HEXAPOD_CONTROLLER_DYNAMIXEL_MOTOR_INTERFACE_H
/*******************************************************************************************************************//**
 *  \file    dynamixel_motor_interface.h
 *  \brief   Interface to dynamixel motor drivers. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   March 2017
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
***********************************************************************************************************************/

#include "standardIncludes.h"

#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "model.h"

/*******************************************************************************************************************//**
 * Class handling interface to dynamixel motor drivers
***********************************************************************************************************************/
class DynamixelMotorInterface
{
public:
	DynamixelMotorInterface(Model* model);
	void setupSpeed(double new_speed);  
	void setPGain(double p_gain);
	void publish(void);

private:
	ros::NodeHandle n_; 
	ros::Publisher motor_publisher_;
	ros::ServiceClient set_speed_1_;
	ros::ServiceClient set_gain_p_;
	Model* model_;
	void setupPublishers(void);
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_DYNAMIXEL_MOTOR_INTERFACE_H */
