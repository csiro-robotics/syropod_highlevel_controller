/*******************************************************************************************************************//**
 *  \file    dynamixel_motor_interface.cpp
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

#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"

/*******************************************************************************************************************//**
 * Interface constructor - setup desired joint state publisher
***********************************************************************************************************************/
DynamixelMotorInterface::DynamixelMotorInterface(Model* model)
  : model_(model)
{
  motor_publisher_ = n_.advertise<sensor_msgs::JointState>("/desired_joint_state", 1);
}

/***********************************************************************************************************************
 * Set speed for all joint motors
***********************************************************************************************************************/
void DynamixelMotorInterface::setupSpeed(double new_speed) //TBD Needed?
{
  dynamixel_controllers::SetSpeed speed;
  speed.request.speed = new_speed;
  std::map<int, Leg*>::iterator leg_it;

  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;
      std::string topic_name = "/hexapod/" + joint->name + "/set_speed";
      set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>(topic_name);
      set_speed_1_.call(speed);
    }
  }
}

/***********************************************************************************************************************
 * Set P gain for all joint motors
***********************************************************************************************************************/
void DynamixelMotorInterface::setPGain(double pGain) //TBD Needed?
{
  dynamixel_controllers::SetComplianceSlope Pgain;
  Pgain.request.slope = pGain;
  std::map<int, Leg*>::iterator leg_it;

  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;
      std::string topic_name = "/hexapod/" + joint->name + "/set_compliance_slope";
      set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(topic_name);
      set_gain_p_.call(Pgain);
    }
  }
}

/***********************************************************************************************************************
 * Iterate through all joints in model and publish desired joint state
***********************************************************************************************************************/
void DynamixelMotorInterface::publish(void)
{
  sensor_msgs::JointState msg;
  msg.header.stamp.sec = ros::Time::now().toSec();
  msg.header.stamp.nsec = ros::Time::now().toNSec();
  std::map<int, Leg*>::iterator leg_it;

  for (leg_it = model_->getLegContainer()->begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;
      msg.name.push_back(joint->name);
      msg.position.push_back(joint->desired_position + joint->position_offset);
      msg.velocity.push_back(joint->desired_velocity);
      msg.effort.push_back(joint->desired_effort);
    }
  }

  motor_publisher_.publish(msg);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
