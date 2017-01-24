#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include <sensor_msgs/JointState.h>
/*
DynamixelProMotorInterface::DynamixelProMotorInterface()
  : MotorInterface()
{
  motor_publisher_ = n_.advertise<sensor_msgs::JointState>("desired_hexapod_joint_state", 1);
}

void DynamixelProMotorInterface::setupSpeed(double my_speed)
{
}

void DynamixelProMotorInterface::setPGain(double pGain)
{
}

void DynamixelProMotorInterface::publish(Model* model)
{  
  sensor_msgs::JointState msg;
  msg.header.stamp.sec = ros::Time::now().toSec();
  msg.header.stamp.nsec = ros::Time::now().toNSec();
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second();
    std::map<int, Joint*>::iterator joint_it;
    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second();
      msg.name.push_back(joint->name);
      msg.position.push_back(joint->desired_position);
      msg.velocity.push_back(joint->desired_velocity);
      msg.effort.push_back(joint->desired_effort);
    }
  } 
  motor_publisher_.publish(msg);
}
*/
//TBD delete
