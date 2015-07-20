#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include <sensor_msgs/JointState.h>

DynamixelProMotorInterface::DynamixelProMotorInterface()
{
  motor_pub = nodehandler.advertise<sensor_msgs::JointState>("desired_motor_state", 1);
}		

void DynamixelProMotorInterface::setupSpeed(double my_speed)
{
}

void DynamixelProMotorInterface::setTargetAngle(int legID, int side, int jointID, double speed)
{
  angles[legID][side][jointID] = speed;
}

void DynamixelProMotorInterface::publish(void)
{
  sensor_msgs::JointState msg;

  msg.name.push_back("front_left_body_coxa");
  msg.position.push_back (angles[0][0][0]);
  msg.name.push_back("front_left_coxa_femour");
  msg.position.push_back (angles[0][0][1]);
  msg.name.push_back("front_left_femour_tibia");
  msg.position.push_back (angles[0][0][2]);
  msg.name.push_back("front_right_body_coxa");
  msg.position.push_back (angles[0][1][0]);
  msg.name.push_back("front_right_coxa_femour");
  msg.position.push_back (angles[0][1][1]);
  msg.name.push_back("front_right_femour_tibia");
  msg.position.push_back (angles[0][1][2]);
  msg.name.push_back("middle_left_body_coxa");
  msg.position.push_back (angles[1][0][0]);
  msg.name.push_back("middle_left_coxa_femour");
  msg.position.push_back (angles[1][0][1]);
  msg.name.push_back("middle_left_femour_tibia");
  msg.position.push_back (angles[1][0][2]);
  msg.name.push_back("middle_right_body_coxa");
  msg.position.push_back (angles[1][1][0]);
  msg.name.push_back("middle_right_coxa_femour");
  msg.position.push_back (angles[1][1][1]);
  msg.name.push_back("middle_right_femour_tibia");
  msg.position.push_back (angles[1][1][2]);
  msg.name.push_back("rear_left_body_coxa");
  msg.position.push_back (angles[2][0][0]);
  msg.name.push_back("rear_left_coxa_femour");
  msg.position.push_back (angles[2][0][1]);
  msg.name.push_back("rear_left_femour_tibia");
  msg.position.push_back (angles[2][0][2]);
  msg.name.push_back("rear_right_body_coxa");
  msg.position.push_back (angles[2][1][0]);
  msg.name.push_back("rear_right_coxa_femour");
  msg.position.push_back (angles[2][1][1]);
  msg.name.push_back("rear_right_femour_tibia");
  msg.position.push_back (angles[2][1][2]);

  motor_pub.publish (msg);
}


