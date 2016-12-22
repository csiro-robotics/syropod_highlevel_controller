#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include <sensor_msgs/JointState.h>

DynamixelProMotorInterface::DynamixelProMotorInterface()
  : MotorInterface()
  , angles_(3, vector<vector<double> >(2, vector<double>(3)))
  , velocities_(3, vector<vector<double> >(2, vector<double>(3)))
{
  motor_publisher_ = n_.advertise<sensor_msgs::JointState>("desired_hexapod_joint_state", 1);
}

void DynamixelProMotorInterface::setupSpeed(double my_speed)
{
}

void DynamixelProMotorInterface::setPGain(double pGain)
{
}

void DynamixelProMotorInterface::setTargetAngle(int legID, int side, int jointID, double angle)
{
  angles_[legID][side][jointID] = angle;
}

void DynamixelProMotorInterface::setVelocity(int legID, int side, int jointID, double velocity)
{
  velocities_[legID][side][jointID] = velocity;
}

void DynamixelProMotorInterface::publish(void)
{
  sensor_msgs::JointState msg;

  msg.header.stamp.sec = ros::Time::now().toSec();
  msg.header.stamp.nsec = ros::Time::now().toNSec();

  msg.name.push_back("front_left_body_coxa");
  msg.position.push_back(angles_[0][0][0]);
  msg.velocity.push_back(velocities_[0][0][0]);

  msg.name.push_back("front_left_coxa_femour");
  msg.position.push_back(angles_[0][0][1]);
  msg.velocity.push_back(velocities_[0][0][1]);

  msg.name.push_back("front_left_femour_tibia");
  msg.position.push_back(angles_[0][0][2]);
  msg.velocity.push_back(velocities_[0][0][2]);

  msg.name.push_back("front_right_body_coxa");
  msg.position.push_back(angles_[0][1][0]);
  msg.velocity.push_back(velocities_[0][1][0]);

  msg.name.push_back("front_right_coxa_femour");
  msg.position.push_back(angles_[0][1][1]);
  msg.velocity.push_back(velocities_[0][1][1]);

  msg.name.push_back("front_right_femour_tibia");
  msg.position.push_back(angles_[0][1][2]);
  msg.velocity.push_back(velocities_[0][1][2]);

  msg.name.push_back("middle_left_body_coxa");
  msg.position.push_back(angles_[1][0][0]);
  msg.velocity.push_back(velocities_[1][0][0]);

  msg.name.push_back("middle_left_coxa_femour");
  msg.position.push_back(angles_[1][0][1]);
  msg.velocity.push_back(velocities_[1][0][1]);

  msg.name.push_back("middle_left_femour_tibia");
  msg.position.push_back(angles_[1][0][2]);
  msg.velocity.push_back(velocities_[1][0][2]);

  msg.name.push_back("middle_right_body_coxa");
  msg.position.push_back(angles_[1][1][0]);
  msg.velocity.push_back(velocities_[1][1][0]);

  msg.name.push_back("middle_right_coxa_femour");
  msg.position.push_back(angles_[1][1][1]);
  msg.velocity.push_back(velocities_[1][1][1]);

  msg.name.push_back("middle_right_femour_tibia");
  msg.position.push_back(angles_[1][1][2]);
  msg.velocity.push_back(velocities_[1][1][2]);

  msg.name.push_back("rear_left_body_coxa");
  msg.position.push_back(angles_[2][0][0]);
  msg.velocity.push_back(velocities_[2][0][0]);

  msg.name.push_back("rear_left_coxa_femour");
  msg.position.push_back(angles_[2][0][1]);
  msg.velocity.push_back(velocities_[2][0][1]);

  msg.name.push_back("rear_left_femour_tibia");
  msg.position.push_back(angles_[2][0][2]);
  msg.velocity.push_back(velocities_[2][0][2]);

  msg.name.push_back("rear_right_body_coxa");
  msg.position.push_back(angles_[2][1][0]);
  msg.velocity.push_back(velocities_[2][1][0]);

  msg.name.push_back("rear_right_coxa_femour");
  msg.position.push_back(angles_[2][1][1]);
  msg.velocity.push_back(velocities_[2][1][1]);

  msg.name.push_back("rear_right_femour_tibia");
  msg.position.push_back(angles_[2][1][2]);
  msg.velocity.push_back(velocities_[2][1][2]);

  motor_publisher_.publish(msg);
}
