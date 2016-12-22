#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"

void DynamixelMotorInterface::setupSpeed(double my_speed)
{
  dynamixel_controllers::SetSpeed speed;
  speed.request.speed = my_speed;

  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_femour_tibia/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_femour_tibia/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_femour_tibia/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ =
      n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_femour_tibia/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_femour_tibia/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_body_coxa/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_coxa_femour/set_speed");
  set_speed_1_.call(speed);
  set_speed_1_ = n_.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_femour_tibia/set_speed");
  set_speed_1_.call(speed);
}

void DynamixelMotorInterface::setPGain(double pGain)
{
  dynamixel_controllers::SetComplianceSlope Pgain;
  Pgain.request.slope = pGain;

  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_left_body_coxa/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_left_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_left_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_right_body_coxa/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_right_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/front_right_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_left_body_coxa/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_left_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_left_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_right_body_coxa/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_right_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/middle_right_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_left_body_coxa/"
                                                                                         "set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/rear_left_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/rear_left_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/rear_right_body_coxa/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/rear_right_coxa_femour/set_compliance_slope");
  set_gain_p_.call(Pgain);
  set_gain_p_ = n_.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>(
      "/hexapod/rear_right_femour_tibia/set_compliance_slope");
  set_gain_p_.call(Pgain);
}

void DynamixelMotorInterface::setupPublishers(void)
{
  position_publishers_[0][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_body_coxa/command", 1);
  position_publishers_[0][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_coxa_femour/command", 1);
  position_publishers_[0][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_femour_tibia/command", 1);
  position_publishers_[0][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_body_coxa/command", 1);
  position_publishers_[0][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_coxa_femour/command", 1);
  position_publishers_[0][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_femour_tibia/command", 1);
  position_publishers_[1][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_body_coxa/command", 1);
  position_publishers_[1][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/command", 1);
  position_publishers_[1][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/command", 1);
  position_publishers_[1][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_body_coxa/command", 1);
  position_publishers_[1][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/command", 1);
  position_publishers_[1][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/command", 1);
  position_publishers_[2][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_body_coxa/command", 1);
  position_publishers_[2][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/command", 1);
  position_publishers_[2][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/command", 1);
  position_publishers_[2][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_body_coxa/command", 1);
  position_publishers_[2][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/command", 1);
  position_publishers_[2][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/command", 1);

  velocity_publishers_[0][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_body_coxa/velocity", 1);
  velocity_publishers_[0][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_coxa_femour/velocity", 1);
  velocity_publishers_[0][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/front_left_femour_tibia/velocity", 1);
  velocity_publishers_[0][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_body_coxa/velocity", 1);
  velocity_publishers_[0][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_coxa_femour/velocity", 1);
  velocity_publishers_[0][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/front_right_femour_tibia/velocity", 1);
  velocity_publishers_[1][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_body_coxa/velocity", 1);
  velocity_publishers_[1][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/velocity", 1);
  velocity_publishers_[1][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/velocity", 1);
  velocity_publishers_[1][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_body_coxa/velocity", 1);
  velocity_publishers_[1][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/velocity", 1);
  velocity_publishers_[1][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/velocity", 1);
  velocity_publishers_[2][0][0] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_body_coxa/velocity", 1);
  velocity_publishers_[2][0][1] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/velocity", 1);
  velocity_publishers_[2][0][2] = n_.advertise<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/velocity", 1);
  velocity_publishers_[2][1][0] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_body_coxa/velocity", 1);
  velocity_publishers_[2][1][1] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/velocity", 1);
  velocity_publishers_[2][1][2] = n_.advertise<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/velocity", 1);
}

void DynamixelMotorInterface::setTargetAngle(int legID, int side, int jointID, double angle)
{
  angles_[legID][side][jointID] = angle;
}

void DynamixelMotorInterface::setVelocity(int legID, int side, int jointID, double velocity)
{
  velocities_[legID][side][jointID] = velocity;
}

void DynamixelMotorInterface::publish(void)
{
  std_msgs::Float64 msg;
  std_msgs::Float64 velMsg;
  int i, j, k;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 2; j++)
    {
      for (k = 0; k < 3; k++)
      {
        msg.data = angles_[i][j][k];
        velMsg.data = velocities_[i][j][k];
        position_publishers_[i][j][k].publish(msg);
        velocity_publishers_[i][j][k].publish(velMsg);
      }
    }
  }
}
