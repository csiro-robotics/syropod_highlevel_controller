#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"

void DynamixelMotorInterface::setupSpeed(double my_speed)
{
  dynamixel_controllers::SetSpeed speed;
  speed.request.speed=my_speed;

  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_left_femour_tibia/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/front_right_femour_tibia/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_left_femour_tibia/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/middle_right_femour_tibia/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_left_femour_tibia/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_body_coxa/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_coxa_femour/set_speed");
  setspeed1.call(speed);
  setspeed1=nodehandler.serviceClient<dynamixel_controllers::SetSpeed>("/hexapod/rear_right_femour_tibia/set_speed");
  setspeed1.call(speed);
}

void DynamixelMotorInterface::setupPublishers(void)
{
  publishers[0][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_body_coxa/command", 1);
  publishers[0][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_coxa_femour/command", 1);
  publishers[0][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_femour_tibia/command", 1);
  publishers[0][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_body_coxa/command", 1);
  publishers[0][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_coxa_femour/command", 1);
  publishers[0][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_femour_tibia/command", 1);
  publishers[1][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_body_coxa/command", 1);
  publishers[1][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/command", 1);
  publishers[1][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/command", 1);
  publishers[1][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_body_coxa/command", 1);
  publishers[1][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/command", 1);
  publishers[1][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/command", 1);
  publishers[2][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_body_coxa/command", 1);
  publishers[2][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/command", 1);
  publishers[2][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/command", 1);
  publishers[2][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_body_coxa/command", 1);
  publishers[2][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/command", 1);
  publishers[2][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/command", 1);
}


void DynamixelMotorInterface::setTargetAngle(int legID, int side, int jointID, double speed)
{
  angles[legID][side][jointID] = speed;
}

void DynamixelMotorInterface::publish(void)
{
  std_msgs::Float64 msg;
  int i, j, k;

  for (i=0; i<3; i++)
    for (j=0; j<2; j++)
      for (k=0; k<3; k++) {
        msg.data = angles[i][j][k];
        publishers[i][j][k].publish(msg);
      }
}


