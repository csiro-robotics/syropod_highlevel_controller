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

void DynamixelMotorInterface::setPGain(double pGain)
{
  dynamixel_controllers::SetComplianceSlope Pgain;
  Pgain.request.slope=pGain;
  
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_left_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_left_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_left_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_right_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_right_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/front_right_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_left_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_left_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_left_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_right_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_right_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/middle_right_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_left_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_left_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_left_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_right_body_coxa/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_right_coxa_femour/set_compliance_slope");
  setGainP.call(Pgain);
  setGainP=nodehandler.serviceClient<dynamixel_controllers::SetComplianceSlopeRequest>("/hexapod/rear_right_femour_tibia/set_compliance_slope");
  setGainP.call(Pgain);
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
  
  velPublishers[0][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_body_coxa/velocity", 1);
  velPublishers[0][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_coxa_femour/velocity", 1);
  velPublishers[0][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_left_femour_tibia/velocity", 1);
  velPublishers[0][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_body_coxa/velocity", 1);
  velPublishers[0][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_coxa_femour/velocity", 1);
  velPublishers[0][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/front_right_femour_tibia/velocity", 1);
  velPublishers[1][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_body_coxa/velocity", 1);
  velPublishers[1][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/velocity", 1);
  velPublishers[1][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/velocity", 1);
  velPublishers[1][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_body_coxa/velocity", 1);
  velPublishers[1][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/velocity", 1);
  velPublishers[1][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/velocity", 1);
  velPublishers[2][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_body_coxa/velocity", 1);
  velPublishers[2][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/velocity", 1);
  velPublishers[2][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/velocity", 1);
  velPublishers[2][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_body_coxa/velocity", 1);
  velPublishers[2][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/velocity", 1);
  velPublishers[2][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/velocity", 1);
  
}

void DynamixelMotorInterface::setTargetAngle(int legID, int side, int jointID, double angle)
{
  angles[legID][side][jointID] = angle;
}

void DynamixelMotorInterface::setVelocity(int legID, int side, int jointID, double velocity)
{
  velocities[legID][side][jointID] = velocity;
}

void DynamixelMotorInterface::publish(void)
{
  std_msgs::Float64 msg;
  std_msgs::Float64 velMsg;
  int i, j, k;

  for (i=0; i<3; i++)
  {
    for (j=0; j<2; j++)
    {
      for (k=0; k<3; k++) 
      {
        msg.data = angles[i][j][k];
        velMsg.data = velocities[i][j][k];
        publishers[i][j][k].publish(msg);
        velPublishers[i][j][k].publish(velMsg);
      }
    }
  }
}


