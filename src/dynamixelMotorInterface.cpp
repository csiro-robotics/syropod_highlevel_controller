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
  /*
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
  */
  
  // Front left leg (AL or [0][0][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[0][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AL_coxa_position_controller/command", 1);
  publishers[0][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AL_coxat_position_controller/command", 1);
  publishers[0][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AL_femur_position_controller/command", 1);
  publishers[0][0][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AL_tibia_position_controller/command", 1);
  publishers[0][0][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AL_tarsus_position_controller/command", 1);

  // Front right leg (AR or [0][1][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[0][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AR_coxa_position_controller/command", 1);
  publishers[0][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AR_coxat_position_controller/command", 1);
  publishers[0][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AR_femur_position_controller/command", 1);
  publishers[0][1][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AR_tibia_position_controller/command", 1);
  publishers[0][1][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/AR_tarsus_position_controller/command", 1);

  // Middle left leg (BL or [1][0][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[1][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BL_coxa_position_controller/command", 1);
  publishers[1][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BL_coxat_position_controller/command", 1);
  publishers[1][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BL_femur_position_controller/command", 1);
  publishers[1][0][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BL_tibia_position_controller/command", 1);
  publishers[1][0][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BL_tarsus_position_controller/command", 1);

  // Middle right leg (BR or [1][1][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[1][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BR_coxa_position_controller/command", 1);
  publishers[1][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BR_coxat_position_controller/command", 1);
  publishers[1][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BR_femur_position_controller/command", 1);
  publishers[1][1][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BR_tibia_position_controller/command", 1);
  publishers[1][1][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/BR_tarsus_position_controller/command", 1);

  // Rear left leg (CL or [2][0][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[2][0][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CL_coxa_position_controller/command", 1);
  publishers[2][0][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CL_coxat_position_controller/command", 1);
  publishers[2][0][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CL_femur_position_controller/command", 1);
  publishers[2][0][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CL_tibia_position_controller/command", 1);
  publishers[2][0][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CL_tarsus_position_controller/command", 1);

  // Rear right leg (CR or [2][1][i] with 5 DOF for each leg i = 0 ... 4)
  publishers[2][1][0]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CR_coxa_position_controller/command", 1);
  publishers[2][1][1]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CR_coxat_position_controller/command", 1);
  publishers[2][1][2]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CR_femur_position_controller/command", 1);
  publishers[2][1][3]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CR_tibia_position_controller/command", 1);
  publishers[2][1][4]=nodehandler.advertise<std_msgs::Float64>("/hexapod/CR_tarsus_position_controller/command", 1);
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
      for (k=0; k<5; k++) {
        msg.data = angles[i][j][k];
        publishers[i][j][k].publish(msg);
      }
}


