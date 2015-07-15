/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/tripodWalk.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

static Vector2d localVelocity(0,0);
static double turnRate = 0;
sensor_msgs::Imu imu;

// target rather than measured data
static Vector3d offsetPos(0,0,0);
static Vector3d offsetVel(0,0,0);

// source catkin_ws/devel/setup.bash
// roslaunch hexapod_teleop hexapod_controllers.launch

void imuCallback(const sensor_msgs::Imu & imudata){  
  imu=imudata;
}

Pose compensation(){
  Pose adjust;
  //Pose desired = Pose::identity();
  //Vector3d angles; 
  adjust.position=Vector3d(0,0,imu.linear_acceleration.x/20);
  adjust.rotation=Quat(Vector3d(imu.linear_acceleration.x/20,0,0));
  return adjust;
  
}


void joypadChangeCallback(const geometry_msgs::Twist &twist)
{
  ASSERT(twist.angular.z < 0.51);
 // ASSERT(twist.linear.y < 0.51);
  // these are 0 to 5 for some reason, so multiply by 0.2
  localVelocity = Vector2d(twist.angular.y, twist.angular.z) * 2.0;
  localVelocity = clamped(localVelocity, 1.0);
  turnRate = -twist.linear.y*0.2;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  
  Model hexapod;
  Vector3d yawOffsets(0.77,0,-0.77);
  TripodWalk walker(&hexapod, 0.5, 0.12, yawOffsets, Vector3d(1.4,1.4,1.4), 2.2);
  DebugOutput debug;

  std_msgs::Float64 angle;  
  dynamixel_controllers::SetSpeed speed;

  MotorInterface interface;  
  speed.request.speed=0.5;
  interface.setupSpeed(speed);

  ros::Subscriber subscriber = n.subscribe("/desired_body_velocity", 1, joypadChangeCallback);
  ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data_ned", 1, imuCallback);
  
  ros::Rate r(roundToInt(1.0/timeDelta));         //frequency of the loop. 
  double t = 0;
  
  while (ros::ok())
  {
    Pose adjust = Pose::identity(); // offset pose for body. Use this to close loop with the IMU
    adjust=compensation();

    Vector3d imuAcceleration(0,0,0); // retrieve from IMU linear acceleration
    double imuStrength = 1.0; // tweak
    double stiffness = 6.0; // how strongly/quickly we return to the neutral pose
    Vector3d offsetAcc = -imuStrength*imuAcceleration - sqr(stiffness)*offsetPos - 2.0*stiffness*offsetVel;
    
    // double integrate
    offsetVel += offsetAcc*timeDelta;
    offsetPos += offsetVel*timeDelta;
    
    adjust.position = offsetPos;
    walker.update(localVelocity*localVelocity.squaredNorm(), turnRate, &adjust); // the * squaredNorm just lets the thumbstick give small turns easier
    debug.drawRobot(hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose * adjust), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));

    if (false)
    {
      std_msgs::Float64 angle;
      for (int s = 0; s<2; s++)
      {
        double dir = s==0 ? -1 : 1;
        for (int l = 0; l<3; l++)
        {
          angle.data = dir*(walker.model->legs[l][s].yaw - yawOffsets[l]);
          interface.setTargetAngle(l, s, 0, angle);
          angle.data = -dir*walker.model->legs[l][s].liftAngle;
          interface.setTargetAngle(l, s, 1, angle);
          angle.data = dir*walker.model->legs[l][s].kneeAngle;
          interface.setTargetAngle(l, s, 2, angle);
        }
      }
    }
    ros::spinOnce();
    r.sleep();

    debug.reset();
    t += timeDelta;
  }
}
