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

static Vector2d localVelocity(0,0);
static double turnRate = 0;

// source catkin_ws/devel/setup.bash
// roslaunch hexapod_teleop hexapod_controllers.launch

void joypadChangeCallback(const geometry_msgs::Twist &twist)
{
  const double maxSpeed = 1.0;
  // these are 0 to 5 for some reason, so multiply by 0.2
  localVelocity = Vector2d(twist.angular.y, twist.angular.z) * 0.2 * maxSpeed;
  const double maxTurnRate = 1.0;
  turnRate = -twist.linear.y*0.2 * maxTurnRate;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  
  Model hexapod;
  TripodWalk walker(&hexapod, 0.2, 0.5, 0.2, Vector3d(0.5,0,-0.5), Vector3d(1.57,1.57,1.57));
//  TripodWalk walker(&hexapod, 0.2, 0.5, 0.2, Vector3d(0.0,0,0), Vector3d(0.8,0.8,0.8));
  DebugOutput debug;

  std_msgs::Float64 angle;  
  dynamixel_controllers::SetSpeed speed;

  MotorInterface interface;  
  speed.request.speed=1.0;
  interface.setupSpeed(speed);

  ros::Subscriber subscriber = n.subscribe("/desired_body_velocity", 1, joypadChangeCallback);
  ros::Rate r(roundToInt(1.0/timeDelta));         //frequency of the loop. 
  double t = 0;
  
  while (ros::ok())
  {
    walker.update(localVelocity, turnRate);
    debug.drawRobot(walker.pose, hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));

    std_msgs::Float64 angle;
    angle.data = sin(t)*0.5;
    for (int s = 0; s<2; s++)
    {
      for (int l = 0; l<3; l++)
      {
   //     for (int j = 0; j<3; j++)
  //        interface.setTargetAngle(l, s, j, angle);
      }
    }
    ros::spinOnce();
    r.sleep();

    debug.reset();
    t += timeDelta;
  }
}
