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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  
  Model hexapod;
  TripodWalk walker(&hexapod, 0.2, 0.6, 0.2, Vector3d(0.5,0,-0.5), Vector3d(1.57,1.57,1.57));
  DebugOutput debug;
  double walkSpeed = 0.5;
  double t = 0;

  std_msgs::Float64 angle;  
  dynamixel_controllers::SetSpeed speed;

  MotorInterface interface;  
  speed.request.speed=0.2;
  interface.setupSpeed(speed);

  ros::Rate r(30);         //frequency of the loop. 

  while (ros::ok())
  {
    double turn = sin(t*0.2);
    walker.update(Vector2d(0, walkSpeed), turn);
    debug.drawRobot(walker.pose, hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));

    std_msgs::Float64 angle;
    angle.data = sin(t)*0.5;
    for (int s = 0; s<2; s++)
    {
      for (int l = 0; l<3; l++)
      {
        for (int j = 0; j<3; j++)
          interface.setTargetAngle(l, s, j, angle);
      }
    }
    ros::spinOnce();
    r.sleep();

    debug.reset();
    t += timeDelta;
  }
}
