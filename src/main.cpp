/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/tripodWalk.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include <chrono>
#include <caca_conio.h>
#include <thread>
#include <iostream>

#include <iostream>
#include <sys/select.h>

int kbhit(void)
{
  struct timeval tv;
  fd_set read_fd;
  tv.tv_sec=0;
  tv.tv_usec=0;
  FD_ZERO(&read_fd);
  FD_SET(0,&read_fd);
  if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */
  if(FD_ISSET(0,&read_fd))
    return 1;
  return 0;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  chrono::milliseconds interval(roundToInt(1000.0*timeDelta)); 
  
  FILE * input;
  Model hexapod;
  TripodWalk walker(&hexapod, 0.2, 0.6, 0.2, Vector3d(0.5,0,-0.5), Vector3d(1.57,1.57,1.57));
  DebugOutput debug;
  double speed = 0.5;
  double t = 0;
  for(;;)
  {
    double turn = sin(t*0.2);
    walker.update(Vector2d(0,speed), turn);
    debug.drawRobot(walker.pose, hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));
    this_thread::sleep_for(interval);
    debug.reset();
    t += timeDelta;
    if (kbhit())
    {
      break;
    }
  }
}
