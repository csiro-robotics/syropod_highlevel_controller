/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/tripodWalk.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
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
  Quat orient;            //Orientation from IMU in quat
  Vector3d accel;         //Accelerations with respect to the world
  Vector3d accelcomp;
  RowVector3d gravityrot;
  
  orient.w=imu.orientation.w;
  orient.x=imu.orientation.x;
  orient.y=imu.orientation.y;
  orient.z=imu.orientation.z;  
  accel(1)=imu.linear_acceleration.x;//This is swaped for hardware purposes
  accel(0)=imu.linear_acceleration.y;
  accel(2)=-imu.linear_acceleration.z;
  
  /*//Compensation for gravity
  gravityrot=orient.toRotationMatrix()*Vector3d(0,0,-9.81);
  accelcomp(0)=accel(0)-gravityrot(0);
  accelcomp(1)=accel(1)-gravityrot(1);  
  accelcomp(2)=accel(2)-gravityrot(2);  
  ROS_ERROR("ACCEL= %f %f %f", accel(0), accel(1),accel(2));
  ROS_ERROR("GRAVITYROT= %f %f %f", gravityrot(0), gravityrot(1),gravityrot(2)); 
  ROS_ERROR("ACCELCOMP= %f %f %f", accelcomp(0), accelcomp(1),accelcomp(2));*/
  
  double imuStrength = 0.5; // tweak
  double stiffness = 6.0; // how strongly/quickly we return to the neutral pose
  Vector3d offsetAcc = -imuStrength*(accel-Vector3d(0,0,9.8)) - sqr(stiffness)*offsetPos - 2.0*stiffness*offsetVel;
  
  // double integrate
  offsetVel += offsetAcc*timeDelta;
  offsetPos += offsetVel*timeDelta;
  
  // Use just the angles
  //adjust.rotation=Quat(Vector3d(angles(0)/15+accelcomp(0)/10,-angles(1)/10,0)); //P control for body stabilization
  /* // control towards imu's orientation
  Quat targetAngle = ~imu.quat;
  static Vector3d angularVel;
  Vector3d angleDelta = (~adjust.rotation * targetAngle).toRotationVector();
  angleDelta[2] = 0;  // this may not be quite right
  Vector3d angularAcc = sqr(stiffnesss)*angleDelta -2.0*stiffness*angularVel;
  angularVel += angularAcc*timeDelta;
  adjust.rotation *= Quat(angularVel*timeDelta);
  */
  // use IMU's velocity control
  
  static Vector3d angularVel(0,0,0);
  Vector3d angleDelta = adjust.rotation.toRotationVector();
  Vector3d angularAcc;
  double stiffnessangular=1;
  
  angularAcc(0)=-sqr(stiffnessangular)*angleDelta(0) + 2.0*stiffnessangular*(imu.angular_velocity.y - angularVel(0));
  angularAcc(1)=-sqr(stiffnessangular)*angleDelta(1) + 2.0*stiffnessangular*(imu.angular_velocity.x - angularVel(1));
 
  angularVel += angularAcc*timeDelta;

 Vector3d rotation;
 rotation(0)=angularVel(0)*timeDelta;
 rotation(1)=angularVel(1)*timeDelta;
 rotation(2)=0;
 
  adjust.rotation *= Quat(rotation);
  adjust.position = offsetPos;
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
  ros::NodeHandle n_priv("~");
  bool dynamixel_interface = true;

  n_priv.param<bool>("dynamixel_interface", dynamixel_interface, true);

  Model hexapod;
  Vector3d yawOffsets(0.77,0,-0.77);
  TripodWalk walker(&hexapod, 0.5, 0.12, yawOffsets, Vector3d(1.4,1.4,1.4), 1.9);
  DebugOutput debug;

  double angle;  
  MotorInterface *interface;

  if (dynamixel_interface)
    interface = new DynamixelMotorInterface();
  else
    interface = new DynamixelProMotorInterface();

  interface->setupSpeed(0.5);

  ros::Subscriber subscriber = n.subscribe("/desired_body_velocity", 1, joypadChangeCallback);
  ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data_ned", 1, imuCallback);
  
  ros::Rate r(roundToInt(1.0/timeDelta));         //frequency of the loop. 
  double t = 0;
  
  while (ros::ok())
  {
    Pose adjust = Pose::identity(); // offset pose for body. Use this to close loop with the IMU
    adjust=compensation();
    walker.update(localVelocity*localVelocity.squaredNorm(), turnRate, &adjust); // the * squaredNorm just lets the thumbstick give small turns easier
    debug.drawRobot(hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose * adjust), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));

      
    
    if (true)
    {
      for (int s = 0; s<2; s++)
      {
        double dir = s==0 ? -1 : 1;
        for (int l = 0; l<3; l++)
        {
          angle = dir*(walker.model->legs[l][s].yaw - yawOffsets[l]);
          interface->setTargetAngle(l, s, 0, angle);
          angle = -dir*walker.model->legs[l][s].liftAngle;
          interface->setTargetAngle(l, s, 1, angle);
          angle = dir*walker.model->legs[l][s].kneeAngle;
          interface->setTargetAngle(l, s, 2, angle);
        }
      }
      interface->publish();
    }
    ros::spinOnce();
    r.sleep();

    debug.reset();
    t += timeDelta;
  }
}
