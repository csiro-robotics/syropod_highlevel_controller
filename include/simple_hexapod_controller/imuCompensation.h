#pragma once
#include "standardIncludes.h"
#include "sensor_msgs/Imu.h"
#include "debugOutput.h"
#include "pose.h"
#include <boost/circular_buffer.hpp> 

struct Imu
{
public:
  Imu(Parameters p);
  ~Imu();
  
  void init(Parameters p);
  
  Vector3d getRotationCompensation(Quat targetRotation);
  Vector3d getTranslationCompensation(Vector3d targetTranslation);
  
  Vector3d getCurrentRotation(void);
 
  void setCompensationDebug(DebugOutput &debug);

  Vector3d removeGravity(Quat orientation, Vector3d linearAcceleration);
  
  sensor_msgs::Imu data;
  
  Parameters params;
  
  DebugOutput *debugDraw = NULL;
  
  Vector3d rotationAbsementError;
  Vector3d rotationPositionError;
  Vector3d rotationVelocityError;
  
  Vector3d translationAbsementError;
  Vector3d translationPositionError;
  Vector3d translationVelocityError;
  Vector3d translationAccelerationError;
};