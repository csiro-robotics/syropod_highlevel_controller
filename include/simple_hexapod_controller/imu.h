#pragma once
#include "quat.h"
#include <Eigen/Dense>

struct ImuData
{
  Quat orientation;
  Vector3d linearAcceleration;
  Vector3d angularVelocity;
};