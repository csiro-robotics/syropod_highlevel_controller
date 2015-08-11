#pragma once
#include "standardIncludes.h"
#include "sensor_msgs/Imu.h"
#include "pose.h"

void imuCallback(const sensor_msgs::Imu &imudata);
Pose compensation(const Vector3d &targetAccel, double targetAngularVel);
