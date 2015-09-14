#pragma once
#include "standardIncludes.h"
#include "sensor_msgs/Imu.h"
#include "debugOutput.h"
#include "pose.h"

void imuCallback(const sensor_msgs::Imu &imudata);
Vector3d compensation(const Vector3d &targetAccel, double targetAngularVel);
void setCompensationDebug(DebugOutput &debug);
