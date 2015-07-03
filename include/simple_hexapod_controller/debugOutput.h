#pragma once
#include <ros/ros.h>
#include "standardIncludes.h"
#include "pose.h"

class DebugOutput 
{
public:
  DebugOutput();
  void drawPoints(const vector<Vector3d>& surveyPoints, const Vector4d &colour);
  void drawRobot(const Pose &bodyFrame, const Vector3d &extents, const vector<Vector3d> &legPoints, const Vector4d &colour);
  void reset(){ robotID = 0; }
  
private:
  int robotID;
#if defined(DEBUGDRAW)
  ros::NodeHandle n;
  ros::Publisher robotPublisher;
  ros::Publisher pointsPublisher;
#endif
};
