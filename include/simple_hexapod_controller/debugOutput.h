#pragma once
#include <ros/ros.h>
#include "standardIncludes.h"
#include "pose.h"

class DebugOutput 
{
public:
  DebugOutput();
  void drawPoints(const vector<Vector3d>& surveyPoints, const Vector4d &colour);
  void drawRobot(const Vector3d &extents, const vector<Vector3d> &legPoints, const Vector4d &colour);
  void plot(const vector<Vector2d> &points); // quick alternative to 2D plotting library
  void reset(){ robotID = 0; plotID = 0; }
  vector<Vector3d> tipPositions;
  vector<Vector3d> staticTipPositions;
  
private:
  int robotID;
  int plotID;
#if defined(DEBUGDRAW)
  ros::NodeHandle n;
  ros::Publisher robotPublisher;
  ros::Publisher pointsPublisher;
  ros::Publisher plotPublisher;
#endif
};
