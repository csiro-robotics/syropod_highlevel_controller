#pragma once
#include "standardIncludes.h"
#include "pose.h"
#include <ros/ros.h>

class DebugOutput
{
public:
  DebugOutput();
  void drawPoints(const vector<Vector3d> &survey_points, const Vector4d &colour);
  void drawRobot(const Vector3d &extents, const vector<Vector3d> &leg_points, const Vector4d &colour);
  void plot(const vector<Vector2d> &points);  // quick alternative to 2D plotting library
  void reset()
  {
    robot_ID_ = 0;
    plot_ID_ = 0;
  }
  vector<Vector3d> tip_positions_;
  vector<Vector3d> static_tip_positions_;

private:
  int robot_ID_;
  int plot_ID_;
#if defined(DEBUGDRAW)
  ros::NodeHandle n;
  ros::Publisher robot_publisher_;
  ros::Publisher points_publisher_;
  ros::Publisher plot_publisher_;
#endif
};
