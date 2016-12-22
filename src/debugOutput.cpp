#include "simple_hexapod_controller/debugOutput.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>

static Pose rotate_90;

DebugOutput::DebugOutput()
{
#if defined(DEBUGDRAW)
  robot_publisher_ = n.advertise<visualization_msgs::Marker>("robot", 1);
  points_publisher_ = n.advertise<visualization_msgs::Marker>("points", 1);
  plot_publisher_ = n.advertise<visualization_msgs::Marker>("plot", 1);
  reset();
  rotate_90.rotation_ = Quat(1, 0, 0, 0);  // Quat(sqrt(0.5), sqrt(0.5), 0, 0);
  rotate_90.position_ = Vector3d(0, 0, 0);
#endif
}

void DebugOutput::drawRobot(const Vector3d &extents, const vector<Vector3d> &legPoints, const Vector4d &colour)
{
#if defined(DEBUGDRAW)
  visualization_msgs::Marker lineList;
  lineList.header.frame_id = "/my_frame";
  lineList.header.stamp = ros::Time::now();
  lineList.ns = "trajectory_polyline_" + to_string(robot_ID_);
  lineList.action = visualization_msgs::Marker::ADD;
  lineList.pose.orientation.x = 0.0;
  lineList.pose.orientation.y = 0.0;
  lineList.pose.orientation.z = 0.0;
  lineList.pose.orientation.w = 1.0;

  lineList.id = robot_ID_++;
  lineList.type = visualization_msgs::Marker::LINE_LIST;

  lineList.scale.x = 0.005;  // only x component needed
  lineList.scale.y = 0.005;  // only x component needed

  lineList.color.r = colour[0];
  lineList.color.g = colour[1];
  lineList.color.b = colour[2];
  lineList.color.a = colour[3];

  Vector3d pos;
  int rootIDs[] = { 0, 4, 8, 20, 16, 12 };
  int oldID = 12;
  for (unsigned int l = 0; l < 6; l++)
  {
    int root = rootIDs[l];
    geometry_msgs::Point p;
    pos = rotate_90 * legPoints[oldID];
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    lineList.points.push_back(p);

    pos = rotate_90 * legPoints[root];
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    lineList.points.push_back(p);

    oldID = root;
  }
  Vector3d midPos = (legPoints[rootIDs[0]] + legPoints[rootIDs[5]]) * 0.5;
  Vector3d dir = (legPoints[rootIDs[0]] - legPoints[rootIDs[2]]) * 0.1;
  geometry_msgs::Point p;
  pos = rotate_90 * midPos;
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  lineList.points.push_back(p);
  pos = rotate_90 * (midPos + dir);
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  lineList.points.push_back(p);

  for (unsigned int l = 0; l < legPoints.size() - 1; l += 4)
  {
    for (unsigned int i = l; i < l + 3; i++)
    {
      geometry_msgs::Point p;
      pos = rotate_90 * legPoints[i];
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];
      lineList.points.push_back(p);

      pos = rotate_90 * legPoints[i + 1];
      p.x = pos[0];
      p.y = pos[1];
      p.z = pos[2];
      lineList.points.push_back(p);
    }
  }
  robot_publisher_.publish(lineList);
#endif
}

void DebugOutput::drawPoints(const vector<Vector3d> &points, const Vector4d &colour)
{
#if defined(DEBUGDRAW)

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "point_marker";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;

  marker.color.r = colour[0];
  marker.color.g = colour[1];
  marker.color.b = colour[2];
  marker.color.a = colour[3];

  marker.lifetime = ros::Duration();

  geometry_msgs::Point point;
  for (unsigned int i = 0; i < points.size(); ++i)
  {
    Vector3d pos = rotate_90 * points[i];
    point.x = pos[0];
    point.y = pos[1];
    point.z = pos[2];
    marker.points.push_back(point);
  }

  points_publisher_.publish(marker);
#endif
}

void DebugOutput::plot(const vector<Vector2d> &points)
{
#if defined(DEBUGDRAW)
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "point_marker";
  marker.id = plot_ID_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  Vector4d colour(1, 0.5 * (double)((plot_ID_ - 1) % 3), (double)(plot_ID_ % 2), 1);
  marker.color.r = colour[0];
  marker.color.g = colour[1];
  marker.color.b = colour[2];
  marker.color.a = colour[3];

  marker.lifetime = ros::Duration();

  geometry_msgs::Point point;
  for (unsigned int i = 0; i < points.size(); ++i)
  {
    point.x = points[i][0];  // + 2.0*(double)plotID;
    point.y = points[i][1];
    point.z = 0;
    marker.points.push_back(point);
  }
  plot_publisher_.publish(marker);
#endif
}
