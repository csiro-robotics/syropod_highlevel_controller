#include "simple_hexapod_controller/debugOutput.h"
#include <visualization_msgs/Marker.h>
#include <sstream>

DebugOutput::DebugOutput()
{
#if defined(DEBUGDRAW)
  robot_publisher_ = n.advertise<visualization_msgs::Marker>("robot", 1);
  points_publisher_ = n.advertise<visualization_msgs::Marker>("points", 1);
  reset();
#endif
}

void DebugOutput::drawRobot(Model* model, const Vector4d &colour, bool static_display)
{
#if defined(DEBUGDRAW)
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "/my_frame";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "trajectory_polyline_" + to_string(robot_ID_);
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.x = 0.0;
  line_list.pose.orientation.y = 0.0;
  line_list.pose.orientation.z = 0.0;
  line_list.pose.orientation.w = 1.0;

  line_list.id = robot_ID_++;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.005;  // only x component needed
  line_list.scale.y = 0.005;  // only x component needed

  line_list.color.r = colour[0];
  line_list.color.g = colour[1];
  line_list.color.b = colour[2];
  line_list.color.a = colour[3];

  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;    
    
    //Draw body
    Joint* first_joint = leg->getJointContainer()->begin()->second;
    Vector3d first_joint_position = first_joint->getCartesianOffset();
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    line_list.points.push_back(point);
    point.x = first_joint_position[0];
    point.y = first_joint_position[1];
    point.z = first_joint_position[2];    
    line_list.points.push_back(point);
    Vector3d previous_joint_position = first_joint_position;   
    
    //Draw legs
    std::map<int, Joint*>::iterator joint_it;    
    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;      
      point.x = previous_joint_position[0];
      point.y = previous_joint_position[1];
      point.z = previous_joint_position[2];
      line_list.points.push_back(point);
      Vector3d joint_position = joint->getCartesianOffset(previous_joint_position);
      point.x = joint_position[0];
      point.y = joint_position[1];
      point.z = joint_position[2];
      line_list.points.push_back(point); 
      previous_joint_position = joint_position;
    }
    point.x = previous_joint_position[0];
    point.y = previous_joint_position[1];
    point.z = previous_joint_position[2];
    line_list.points.push_back(point);
    Vector3d tip_point = leg->getLocalTipPosition();    
    point.x = tip_point[0];
    point.y = tip_point[1];
    point.z = tip_point[2];
    line_list.points.push_back(point);     
  }
  robot_publisher_.publish(line_list);
#endif
}

void DebugOutput::drawPoints(Model* model, const Vector4d &colour, bool static_display)
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
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = model->getLegContainer()->begin(); leg_it != model->getLegContainer()->end(); ++leg_it)
  {
    Leg* leg = leg_it->second;    
    Pose pose = (static_display ? Pose::identity() : odometry_pose_);
    Vector3d tip_position = pose.transformVector(leg->getLocalTipPosition());
    point.x = tip_position[0];
    point.y = tip_position[1];
    point.z = tip_position[2];
    marker.points.push_back(point);
  }
  points_publisher_.publish(marker);
#endif
}
