#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "standardIncludes.h"
#include "dynamixel_controllers/SetSpeed.h"

class MotorInterface
{
public:
  MotorInterface() : publishers(3, vector<vector<ros::Publisher> >(2,vector<ros::Publisher>(3))) 
  {
    setupPublishers();		
  }		

  void setTargetAngle(int &legID,int &side,int &jointID, std_msgs::Float64& msg);		
  void setupSpeed(dynamixel_controllers::SetSpeed& speed);		

private:
  ros::ServiceClient setspeed1;

  ros::NodeHandle nodehandler;
  void setupPublishers(void);
  ros::Publisher publicador;
  vector<vector<vector<ros::Publisher> > > publishers;
};
