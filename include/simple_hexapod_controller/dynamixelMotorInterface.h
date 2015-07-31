#include "ros/ros.h"
#include "motorInterface.h"
#include "std_msgs/Float64.h"
#include "standardIncludes.h"
#include "dynamixel_controllers/SetSpeed.h"

class DynamixelMotorInterface : public MotorInterface
{
public:
  DynamixelMotorInterface() : publishers(3, vector<vector<ros::Publisher> >(2,vector<ros::Publisher>(3))), angles(3, vector<vector<double > >(2, vector<double >(3)))
  {
    setupPublishers();
  }
  
  void setTargetAngle(int legID, int side, int jointID, double angle);		
  void setupSpeed(double speed);
  void publish(void);
 
private:
  ros::ServiceClient setspeed1;
  ros::NodeHandle nodehandler;
  void setupPublishers(void);
  ros::Publisher publicador;
  vector<vector<vector<ros::Publisher> > > publishers;
  vector<vector<vector<double> > > angles;
};
