#include "ros/ros.h"
#include "motorInterface.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include "standardIncludes.h"

class DynamixelProMotorInterface : public MotorInterface
{
public:

  DynamixelProMotorInterface();

  void setTargetAngle(int legID, int side, int jointID, double speed);		
  void setupSpeed(double speed);		
  virtual void publish(void);

private:

  ros::NodeHandle nodehandler;
  ros::Publisher motor_pub;
  vector<vector<vector<double> > > angles;
};
