#include "ros/ros.h"
#include "motorInterface.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include "standardIncludes.h"

class DynamixelProMotorInterface : public MotorInterface
{
public:
  DynamixelProMotorInterface();

  void setTargetAngle(int leg_id, int side, int joint_id, double angle);
  void setVelocity(int leg_id, int side, int joint_id, double velocity);
  void setupSpeed(double speed);
  virtual void publish(void);
  void setPGain(double p_gain);

private:
  ros::NodeHandle n_;
  ros::Publisher motor_publisher_;
  vector<vector<vector<double> > > angles_;
  vector<vector<vector<double> > > velocities_;
};
