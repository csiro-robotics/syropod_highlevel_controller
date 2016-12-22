#include "ros/ros.h"
#include "motorInterface.h"
#include "std_msgs/Float64.h"
#include "standardIncludes.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"

class DynamixelMotorInterface : public MotorInterface
{
public:
  DynamixelMotorInterface()
    : position_publishers_(3, vector<vector<ros::Publisher> >(2, vector<ros::Publisher>(3)))
    , velocity_publishers_(3, vector<vector<ros::Publisher> >(2, vector<ros::Publisher>(3)))
    , angles_(3, vector<vector<double> >(2, vector<double>(3)))
    , velocities_(3, vector<vector<double> >(2, vector<double>(3)))
  {
    setupPublishers();
  }

  void setTargetAngle(int leg_ID, int side, int joint_ID, double angle);
  void setVelocity(int leg_ID, int side, int joint_ID, double velocity);
  void setupSpeed(double speed);
  void publish(void);
  void setPGain(double p_gain);

private:
  ros::ServiceClient set_speed_1_;
  ros::ServiceClient set_gain_p_;
  ros::NodeHandle n_;  
  vector<vector<vector<ros::Publisher> > > position_publishers_;
  vector<vector<vector<ros::Publisher> > > velocity_publishers_;
  vector<vector<vector<double> > > angles_;
  vector<vector<vector<double> > > velocities_;
  
  void setupPublishers(void);
};
