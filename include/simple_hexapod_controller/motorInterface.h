
#ifndef __MOTOR_INTERFACE_H__
#define __MOTOR_INTERFACE_H__

class MotorInterface
{
public:
  virtual void setTargetAngle(int leg_id, int side_id, int joint_id, double angle) = 0;
  virtual void setVelocity(int leg_id, int side_id, int joint_id, double velocity) = 0;
  virtual void setupSpeed(double speed) = 0;
  virtual void publish(void) = 0;
  virtual void setPGain(double p_gain) = 0;
};

#endif
