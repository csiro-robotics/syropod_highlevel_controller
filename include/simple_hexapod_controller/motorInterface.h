
#ifndef __MOTOR_INTERFACE_H__
#define __MOTOR_INTERFACE_H__

class MotorInterface
{
public:
  virtual void setTargetAngle(int legID, int side, int jointID, double angle) = 0;		
  virtual void setupSpeed(double speed) = 0;		
  virtual void publish(void) = 0;
  virtual void setPGain(double Pgain) = 0;
};

#endif
