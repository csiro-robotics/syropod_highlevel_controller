/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/imuCompensation.h"
#include <boost/circular_buffer.hpp> 
#include "../include/simple_hexapod_controller/debugOutput.h"

sensor_msgs::Imu imu;
// target rather than measured data
static Vector3d offsetPos(0.0,0.0,0.0);
static Vector3d offsetVel(0,0,0);
static DebugOutput *debugDraw = NULL;

void calculatePassiveAngularFrequency();
void setCompensationDebug(DebugOutput &debug)
{
  debugDraw = &debug;
}

void imuCallback(const sensor_msgs::Imu &imudata)
{  
  imu = imudata;
}

Pose compensation(const Vector3d &targetAccel, double targetAngularVel)
{
#if defined(PHASE_ANALYSIS)
  calculatePassiveAngularFrequency();
  return Pose::identity();
#endif
  Pose adjust;
  Quat orient;            //Orientation from IMU in quat
  Vector3d accel;         //Accelerations with respect to the IMU
  Vector3d rotation;
  Vector3d angularAcc;
  static Vector3d angularVel(0,0,0);
  adjust.rotation=Quat(Vector3d(0,0,0));
  static boost::circular_buffer<float> cbx(4,0);
  static boost::circular_buffer<float> cby(4,1);
  static boost::circular_buffer<float> cbz(4,2);
 
  orient.w = imu.orientation.w;
  orient.x = imu.orientation.x;
  orient.y = imu.orientation.y;
  orient.z = imu.orientation.z;
  accel(1) = -imu.linear_acceleration.x;
  accel(0) = -imu.linear_acceleration.y;
  accel(2) = -imu.linear_acceleration.z;
  
  /*//Compensation for gravity
  Vector3d accelcomp;
  RowVector3d gravityrot; 
  gravityrot=orient.toRotationMatrix()*Vector3d(0,0,-9.81);
  accelcomp(0)=accel(0)-gravityrot(0);
  accelcomp(1)=accel(1)-gravityrot(1);  
  accelcomp(2)=accel(2)-gravityrot(2);  
  ROS_ERROR("ACCEL= %f %f %f", accel(0), accel(1),accel(2));0
  ROS_ERROR("GRAVITYROT= %f %f %f", gravityrot(0), gravityrot(1),gravityrot(2)); 
  ROS_ERROR("ACCELCOMP= %f %f %f", accelcomp(0), accelcomp(1),accelcomp(2));*/
  
//Postion compensation

//#define ZERO_ORDER_FEEDBACK
//#define FIRST_ORDER_FEEDBACK
//#define SECOND_ORDER_FEEDBACK

#if defined(SECOND_ORDER_FEEDBACK)
  double imuStrength = 1;
  double stiffness = 13; // how strongly/quickly we return to the neutral pose
  Vector3d offsetAcc = imuStrength*(targetAccel-accel+Vector3d(0,0,9.8)) - sqr(stiffness)*offsetPos - 2.0*stiffness*offsetVel;
  offsetVel += offsetAcc*timeDelta;
  offsetPos += offsetVel*timeDelta;
  
#elif defined(FIRST_ORDER_FEEDBACK)
  double imuStrength = 0.5;
  double stiffness = 10; // how strongly/quickly we return to the neutral pose
  offsetVel = imuStrength*(targetAccel-accel+Vector3d(0,0,9.8)) - sqr(stiffness)*offsetPos;
  offsetPos += offsetVel*timeDelta;
  
#elif defined(ZERO_ORDER_FEEDBACK)
  double imuStrength = 0.001;
  cbx.push_back(-imu.linear_acceleration.y); 
  cby.push_back(-imu.linear_acceleration.x); 
  cbz.push_back(-imu.linear_acceleration.z);
  offsetPos(0) = imuStrength * (targetAccel(0)-(cbx[0]+cbx[1]+cbx[2]+cbx[3]+cbx[3])/4);
  offsetPos(1) = imuStrength * (targetAccel(1)-(cby[0]+cby[1]+cby[2]+cby[3]+cbx[3])/4);
  offsetPos(2) = imuStrength * (targetAccel(2)-(cbz[0]+cbz[1]+cbz[2]+cbz[3]+cbx[3])/4 + 9.8);
  
  //offsetPos = imuStrength * (targetAccel-accel+Vector3d(0,0,9.8));
 
#endif
  
  //Angular body velocity compensation.
  double stiffnessangular=5;
  Vector3d angleDelta = adjust.rotation.toRotationVector();  
  //angularAcc(0)=-sqr(stiffnessangular)*angleDelta(0) + 2.0*stiffnessangular*(imu.angular_velocity.y - angularVel(0));
  //angularAcc(1)=-sqr(stiffnessangular)*angleDelta(1) + 2.0*stiffnessangular*(imu.angular_velocity.x - angularVel(1));
  angularAcc= -sqr(stiffnessangular)*angleDelta + 2.0*stiffnessangular*(Vector3d(0,0,targetAngularVel) - Vector3d(-imu.angular_velocity.y, -imu.angular_velocity.x, -imu.angular_velocity.z) - angularVel);
  angularVel += angularAcc*timeDelta;
  rotation(0)=angularVel(0)*timeDelta;
  rotation(1)=angularVel(1)*timeDelta;
  rotation(2)=angularVel(2)*timeDelta;
  
  /*// control towards imu's orientation
  double stiffnessangular=15;
  Quat targetAngle = ~orient;
  Vector3d angleDelta = (targetAngle*(~adjust.rotation)).toRotationVector(); // diff=target*current^-1
  angleDelta[2] = 0;  // this may not be quite right  
  angularAcc = sqr(stiffnessangular)*angleDelta -2.0*stiffnessangular*angularVel;
  angularVel += angularAcc*timeDelta;  
  rotation(0)=angularVel(0)*timeDelta;
  rotation(1)=angularVel(1)*timeDelta;
  rotation(2)=angularVel(2)*timeDelta;*/
  
  //adjust.rotation*= Quat(rotation);  
  adjust.position = offsetPos;
  return adjust;
}

static double vel = 0.0;
static const int numStates = 200;
vector<Vector2d> states(numStates);
static int stateIndex = 0; 
static double timex = 0.0;

void calculatePassiveAngularFrequency()
{
  // just 1 dimension for now
  double acc = imu.linear_acceleration.y;
  double decayRate = 0.25;
  // lossy integrator
  vel = (vel + acc*timeDelta) / (1.0 + decayRate*timeDelta);
  states[stateIndex] = Vector2d(vel, acc);
  stateIndex = (stateIndex + 1) % numStates; // poor man's circular queue!
  debugDraw->plot(states); // should look noisy and elliptical

  Vector2d sumSquare(0,0);
  for (int i = 0; i<numStates; i++)
    sumSquare += Vector2d(sqr(states[i][0]), sqr(states[i][1]));
  double omega = sqrt(sumSquare[1]) / (sqrt(sumSquare[0]) + 1e-10);
  cout << "rolling omega estimate: " << omega << endl;
  
  vector<Vector2d> normalisedStates(numStates);
  for (int i = 0; i<numStates; i++)
  {
    normalisedStates[i] = states[i];
    normalisedStates[i][0] *= omega;
  }
  debugDraw->plot(normalisedStates); // this should look noisy but circularish
  
  // next, based on omega, lets plot the points 'unrotated' by the angular rate, to get a phase offset
  vector<Vector2d> unrotatedStates(numStates);
  Vector2d sumUnrotated(0,0);
  for (int i = 0; i<numStates; i++)
  {
    double theta = -omega * timex * (double)(-numStates + (i+numStates-stateIndex)%numStates)*timeDelta;
    double x = normalisedStates[i][0] * sin(theta) + normalisedStates[i][1] * cos(theta);
    double y = normalisedStates[i][0] * cos(theta) + normalisedStates[i][1] * -sin(theta);
    unrotatedStates[i] = Vector2d(x,y);
    sumUnrotated += unrotatedStates[i];
  }
  double phaseOffset = atan2(sumUnrotated[0], sumUnrotated[1]);
  cout << "rolling phase offset estimate: " << phaseOffset << endl;
  debugDraw->plot(unrotatedStates); // this should cluster around a particular phase
  
  timex += timeDelta;
}
