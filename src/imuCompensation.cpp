#include "../include/simple_hexapod_controller/imuCompensation.h"

/***********************************************************************************************************************
 * Constructor
***********************************************************************************************************************/
Imu::Imu(Parameters p)
{ 
  data = sensor_msgs::Imu();  
  init(p);
}

/***********************************************************************************************************************
 * Destructor
***********************************************************************************************************************/
Imu::~Imu(void)
{
}

/***********************************************************************************************************************
 * Initialisation
***********************************************************************************************************************/
void Imu::init(Parameters p)
{
  params = p;
  
  rotationAbsementError = Vector3d(0,0,0);
  rotationPositionError = Vector3d(0,0,0);
  rotationVelocityError = Vector3d(0,0,0);
  
  translationAbsementError = Vector3d(0,0,0);
  translationPositionError = Vector3d(0,0,0);
  translationVelocityError = Vector3d(0,0,0);
  translationAccelerationError = Vector3d(0,0,0);
}
 
/***********************************************************************************************************************
 * Assigns debug object
***********************************************************************************************************************/
void Imu::setCompensationDebug(DebugOutput &debug)
{
  debugDraw = &debug;
}

/***********************************************************************************************************************
 * Rotates gravity acceleration vector by orientation quaterian and subtracts from acceleration vector
***********************************************************************************************************************/
Vector3d Imu::removeGravity(Quat orientation, Vector3d linearAcceleration)
{
  Vector3d gravity = {0.0,0.0,9.75};
  
  Vector3d orientedGravity = orientation.rotateVector(gravity);  
  
  return linearAcceleration - orientedGravity;
}

/***********************************************************************************************************************
 * Returns current pitch/roll/yaw accoding to IMU
***********************************************************************************************************************/
Vector3d Imu::getCurrentRotation(void)
{
  Quat orientation; //IMU Orientation
  
  //Get orientation data from IMU
  orientation.w = data.orientation.w;
  orientation.x = data.orientation.x;
  orientation.y = data.orientation.y;
  orientation.z = data.orientation.z;
  
  return orientation.toEulerAngles();
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
Vector3d Imu::getRotationCompensation(Quat targetRotation)
{
  double timeDelta = params.timeDelta;
  
  Quat orientation; 		//IMU Orientation
  Vector3d angularVelocity;	//IMU Angular Velocity
  
  //Get orientation data from IMU
  orientation.w = data.orientation.w;
  orientation.x = data.orientation.x;
  orientation.y = data.orientation.y;
  orientation.z = data.orientation.z;
  
  //Get angular velocity data from IMU
  angularVelocity(0) = data.angular_velocity.x;
  angularVelocity(1) = data.angular_velocity.y;
  angularVelocity(2) = data.angular_velocity.z;
  
  //PID gains
  double kD = params.rotationCompensationDerivativeGain;
  double kP = params.rotationCompensationProportionalGain;
  double kI = params.rotationCompensationIntegralGain;
  
  //There are two orientations per quaternion and we want the shorter/smaller difference. 
  double dot = targetRotation.dot(~orientation);
  if (dot < 0.0)
  {
    targetRotation = -targetRotation;
  }
  
  rotationPositionError = orientation.toEulerAngles();// - targetRotation.toEulerAngles();
  rotationAbsementError += rotationPositionError*timeDelta; //Integration of angle position error (absement)
  
  //Low pass filter of IMU angular velocity data
  double smoothingFactor = 0.15;
  rotationVelocityError = smoothingFactor*angularVelocity + (1-smoothingFactor)*rotationVelocityError;
  
  Vector3d rotationCorrection = kD*rotationVelocityError + kP*rotationPositionError + kI*rotationAbsementError;
  rotationCorrection[2] = 0; //No compensation in yaw rotation
  
  return -rotationCorrection;
}

/***********************************************************************************************************************
 * Returns x & y translation values to compensate for x/y accelerations of IMU and keep body at target position
***********************************************************************************************************************/
Vector3d Imu::getTranslationCompensation(Vector3d targetTranslation)
{
  double timeDelta = params.timeDelta;
  
  Quat orientation;	//IMU Orientation
  Vector3d linearAcceleration;	//IMU Linear Acceleration
  
  //Get orientation data from IMU
  orientation.w = data.orientation.w;
  orientation.x = data.orientation.x;
  orientation.y = data.orientation.y;
  orientation.z = data.orientation.z;
  
  //Get linear acceleration data from IMU
  linearAcceleration(0) = data.linear_acceleration.x;
  linearAcceleration(1) = data.linear_acceleration.y;
  linearAcceleration(2) = data.linear_acceleration.z;
  
  //PID gains
  double kD = params.translationCompensationDerivativeGain;
  double kP = params.translationCompensationProportionalGain;
  double kI = params.translationCompensationIntegralGain;
  
  double decayRate = 2.3; 
  
  //Low pass filter of IMU linear acceleration data (after removing acceleration due to gravity)  
  Vector3d dynamicLinearAcceleration = removeGravity(orientation, linearAcceleration);
  double smoothingFactor = 0.15;
  translationAccelerationError = smoothingFactor*dynamicLinearAcceleration + (1-smoothingFactor)*translationAccelerationError;
  
  //Integrate for velocity and position and absement
  translationVelocityError += translationAccelerationError*timeDelta - decayRate*timeDelta*translationVelocityError;
  translationPositionError += translationVelocityError*timeDelta - decayRate*timeDelta*translationPositionError;
  translationAbsementError += translationPositionError*timeDelta;
  
  Vector3d translationCorrection = kD*translationVelocityError + kP*translationPositionError + kI*translationAbsementError;  
  translationCorrection[2] = 0; //No compensation in z translation (competes with impedance controller)
  
  return translationCorrection;
}

/***********************************************************************************************************************
***********************************************************************************************************************/

