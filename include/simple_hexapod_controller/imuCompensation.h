#pragma once
#include "standardIncludes.h"
#include "sensor_msgs/Imu.h"
#include "debugOutput.h"
#include "pose.h"
#include <boost/circular_buffer.hpp> 

struct Imu
{
public:
  Imu(Parameters p);
  ~Imu();
  
  void init(Parameters p);
  
  Vector3d getRotationCompensation(Quat targetRotation);
  Vector3d getTranslationCompensation(Vector3d targetTranslation);
  
  void imuCompensation(const Vector3d &targetAccel, double targetAngularVel, double timeDelta);
  void setCompensationDebug(DebugOutput &debug);
  
  void calculatePassiveAngularFrequency(double timeDelta);
  
  Vector3d gaussianMean(double time, double timeStandardDeviation, double omega);
  Vector3d removeGravity(Quat orientation, Vector3d linearAcceleration);
  
  vector<Vector2d> queueToVector(const vector<Vector2d> &queue, int head, int tail);
  
  sensor_msgs::Imu data;
  
  Parameters params;
  
  DebugOutput *debugDraw = NULL;
  
  const int numAccs = 500;
  vector<Vector3d> accs;
  vector<double> times;
  //static int accIndex = 0;
  
  //static double t = 0;
  const double inputPhase = 0;

  double vel = 0.0;
  const int maxStates = 10000;
  //static const int numStates = 200;
  vector<Vector2d> states;
  vector<Vector2d> relativeStates;
  // poor man's dynamic size circular queue. Just keep a head and tail index
  int queueHead = 0; 
  int queueTail = 0; 
  double timex = 0.0;
  int lastHead = 0;

  Vector2d totalPhase;
  double totalNumerator = 0;
  double totalDenominator = 0;
  
  Vector3d rotationAbsementError;
  Vector3d rotationPositionError;
  Vector3d rotationVelocityError;
  
  Vector3d translationAbsementError;
  Vector3d translationPositionError;
  Vector3d translationVelocityError;
  Vector3d translationAccelerationError;
  
  //Vector3d translationCorrection; //Translation compensation output
  //Vector3d rotationCorrection;	//Rotation compensation output  
};