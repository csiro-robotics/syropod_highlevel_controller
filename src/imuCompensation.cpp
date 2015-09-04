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
static double t = 0;
static double inputPhase = 0;
Pose compensation(const Vector3d &targetAccel, double targetAngularVel)
{
//#define PHASE_ANALYSIS
#if defined(PHASE_ANALYSIS)
  Pose pose = Pose::identity();
  double driveFrequency = 12.4;  // angular frequency
  inputPhase = t * driveFrequency;
  pose.position[0] = 0.0025*sin(inputPhase);
  t += timeDelta;
  calculatePassiveAngularFrequency();
  return pose;
#endif
  Pose adjust;
  Quat orient;            //Orientation from IMU in quat
  Vector3d accel;         //Accelerations with respect to the IMU
  Vector3d rotation;
  Vector3d angularAcc;
  static Vector3d angularVel(0,0,0);
  adjust.rotation=Quat(Vector3d(0,0,0));
  //static boost::circular_buffer<float> cbx(4,0);
  //static boost::circular_buffer<float> cby(4,1);
  //static boost::circular_buffer<float> cbz(4,2);
  static Vector3d IMUPos(0,0,0);
  static Vector3d IMUVel(0,0,0);
  
  
  
  orient.w = imu.orientation.w;
  orient.x = imu.orientation.x;
  orient.y = imu.orientation.y;
  orient.z = imu.orientation.z;
  accel(1) = -imu.linear_acceleration.x;
  accel(0) = -imu.linear_acceleration.y;
  accel(2) = -imu.linear_acceleration.z;
  if (accel.squaredNorm()==0)
    return Pose::identity();
  
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
#define IMUINTEGRATION_FIRST_ORDER
//#define IMUINTEGRATION_SECOND_ORDER

#if defined(SECOND_ORDER_FEEDBACK)
  double imuStrength = 0.01;
  double stiffness = 0.5; // how strongly/quickly we return to the neutral pose
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
  
 #elif defined(IMUINTEGRATION_SECOND_ORDER)
  double imuStrength = 0.1;
  double decayRate = 1;
  IMUPos += IMUVel*timeDelta - decayRate*timeDelta*IMUPos;
  IMUVel += (targetAccel-accel+Vector3d(0, 0, 9.8))*timeDelta - decayRate*timeDelta*IMUVel;  
  Vector3d offsetAcc = -imuStrength*IMUPos;
  
#elif defined(IMUINTEGRATION_FIRST_ORDER)
  double imuStrength = 0.03;
  double decayRate =1;
  IMUVel += (targetAccel+accel-Vector3d(0, 0, 9.8))*timeDelta - decayRate*timeDelta*IMUVel; 
  //IMUVel(0) += (targetAccel(0)+accel(0))*timeDelta - decayRate*timeDelta*IMUVel(0);
  //IMUVel(1) += (targetAccel(1)+accel(1))*timeDelta- decayRate*timeDelta*IMUVel(1);
  //IMUVel(2) += (targetAccel(2)+accel(2)-9.8)*timeDelta - decayRate*timeDelta*IMUVel(2);
  //IMUVel = (IMUVel + (targetAccel+accel-Vector3d(0, 0, 9.8))*timeDelta)/(1.0 + decayRate*timeDelta);  
  //Vector3d offsetAcc = -imuStrength*(IMUVel + (accel-Vector3d(0,0,9.8))*0.06);
  Vector3d offsetAcc = -imuStrength*IMUVel;
  
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
  adjust.position = offsetAcc;
  return adjust;
}

static double vel = 0.0;
static const int maxStates = 10000;
//static const int numStates = 200;
vector<Vector2d> states(maxStates);
vector<Vector2d> relativeStates(maxStates);
// poor man's dynamic size circular queue. Just keep a head and tail index
static int queueHead = 0; 
static int queueTail = 0; 
static double timex = 0.0;
static int lastHead = 0;

static Vector2d totalPhase(0,0);
static double totalNumerator = 0;
static double totalDenominator = 0;

vector<Vector2d> queueToVector(const vector<Vector2d> &queue, int head, int tail)
{
  vector<Vector2d> result;
  for (int j = tail, i=j; i!=head; j++, i=(j%queue.size()))
    result.push_back(queue[i]);
  return result;
}

static Vector2d mean2(0,0);
static const int numAccs = 1;
static vector<Vector3d> accs(numAccs);
static int accIndex = 0;
void calculatePassiveAngularFrequency()
{
  accs[accIndex] = Vector3d(imu.linear_acceleration.y, imu.linear_acceleration.x, imu.linear_acceleration.z);
  accIndex = (accIndex + 1)%numAccs;
  Vector3d averageAcc = mean(accs);
  
  Vector2d newMean(0,0);
  int numStates = 0;
  for (int j = queueTail, i=j; i!=queueHead; j++, i=(j%maxStates))
  {
    newMean += states[i];
    numStates++;
  }
  if (numStates > 0)
    newMean /= (double)numStates;
  mean2 += (newMean-mean2)*1.0*timeDelta;
//  mean2 = newMean;
  
  vector<Vector3d> ps(1);
  ps[0] = Vector3d(mean2[0], mean2[1], 0);
  debugDraw->drawPoints(ps, Vector4d(1,1,1,1));

  // just 1 dimension for now
  
  double acc = averageAcc[0]; // CHANGE when doign a different axis
  double inputAngle = inputPhase;
  double decayRate = 0.1;
  // lossy integrator
  vel = (vel + (acc - mean2[1])*timeDelta) / (1.0 + decayRate*timeDelta);
  states[queueHead] = Vector2d(vel, acc);

  // increment 
  timex += timeDelta;
  const int noiseRobustnessCount = 30;
  if (numStates > noiseRobustnessCount)
  {
    int tail = queueHead;
    Vector2d rollingSum(0,0);
    double count = 0;
    bool returnedToPositiveDot = false;
    while (tail != queueTail)
    {
      rollingSum += states[tail];
      count++;
      
      if (count > noiseRobustnessCount)
      {
        Vector2d headState = states[queueHead] - (rollingSum / count);
        Vector2d tailState = states[tail] - (rollingSum / count);
        double dot = headState.dot(tailState);
        double cross = headState.dot(Vector2d(tailState[1], -tailState[0]));
        if (!returnedToPositiveDot)
        {
          if (dot > 0.0)
            returnedToPositiveDot = true;
        }
        else
        {
          if (dot > 0.0 && cross > 0.0)
          {
            queueTail = tail;
            break;
          }
        }
      }
      tail = (tail + maxStates-1) % maxStates;
    }
  }

  
  lastHead = queueHead;
  queueHead = (queueHead + 1) % maxStates; // poor man's circular queue!
    
  debugDraw->plot(queueToVector(states, queueHead, queueTail)); // should look noisy and elliptical

  
  Vector2d sumSquare(0,0);
  for (int j = queueTail, i=j; i!=queueHead; j++, i=(j%maxStates))
    sumSquare += Vector2d(sqr(states[i][0] - mean2[0]), sqr(states[i][1] - mean2[1]));
  totalNumerator += sumSquare[1];
  totalDenominator += sumSquare[0];
  double omega = sqrt(sumSquare[1]) / (sqrt(sumSquare[0]) + 1e-10);
  cout << "rolling omega estimate: " << omega << ", running omega estimate: " << sqrt(totalNumerator) / (sqrt(totalDenominator)+1e-10) << endl;
  
  vector<Vector2d> normalisedStates(maxStates);
  for (int j = queueTail, i=j; i!=queueHead; j++, i=(j%maxStates))
  {
    normalisedStates[i] = states[i] - mean2;
    normalisedStates[i][0] *= omega;
  }
  debugDraw->plot(queueToVector(normalisedStates, queueHead, queueTail)); // this should look noisy but circularish
  
  // next, based on omega, lets plot the points 'unrotated' by the angular rate, to get a phase offset
  
  Vector2d sumUnrotated(0,0);
  double theta = -inputAngle;
//  double y = normalisedStates[lastHead][0] * cos(theta) + normalisedStates[lastHead][1] * sin(theta);
//  double x = normalisedStates[lastHead][0] * -sin(theta) + normalisedStates[lastHead][1] * cos(theta);
  // i.e. y = acc = sin(theta)  <-- if offset = sin(theta) then you'd expect the drive force to be also proportional to sin(theta)
  // i.e. x = vel = -cos(theta)
  double x = normalisedStates[lastHead][0] * cos(theta) + normalisedStates[lastHead][1] * sin(theta);
  double y = normalisedStates[lastHead][0] * -sin(theta) + normalisedStates[lastHead][1] * cos(theta);
  relativeStates[lastHead] = Vector2d(x,y);
  for (int j = queueTail, i=j; i!=queueHead; j++, i=(j%maxStates))
    sumUnrotated += relativeStates[i];
  totalPhase += sumUnrotated;
  double phaseOffset = atan2(sumUnrotated[1], -sumUnrotated[0]);
  double runningPhaseOffset = atan2(totalPhase[1], -totalPhase[0]);
  cout << "rolling phase offset estimate: " << phaseOffset << ", running phase offset estimate: " << runningPhaseOffset << endl;
  debugDraw->plot(queueToVector(relativeStates, queueHead, queueTail)); // this should cluster around a particular phase
}
