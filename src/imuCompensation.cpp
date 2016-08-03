/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/imuCompensation.h"

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
Imu::Imu(void)
{
  offsetPos = {0.0,0.0,0.0};
  offsetVel = {0.0,0.0,0.0};
  
  accs = vector<Vector3d>(numAccs);
  times = vector<double>(numAccs);
  
  states = vector<Vector2d>(maxStates);
  relativeStates = vector<Vector2d>(maxStates);
  
  totalPhase = {0,0};
  
  data = sensor_msgs::Imu();
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
Imu::~Imu(void)
{
}
 
/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
void Imu::setCompensationDebug(DebugOutput &debug)
{
  debugDraw = &debug;
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
Vector3d Imu::gaussianMean(double time, double timeStandardDeviation, double omega)
{
  Vector3d meanAcc(0,0,0);
  double totalWeight = 0.0;
  double sigma = timeStandardDeviation;
  double testTime = 0;
  for (int i = 0; i<numAccs; i++)
  {
    double tError = times[i] - time;
    //double weight = cos(tError*omega)*exp(-sqr(tError)/(2.0*sqr(sigma)));
    double weight = exp(-sqr(tError)/(2.0*sqr(sigma)));
    meanAcc += accs[i]*weight;
    testTime += times[i]*weight;
    totalWeight += weight;
  }
  meanAcc /= totalWeight;
  testTime /= totalWeight;
//  ASSERT(abs(time - testTime) < 0.1);
  return meanAcc;
}


/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
void Imu::imuCompensation(const Vector3d &targetAccel, double targetAngularVel, Vector3d *deltaAngle, Vector3d *deltaPos, double pIncrement, double timeDelta)
{
//#define PHASE_ANALYSIS
#if defined(PHASE_ANALYSIS)
  Pose pose = Pose::identity();
  double driveFrequency = 12.4;  // angular frequency
  inputPhase = t * driveFrequency;
  pose.position[0] = 0.0025*sin(inputPhase);
  t += timeDelta;
  calculatePassiveAngularFrequency();
  return pose.position;
#endif
  Pose adjust;
  Quat orient;            //Orientation from IMU in quat
  Vector3d accel;         //Accelerations with respect to the IMU
  Vector3d rotation; 
  Vector3d angVel;
  Vector3d angularAcc;
  
  
  static Vector3d angularVel(0,0,0);
  adjust.rotation=Quat(Vector3d(0,0,0));
  static Vector3d IMUAbs(0,0,0);
  static Vector3d IMUPos(0,0,0);
  static Vector3d IMUVel(0,0,0);
  
  orient.w = data.orientation.w;
  orient.x = data.orientation.x;
  orient.y = data.orientation.y;
  orient.z = data.orientation.z;
  
  accel(0) = data.linear_acceleration.x;
  accel(1) = data.linear_acceleration.y;
  accel(2) = data.linear_acceleration.z;
  
  angVel(0) = data.angular_velocity.x;
  angVel(1) = data.angular_velocity.y;
  angVel(2) = data.angular_velocity.z;
  
// #define GENERIC_FEEDBACK_CONTROL  
#if defined(GENERIC_FEEDBACK_CONTROL)
  double omega = 10.0;
  const int numPrevious = 1000;
  static double time = 0;
  static int accIndex = 0;
  static Vector3d previousAccelerations[numPrevious];
  static double previousTimes[numPrevious];
  previousAccelerations[accIndex] = accel - Vector3d(0, 0, 9.8);
  previousTimes[accIndex] = time;
  accIndex = (accIndex+1) % numPrevious;
  time += timeDelta;
  
  Vector3d totalAcc(0,0,0);
  for (int i = 0; i<numPrevious; i++)
  {
    double t = (time - previousTimes[i]) * omega;
    double scale = t*exp(-0.7*t);
    totalAcc += previousAccelerations[i]*scale;
  }
  totalAcc /= (double)numPrevious;
  const double strength = 0.1;
  offsetPos = -totalAcc * strength;
  
  *deltaPos = offsetPos;
  *deltaAngle = Vector3d(0,0,0);
#endif
  if (accel.squaredNorm()==0)
  {
    *deltaAngle = Vector3d(0,0,0);
    *deltaPos = Vector3d(0,0,0);
  }
  
  /*//Compensation for gravity
  Vector3d accelcomp;
  RowVector3d gravityrot; 
  gravityrot=orient.toRotationMatrix()*Vector3d(0,0,-9.81);
  accelcomp(0)=accel(0)-gravityrot(0);
  accelcomp(1)=accel(1)-gravityrot(1);  
  accelcomp(2)=accel(2)-gravityrot(2); 
  
  ROS_ERROR("ACCEL= %f %f %f", accel(0), accel(1),accel(2));
  ROS_ERROR("GRAVITYROT= %f %f %f", gravityrot(0), gravityrot(1),gravityrot(2)); 
  ROS_ERROR("ACCELCOMP= %f %f %f", accelcomp(0), accelcomp(1),accelcomp(2));*/
  
#define IMUINTEGRATION_FIRST_ORDER
//#define IMUINTEGRATION_SECOND_ORDER
//#define FILTERED_DELAY_RESPONSE
//#define BEST_CONTROLLER_ON_EARTH
#define ANGULAR_COMPENSATION

#if defined(IMUINTEGRATION_SECOND_ORDER)
  double imuStrength = 0.03;
  double decayRate = 1;
  IMUPos += IMUVel*timeDelta - decayRate*timeDelta*IMUPos;
  IMUVel += (targetAccel-accel+Vector3d(0, 0, 9.8))*timeDelta - decayRate*timeDelta*IMUVel;  
  Vector3d offsetPos = -imuStrength*IMUPos;

#elif defined(BEST_CONTROLLER_ON_EARTH)
  double kp = 10.0*timeDelta;
  double kd = -1.0*timeDelta;
  double ki = 0.0;
    
  IMUVel += (accel - Vector3d(0, 0, 9.80665))*timeDelta;
  IMUPos += timeDelta*IMUVel;
  //IMUVel += (targetAccel - decayRate*timeDelta*IMUVel;
  //IMUPos += IMUVel*timeDelta - decayRate*timeDelta*IMUPos;
  //IMUVel = (IMUVel + (targetAccel+accel-Vector3d(0, 0, 9.8))*timeDelta)/(1.0 + decayRate*timeDelta);  
  //Vector3d offsetAcc = -imuStrength*(IMUVel + (accel-Vector3d(0,0,9.8))*0.06);
  //Vector3d offsetAcc = -imuStrength*IMUVel-stiffness*IMUPos;
  Vector3d controlPositionDelta = kp*(desiredPosition - IMUPos) + kd*(desiredVelocity - IMUVel);
  offsetPos = IMUPos + controlPositionDelta;
  
#elif defined(IMUINTEGRATION_FIRST_ORDER)
  double kD = 0.00;
  double decayRate = 2.3; 
  double kP = 0.25; 
  double kI = 0.0;
  //double stiffness = 0; 
  Vector3d dynamicLinearAcc = removeGravity(orient, accel);
  
  IMUVel += (dynamicLinearAcc - targetAccel)*timeDelta - decayRate*timeDelta*IMUVel;
  IMUPos += IMUVel*timeDelta - decayRate*timeDelta*IMUPos;
  IMUAbs += IMUPos*timeDelta;
  
  //cout << targetAccel[0] << "\t" << targetAccel[1] << "\t" << targetAccel[2] << endl;
  //cout << "ABS: " << IMUAbs[2] << "\tPOS: " << IMUPos[2] << "\tVEL: " << IMUVel[2] << "\tACC: " << dynamicLinearAcc[2] << endl;
  //cout << "X: " << dynamicLinearAcc[0] << "\tY: " << dynamicLinearAcc[1] << "\tZ: " << dynamicLinearAcc[2] << endl;
  //IMUVel = (IMUVel + (targetAccel+accel-Vector3d(0, 0, 9.8))*timeDelta)/(1.0 + decayRate*timeDelta);  
  //Vector3d offsetAcc = -imuStrength*(IMUVel + (accel-Vector3d(0,0,9.8))*0.06);
  
  Vector3d offsetPos = kD*IMUVel + (kP + pIncrement)*IMUPos + kI*IMUAbs;
  offsetPos[0] *= -1; //???
  Vector3d offsetAng = dynamicLinearAcc;
  
  cout << "IMUVel: " << IMUVel[0] << "\t" << IMUVel[1];

#if defined(ANGULAR_COMPENSATION)
  double angularD = 0.00;
  double angularP = 1.00;

  Quat targetOrient(1,0,0,0);
  // since there are two orientations per quaternion we want the shorter/smaller difference. 
  // not certain this is needed though
  {
  double dot = targetOrient.dot(orient);
  if (dot < 0.0)
    targetOrient = -targetOrient;
  }
  
  Quat diff = targetOrient*(~orient);
  Vector3d diffVec = diff.toRotationVector();
  
  diffVec[2] = 0.0;    
  angVel[2] = 0.0;  
  
  //Vector3d offsetAng = angularD*angVel + angularP*diffVec;
#endif
  
#elif defined(FILTERED_DELAY_RESPONSE)
  const double omega = 11; // natural angular frequency 
  const double spreadRatio = 0.7; // small is more susceptible to noise
  const double strength = 2;
  t += timeDelta;
  accs[accIndex] = /*targetAccel + */ accel - Vector3d(0, 0, 9.80665);
  times[accIndex] = t;
  accIndex = (accIndex + 1)%numAccs;
  const double timeDelay = 0.5 / (omega / (2.0*pi));
  Vector3d offsetPos = strength * gaussianMean(t - timeDelay, spreadRatio*timeDelay, omega);  
    
Vector3d desiredVelocity(0.0, 0.0, 0.0);
Vector3d desiredPosition(0.0, 0.0, 0.0);

#endif  
  //adjust.rotation*= Quat(rotation);    
  *deltaAngle = offsetAng;
  *deltaPos = offsetPos; //Negative to work
 
  
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
Vector3d Imu::removeGravity(Quat orientation, Vector3d linearAcceleration)
{
  Vector3d gravity = {0.0,0.0,-9.78575};
  
  Vector3d orientedAcc = orientation.rotateVector(linearAcceleration);
  
  return orientedAcc - gravity;
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
vector<Vector2d> Imu::queueToVector(const vector<Vector2d> &queue, int head, int tail)
{
  vector<Vector2d> result;
  for (int j = tail, i=j; i!=head; j++, i=(j%queue.size()))
    result.push_back(queue[i]);
  return result;
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/
void Imu::calculatePassiveAngularFrequency(double timeDelta)
{
  static Vector2d mean2(0,0);
  static const int numAccs = 1;
  static vector<Vector3d> accs(numAccs);
  static int accIndex = 0;
  
  accs[accIndex] = Vector3d(data.linear_acceleration.y, data.linear_acceleration.x, data.linear_acceleration.z);
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
