#pragma once

#include <Eigen/Dense>
#include <vector>
#include <boost/iterator/iterator_concepts.hpp>
using namespace Eigen;
using namespace std;

//#define timeDelta (1.0/50)
#define DEBUGDRAW
#define NUM_LEGS 6
const double pi = M_PI; //< easier to read

// ifdef because assert(x) is nop on NDEBUG defined, not on 'DEBUG not defined'
#ifndef DEBUG
#define ASSERT(x) {}
 #define ATTEMPT(x_, y_) x_
#else
 #define ASSERT(x) assert(x)
 #define ATTEMPT(x_, y_) assert(x_ y_)
#endif

struct Parameters
{
  std::string hexapodType;
  double timeDelta;  
  bool imuCompensation;
  bool autoCompensation;  
  bool manualCompensation;

  //Hexapod Parameters
  Vector3d rootOffset[3][2];
  Vector3d hipOffset[3][2];
  Vector3d kneeOffset[3][2];
  Vector3d tipOffset[3][2];
  Vector3d stanceLegYaws;
  Vector3d physicalYawOffset;
  Vector3d yawLimits;
  Vector2d kneeLimits;
  Vector2d hipLimits;
  Vector3d jointMaxAngularSpeeds;
  bool dynamixelInterface;

  //Walk Controller Parameters
  std::string gaitType;  
  double stepFrequency;
  double stepClearance;
  double bodyClearance;
  double legSpanScale; 
  bool legStateCorrection;
  double maxAcceleration;
  double maxCurvatureSpeed;
  double stepCurvatureAllowance;
  double interfaceSetupSpeed;
  
  //Pose Controller Parameters
  bool startUpSequence;
  bool moveLegsSequentially;
  double timeToStart;
  double pitchAmplitude;
  double rollAmplitude;
  double maxPoseTime;
  double maxRoll;
  double maxPitch;
  double maxYaw;
  double maxX;
  double maxY;
  double maxZ;
  Vector3d packedJointPositionsAL;
  Vector3d packedJointPositionsAR;
  Vector3d packedJointPositionsBL;
  Vector3d packedJointPositionsBR;
  Vector3d packedJointPositionsCL;
  Vector3d packedJointPositionsCR;
    
  //Gait Parameters
  double stancePhase;
  double swingPhase;
  double phaseOffset;  
  std::vector<int> legSelectionPattern;
  std::vector<int> sideSelectionPattern;
  double transitionPeriod;
};

template<class T>
T sqr(const T &val){ return val*val; }

template<class T>
T sign(const T &val){ return val>0 ? 1 : -1; }

inline double solveQuadratic(double a, double b, double c, double *negative = NULL)
{
  double inside = sqr(b)-4.0*a*c;
  ASSERT(inside >= 0.0);
  if (negative != NULL)
  {
    double numerator = -b - sqrt(inside);
    if (2.0*a < numerator)
      *negative = 2.0*c / numerator;
    else
      *negative = numerator/(2.0*a);
  }
  double numerator = -b + sqrt(inside);
  if (2.0*a < numerator)
    return 2.0*c / numerator;
  return numerator/(2.0*a);
}

template <class T>
inline T cubicBezier(T *points, double t)
{
  double s = 1.0 - t;
  return points[0] * (s*s*s) +
         points[1] * (3.0*t*s*s) +
         points[2] * (3.0*t*t*s) +
         points[3] * (t*t*t);
}

template <class T>
inline T cubicBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (3*s*s*(points[1]-points[0]) + 
         6*s*t*(points[2]-points[1]) + 
         3*t*t*(points[3]-points[2]));                
}

inline Vector3d maxVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(max(a[0], b[0]), max(a[1], b[1]), max(a[2], b[2]));
}

inline Vector3d minVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]));
}

/// Past tense avoids confusion, result is passed back
template <class T>
T clamped(const T &value, const T &minValue, const T &maxValue)
{
  ASSERT(minValue <= maxValue);
  return max(minValue, min(value, maxValue));
}
// for Vector types
template <class T>
T clamped(const T &value, double magnitude)
{
  double magSqr = value.squaredNorm();
  if (magSqr > sqr(magnitude))
    return value / sqrt(magSqr);
  return value;
}


inline int roundToInt(double x)
{
  if (x >= 0)
    return int(x + 0.5);
  return -int(0.5 - x);
}

/// Uniform distribution within range
inline double random(double min, double max)
{
  return min + (max - min) * (double(rand()) / double(RAND_MAX));
}

/// Increasing list of integers from start to end inclusive
inline vector<int> ints(int start, int end, int step = 1)
{
  ASSERT(end >= start);
  vector<int> result;
  result.reserve((((end+1)-start) / step)+1);
  for (int i = start; i<=end; i+=step)
    result.push_back(i);
  return result;
}

/// Increasing list of doubles from start to end inclusive, increasing by step
inline vector<double> doubles(double start, double end, double step = 1.0) 
{
  ASSERT(end >= start && step > 0);
  vector<double> result;
  
  // The complication here is to make sure that you get the right result by passing in double(start, end, (end-start)/integerCount)
  
  unsigned int count = (unsigned int)(0.5 + (end - start) / step); // round
  if ((end - start) / double(count) != step) // for non-simple cases, just use the normal count calculation
    count = (unsigned int)((end - start) / step); 
  result.reserve(count+1);
  for (unsigned int i = 0; i<=count; ++i)
    result.push_back(start + double(i)*step); // I avoid incrementing by step due to drift for very small step
  return result;
}

/// Return mean of elements in the list
template <class T>
T mean(const vector<T> &list)
{
  ASSERT(list.size() > 0);
  T result = list[0];
  for (unsigned int i = 1; i<list.size(); i++)
    result += list[i];
  result /= double(list.size());
  return result;
}

/** Return median of elements in the list
 * When there are an even number of elements it returns the mean of the two medians
 */
template <class T>
T median(vector<T> list)
{
  typename vector<T>::iterator first = list.begin();
  typename vector<T>::iterator last = list.end();
  typename vector<T>::iterator middle = first + ((last - first) / 2);
  nth_element(first, middle, last); // can specify comparator as optional 4th arg
  if (list.size() % 2) // odd
    return *middle;
  else
  {
    typename vector<T>::iterator middle2 = middle+1;
    nth_element(first, middle2, last);
    return (*middle + *middle2)/2.0;
  }
}

