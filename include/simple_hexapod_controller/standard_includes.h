#ifndef SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
#define SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
/*******************************************************************************************************************//**
 *  \file    standard_includes.h
 *  \brief   Collection of standard libaries and common functions. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   June 2017
 *  \version 0.5.0
 *
 *  CSIRO Autonomous Systems Laboratory
 *  Queensland Centre for Advanced Technologies
 *  PO Box 883, Kenmore, QLD 4069, Australia
 *
 *  (c) Copyright CSIRO 2017
 *
 *  All rights reserved, no part of this program may be used
 *  without explicit permission of CSIRO
 *
***********************************************************************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/exceptions.h>

#include <dynamic_reconfigure/server.h>
#include <simple_hexapod_controller/DynamicConfig.h>

#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Eigen>

#include <boost/iterator/iterator_concepts.hpp>

#include <sstream>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#define UNASSIGNED_VALUE 1e10 // Value used to determine if variable has been assigned
#define PROGRESS_COMPLETE 100 // Vale denoting 100% and a completion of progress of various functions
#define THROTTLE_PERIOD 5  // Default throttle period for all throttled rosconsole messages (seconds)

using namespace Eigen;
using namespace std;
using namespace std_msgs;

/***********************************************************************************************************************
 * Math helper functions
***********************************************************************************************************************/

// Performs the modulo operation with adherence to Euclidean division
template <class T>
inline T mod(const T &a, const T &b) { return (a % b + b) % b; };

template <class T>
inline T sqr(const T &val) { return val * val; };

template <class T>
inline T sign(const T &val) { return (val > 0 ? 1 : -1); };

inline int roundToInt(double x) { return (x >= 0 ? int(x + 0.5) : -int(0.5 - x)); };

inline int roundToEvenInt(double x) { return (int(x) % 2 == 0 ? int(x) : int(x) + 1); };

inline double random(double min, double max) { return (min + (max - min) * (double(rand()) / double(RAND_MAX))); };

inline Vector3d absVector(const Vector3d &a) { return Vector3d(abs(a[0]), abs(a[1]), abs(a[2])); };

// Returns mean of elements in the list
template <class T>
inline T mean(const vector<T> &list)
{
  ROS_ASSERT(list.size() > 0);
  T result = list[0];
  for (unsigned int i = 1; i < list.size(); i++)
  {
    result += list[i];
  }
  result /= double(list.size());
  return result;
}

// Returns median of elements in the list (For even number of elements, returns the mean of the two medians)
template <class T>
inline T median(vector<T> list)
{
  typename vector<T>::iterator first = list.begin();
  typename vector<T>::iterator last = list.end();
  typename vector<T>::iterator middle = first + ((last - first) / 2);
  nth_element(first, middle, last);  // can specify comparator as optional 4th arg
  if (list.size() % 2) // odd
  {
    return *middle;
  }
  else
  {
    typename vector<T>::iterator middle2 = middle + 1;
    nth_element(first, middle2, last);
    return (*middle + *middle2) / 2.0;
  }
}

/***********************************************************************************************************************
 * Min/Max clamping functions
***********************************************************************************************************************/

inline Vector3d maxVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(max(a[0], b[0]), max(a[1], b[1]), max(a[2], b[2]));
}

inline Vector3d minVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]));
}

template <class T>
inline T clamped(const T &value, const T &min_value, const T &max_value)
{
  ROS_ASSERT(min_value <= max_value);
  return max(min_value, min(value, max_value));
}

template <class T>
inline T clamped(const T &value, double magnitude)
{
  double mag_sqr = value.norm();
  if (mag_sqr > magnitude)
    return value * (magnitude / mag_sqr);
  return value;
}

/***********************************************************************************************************************
 * Formating functions
***********************************************************************************************************************/

inline double setPrecision(double value, int precision)
{
  double return_value = roundToInt(value*pow(10, precision))/pow(10, precision);
  return return_value;
}

template <typename T>
inline std::string numberToString ( T number )
{
  ostringstream ss;
  ss << number;
  return ss.str();
}

template<typename ... Args>
inline string stringFormat( const std::string& format, Args ... args )
{
  size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  unique_ptr<char[]> buf( new char[ size ] ); 
  snprintf( buf.get(), size, format.c_str(), args ... );
  return string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

/***********************************************************************************************************************
 * Bezier curve functions
***********************************************************************************************************************/

// 3rd order bezier curve function
template <class T>
inline T cubicBezier(T *points, double t) 
{
  double s = 1.0 - t;
  return points[0] * (s * s * s) + points[1] * (3.0 * t * s * s) + points[2] * (3.0 * t * t * s) +
         points[3] * (t * t * t);
}

// Derivative of 3rd order bezier curve function
template <class T>
inline T cubicBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (3 * s * s * (points[1] - points[0]) + 6 * s * t * (points[2] - points[1]) +
          3 * t * t * (points[3] - points[2]));
}

// 4th order bezier curve function
template <class T>
inline T quarticBezier(T *points, double t)
{
  double s = 1.0 - t;
  return points[0] * (s * s * s * s) + points[1] * (4.0 * t * s * s * s) + points[2] * (6.0 * t * t * s * s) +
         points[3] * (4.0 * t * t * t * s) + points[4] * (t * t * t * t);
}

// Derivative of 4th order bezier curve function
template <class T>
inline T quarticBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (4.0 * s * s * s * (points[1] - points[0]) + 12.0 * s * s * t * (points[2] - points[1]) +
          12.0 * s * t * t * (points[3] - points[2]) + 4.0 * t * t * t * (points[4] - points[3]));
}

/***********************************************************************************************************************
 * DH Matrix generation
***********************************************************************************************************************/
inline Matrix4d createDHMatrix(double d, double theta, double r, double alpha)
{
  Matrix4d m;
  m << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), r*cos(theta), 
       sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
                0,             sin(alpha),             cos(alpha),            d,
                0,                      0,                      0,            1;
  return m;
}

/***********************************************************************************************************************
 * Jacobian function declarations (see jacobian.cpp for definitions)
***********************************************************************************************************************/
MatrixXd createJacobian1DOF(vector<map<string, double>> dh);
MatrixXd createJacobian2DOF(vector<map<string, double>> dh);
MatrixXd createJacobian3DOF(vector<map<string, double>> dh);
MatrixXd createJacobian4DOF(vector<map<string, double>> dh);
MatrixXd createJacobian5DOF(vector<map<string, double>> dh);
MatrixXd createJacobian6DOF(vector<map<string, double>> dh);

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H */
