#ifndef SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
#define SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
/*******************************************************************************************************************//**
 *  @file    standard_includes.h
 *  @brief   Collection of standard libaries and common functions. Part of simple hexapod controller.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    June 2017
 *  @version 0.5.0
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
#include <tf2/transform_datatypes.h>

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

/**
 * Performs the modulo operation with adherence to Euclidean division.
 * @param[in] a The dividend for the operation.
 * @param[in] b The divisor for the operation.
 */
template <class T>
inline T mod(const T &a, const T &b) { return (a % b + b) % b; };

/**
 * Performs the square operation.
 * @param[in] val The value to be squared.
 */
template <class T>
inline T sqr(const T &val) { return val * val; };

/**
 * Returns 1 or -1 depending on the sign of the input.
 * @param[in] val The input value.
 */
template <class T>
inline T sign(const T &val) { return (val > 0 ? 1 : -1); };

/**
 * Rounds the input number to the nearest integer.
 * @param[in] x The input number.
 */
inline int roundToInt(double x) { return (x >= 0 ? int(x + 0.5) : -int(0.5 - x)); };

/**
 * Rounds the input number to the nearest EVEN integer.
 * @param[in] x The input number.
 */
inline int roundToEvenInt(double x) { return (int(x) % 2 == 0 ? int(x) : int(x) + 1); };

/**
 * Returns the input value 'clamped' within min and max limits.
 * @param[in] value The input value.
 * @param[in] min_value The minimum limit.
 * @param[in] max_value The maximum limit.
 */
template <class T>
inline T clamped(const T &value, const T &min_value, const T &max_value)
{
  ROS_ASSERT(min_value <= max_value);
  return max(min_value, min(value, max_value));
}

/**
 * Returns the input vector scaled such that the magnitude does not exceed the input magnitude.
 * @param[in] value The input vector.
 * @param[in] magnitude The maximum magnitude.
 */
template <class T>
inline T clamped(const T &value, double magnitude)
{
  return value.norm() > magnitude ? (value * (magnitude / value.norm())) : value;
}

/**
 * Returns the input value with a precision defined by the precision input. (Eg: 1.00001 @ precision = 3 -> 1.000)
 * @param[in] value The input value.
 * @param[in] precision The required precision.
 */
inline double setPrecision(double value, int precision)
{
  return roundToInt(value*pow(10, precision))/pow(10, precision);
}

/**
 * Returns a string representation of the input value.
 * @param[in] number The input value.
 */
template <typename T>
inline string numberToString ( T number )
{
  ostringstream ss;
  ss << number;
  return ss.str();
}

/**
 * Returns a string formatted using the same input arguments as rosconsole.
 * @param[in] format The input string.
 * @param[in] args The list of arguments to populate the format string.
 */
template<typename ... Args>
inline string stringFormat( const string& format, Args ... args )
{
  size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  unique_ptr<char[]> buf( new char[ size ] ); 
  snprintf( buf.get(), size, format.c_str(), args ... );
  return string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

/**
 * Returns a vector representing a 3d point at a given time input along a 3rd order bezier curve defined by input 
 * control nodes.
 * @param[in] points An array of control node vectors.
 * @param[in] t A time input from 0.0 to 1.0.
 */
template <class T>
inline T cubicBezier(T *points, double t) 
{
  double s = 1.0 - t;
  return points[0] * (s * s * s) + points[1] * (3.0 * t * s * s) + points[2] * (3.0 * t * t * s) +
         points[3] * (t * t * t);
}

/**
 * Returns a vector representing a 3d point at a given time input along the derivative of a 3rd order bezier curve 
 * defined by input control nodes.
 * @param[in] points An array of control node vectors.
 * @param[in] t A time input from 0.0 to 1.0.
 */
template <class T>
inline T cubicBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (3 * s * s * (points[1] - points[0]) + 6 * s * t * (points[2] - points[1]) +
          3 * t * t * (points[3] - points[2]));
}

/**
 * Returns a vector representing a 3d point at a given time input along a 4th order bezier curve defined by input 
 * control nodes.
 * @param[in] points An array of control node vectors.
 * @param[in] t A time input from 0.0 to 1.0.
 */
template <class T>
inline T quarticBezier(T *points, double t)
{
  double s = 1.0 - t;
  return points[0] * (s * s * s * s) + points[1] * (4.0 * t * s * s * s) + points[2] * (6.0 * t * t * s * s) +
         points[3] * (4.0 * t * t * t * s) + points[4] * (t * t * t * t);
}

/**
 * Returns a vector representing a 3d point at a given time input along the derivative of a 4th order bezier curve 
 * defined by input control nodes.
 * @param[in] points An array of control node vectors.
 * @param[in] t A time input from 0.0 to 1.0.
 */
template <class T>
inline T quarticBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (4.0 * s * s * s * (points[1] - points[0]) + 12.0 * s * s * t * (points[2] - points[1]) +
          12.0 * s * t * t * (points[3] - points[2]) + 4.0 * t * t * t * (points[4] - points[3]));
}

/**
 * Generates a classical Denavit–Hartenberg (DH) matrix from DH parameters.
 * @param[in] d The DH parameter representing offset along previous z-axis to the common normal.
 * @param[in] theta The DH parameter representing angle about previous z axis, from old x-axis to new x-axis.
 * @param[in] r The DH parameter representing length of the common normal.
 * @param[in] alpha The DH parameter representing angle about common normal, form old z-axis to new z-axis.
 */
inline Matrix4d createDHMatrix(double d, double theta, double r, double alpha)
{
  Matrix4d m;
  m << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), r*cos(theta), 
       sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
                0,             sin(alpha),             cos(alpha),            d,
                0,                      0,                      0,            1;
  return m;
}

/**
 * Generates a Jacobian for a single degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 */
MatrixXd createJacobian1DOF(vector<map<string, double>> dh);

/**
 * Generates a Jacobian for 2 degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 */
MatrixXd createJacobian2DOF(vector<map<string, double>> dh);

/**
 * Generates a Jacobian for 3 degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 */
MatrixXd createJacobian3DOF(vector<map<string, double>> dh);

/**
 * Generates a Jacobian for 4 degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 */
MatrixXd createJacobian4DOF(vector<map<string, double>> dh);

/**
 * Generates a Jacobian for 5 degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 */
MatrixXd createJacobian5DOF(vector<map<string, double>> dh);

/**
 * Generates a Jacobian for 6 degree of freedom system from DH parameters.
 * @param[in] dh A vector of DH parameters for each joint in the system.
 * @todo Create definition for this function.
 */
MatrixXd createJacobian6DOF(vector<map<string, double>> dh);

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H */
