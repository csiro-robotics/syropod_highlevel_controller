#ifndef SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
#define SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H
/** 
 *  \file    standard_includes.h
 *  \brief   Collection of standard libaries and common functions. Part of simple hexapod controller.
 *
 *  \author Fletcher Talbot
 *  \date   January 2017
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
 */

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

#define UNASSIGNED_VALUE 1e10
#define PROGRESS_COMPLETE 100
const double pi = M_PI;  //< easier to read

// ifdef because assert(x) is nop on NDEBUG defined, not on 'DEBUG not defined'
#ifndef DEBUG
#define ASSERT(x)                                                                                                      
  {                                                                                                                    
  }
#define ATTEMPT(x_, y_) x_
#else
#define ASSERT(x) assert(x)
#define ATTEMPT(x_, y_) assert(x_ y_)
#endif

using namespace Eigen;
using namespace std;
using namespace std_msgs;

template <class T>
T mod(const T &a, const T &b)
{
  return (a % b + b) % b;
}

template <class T>
T sqr(const T &val)
{
  return val * val;
}

template <class T>
T sign(const T &val)
{
  return val > 0 ? 1 : -1;
}

inline double solveQuadratic(double a, double b, double c, double *negative = NULL)
{
  double inside = sqr(b) - 4.0 * a * c;
  ASSERT(inside >= 0.0);
  if (negative != NULL)
  {
    double numerator = -b - sqrt(inside);
    if (2.0 * a < numerator)
      *negative = 2.0 * c / numerator;
    else
      *negative = numerator / (2.0 * a);
  }
  double numerator = -b + sqrt(inside);
  if (2.0 * a < numerator)
    return 2.0 * c / numerator;
  return numerator / (2.0 * a);
}

template <class T>
inline T cubicBezier(T *points, double t)
{
  double s = 1.0 - t;
  return points[0] * (s * s * s) + points[1] * (3.0 * t * s * s) + points[2] * (3.0 * t * t * s) +
         points[3] * (t * t * t);
}

template <class T>
inline T cubicBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (3 * s * s * (points[1] - points[0]) + 6 * s * t * (points[2] - points[1]) +
          3 * t * t * (points[3] - points[2]));
}

template <class T>
inline T quarticBezier(T *points, double t)
{
  double s = 1.0 - t;
  return points[0] * (s * s * s * s) + points[1] * (4.0 * t * s * s * s) + points[2] * (6.0 * t * t * s * s) +
         points[3] * (4.0 * t * t * t * s) + points[4] * (t * t * t * t);
}

template <class T>
inline T quarticBezierDot(T *points, double t)
{
  double s = 1.0 - t;
  return (4.0 * s * s * s * (points[1] - points[0]) + 12.0 * s * s * t * (points[2] - points[1]) +
          12.0 * s * t * t * (points[3] - points[2]) + 4.0 * t * t * t * (points[4] - points[3]));
}

inline Matrix4d createDHMatrix(double d, double theta, double r, double alpha)
{
  Matrix4d m;
  m << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), r*cos(theta), 
       sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
                0,             sin(alpha),             cos(alpha),            d,
                0,                      0,                      0,            1;
  return m;
}

inline MatrixXd createJacobian(vector<map<string, double>> dh, int degrees_of_freedom)
{
  switch(degrees_of_freedom)
  {
    case(1):
    {
      //double d1 = dh[0]["d"]; //Unused
      double r1 = dh[0]["r"];
      double sT1 = sin(dh[0]["theta"]);
      double cT1 = cos(dh[0]["theta"]);
      //double sA1 = sin(dh[0]["alpha"]); //Unused
      //double cA1 = cos(dh[0]["alpha"]); //Unused
      MatrixXd j(3, 1);
      j(0,0) = -r1*sT1;
      j(1,0) = r1*cT1;
      j(2,0) = 0.0;
      return j;
    }
    case(2):
    {
      //double d1 = dh[0]["d"]; //Unused
			double d2 = dh[1]["d"];
      double r1 = dh[0]["r"];
			double r2 = dh[1]["r"];
      double sT1 = sin(dh[0]["theta"]);
			double sT2 = sin(dh[1]["theta"]);
      double cT1 = cos(dh[0]["theta"]);
			double cT2 = cos(dh[1]["theta"]);
      double sA1 = sin(dh[0]["alpha"]);
			//double sA2 = sin(dh[1]["alpha"]); //Unused
      double cA1 = cos(dh[0]["alpha"]);
			//double cA2 = cos(dh[1]["alpha"]); //Unused
      MatrixXd j(3, 2);
      j(0,0) = -(sT1*r2*cT2)-(cT1*cA1*r2*sT2)+(cT1*sA1*d2)-(r1*sT1);
      j(0,1) = -(cT1*r2*sT2)-(sT1*cA1*r2*cT2);
      j(1,0) = (cT1*r2*cT2)-(sT1*cA1*r2*sT2)+(sT1*sA1*d2)+(r1*cT1);
      j(1,1) = -(sT1*r2*sT2)+(cT1*cA1*r2*cT2);
      j(2,0) = 0.0;
      j(2,1) = (sA1*r2*cT2);
      return j;
    }
    case(3):
    {
      //double d1 = dh[0]["d"]; //Unused
			double d2 = dh[1]["d"];
			double d3 = dh[2]["d"];
      double r1 = dh[0]["r"];
			double r2 = dh[1]["r"];
			double r3 = dh[2]["r"];
      double sT1 = sin(dh[0]["theta"]);
			double sT2 = sin(dh[1]["theta"]);
			double sT3 = sin(dh[2]["theta"]);
      double cT1 = cos(dh[0]["theta"]);
			double cT2 = cos(dh[1]["theta"]);
			double cT3 = cos(dh[2]["theta"]);
      double sA1 = sin(dh[0]["alpha"]);
			double sA2 = sin(dh[1]["alpha"]);
			//double sA3 = sin(dh[2]["alpha"]); //Unused
      double cA1 = cos(dh[0]["alpha"]);
			double cA2 = cos(dh[1]["alpha"]);
			//double cA3 = cos(dh[2]["alpha"]); //Unused
      MatrixXd j(3,3);
      j(0,0) = -(sT1*cT2*r3*cT3)-(cT1*cA1*sT2*r3*cT3)+(sT1*sT2*cA2*r3*sT3)-(cT1*cA1*cT2*cA2*r3*sT3)+(cT1*sA1*sA2*r3*sT3)-(sT1*sT2*sA2*d3)+(cT1*cA1*cT2*sA2*d3)+(cT1*sA1*cA2*d3)-(sT1*r2*cT2)-(cT1*cA1*r2*sT2)+(cT1*sA1*d2)-(r1*sT1);
      j(0,1) = -(cT1*sT2*r3*cT3)-(sT1*cA1*cT2*r3*cT3)-(cT1*cT2*cA2*r3*sT3)+(sT1*cA1*sT2*cA2*r3*sT3)+(cT1*cT2*sA2*d3)-(sT1*cA1*sT2*sA2*d3)-(cT1*r2*sT2)-(sT1*cA1*r2*cT2);
      j(0,2) = -(cT1*cT2*r3*sT3)+(sT1*cA1*sT2*r3*sT3)-(cT1*sT2*cA2*r3*cT3)-(sT1*cA1*cT2*cA2*r3*cT3)+(sT1*sA1*sA2*r3*cT3);
      j(1,0) = (cT1*cT2*r3*cT3)-(sT1*cA1*sT2*r3*cT3)-(cT1*sT2*cA2*r3*sT3)-(sT1*cA1*cT2*cA2*r3*sT3)+(sT1*sA1*sA2*r3*sT3)+(cT1*sT2*sA2*d3)+(sT1*cA1*cT2*sA2*d3)+(sT1*sA1*cA2*d3)+(cT1*r2*cT2)-(sT1*cA1*r2*sT2)+(sT1*sA1*d2)+(r1*cT1);
      j(1,1) = -(sT1*sT2*r3*cT3)+(cT1*cA1*cT2*r3*cT3)-(sT1*cT2*cA2*r3*sT3)-(cT1*cA1*sT2*cA2*r3*sT3)+(sT1*cT2*sA2*d3)+(cT1*cA1*sT2*sA2*d3)-(sT1*r2*sT2)+(cT1*cA1*r2*cT2);
      j(1,2) = -(sT1*cT2*r3*sT3)-(cT1*cA1*sT2*r3*sT3)-(sT1*sT2*cA2*r3*cT3)+(cT1*cA1*cT2*cA2*r3*cT3)-(cT1*sA1*sA2*r3*cT3);
      j(2,0) = 0;
      j(2,1) = (sA1*cT2*r3*cT3)-(sA1*sT2*cA2*r3*sT3)+(sA1*sT2*sA2*d3)+(sA1*r2*cT2);
      j(2,2) = -(sA1*sT2*r3*sT3)+(sA1*cT2*cA2*r3*cT3)+(cA1*sA2*r3*cT3);
      return j; 
    }
    case(4):
    {
      //double d1 = dh[0]["d"]; //Unused
			double d2 = dh[1]["d"];
			double d3 = dh[2]["d"];
      double d4 = dh[3]["d"];
      double r1 = dh[0]["r"];
			double r2 = dh[1]["r"];
			double r3 = dh[2]["r"];
      double r4 = dh[3]["r"];
      double sT1 = sin(dh[0]["theta"]);
			double sT2 = sin(dh[1]["theta"]);
			double sT3 = sin(dh[2]["theta"]);
      double sT4 = sin(dh[3]["theta"]);
      double cT1 = cos(dh[0]["theta"]);
			double cT2 = cos(dh[1]["theta"]);
			double cT3 = cos(dh[2]["theta"]);
      double cT4 = cos(dh[3]["theta"]);
      double sA1 = sin(dh[0]["alpha"]);
			double sA2 = sin(dh[1]["alpha"]);
			double sA3 = sin(dh[2]["alpha"]);
      //double sA4 = sin(dh[3]["alpha"]); //Unused
      double cA1 = cos(dh[0]["alpha"]);
			double cA2 = cos(dh[1]["alpha"]);
			double cA3 = cos(dh[2]["alpha"]);
      //double cA4 = cos(dh[3]["alpha"]); //Unused
      MatrixXd j(3,4);
      j(0,0) = -(sT1*cT2*cT3*r4*cT4)-(cT1*cA1*sT2*cT3*r4*cT4)+(sT1*sT2*cA2*sT3*r4*cT4)-(cT1*cA1*cT2*cA2*sT3*r4*cT4)-(cT1*sA1*sA2*sT3*r4*cT4)+(sT1*cT2*sT3*cA3*r4*sT4)+(cT1*cA1*sT2*sT3*cA3*r4*sT4)+(sT1*sT2*cA2*cT3*cA3*r4*sT4)-(cT1*cA1*cT2*cA2*cT3*cA3*r4*sT4)+(cT1*sA1*sA2*cT3*cA3*r4*sT4)-(sT1*sT2*sA2*sA3*r4*sT4)+(cT1*cA1*cT2*sA2*sA3*r4*sT4)+(cT1*sA1*cA2*sA3*r4*sT4)-(sT1*cT2*sT3*sA3*d4)-(cT1*cA1*sT2*sT3*sA3*d4)-(sT1*sT2*cA2*cT3*sA3*d4)+(cT1*cA1*cT2*cA2*cT3*sA3*d4)-(cT1*sA1*sA2*cT3*sA3*d4)-(sT1*sT2*sA2*cA3*d4)+(cT1*cA1*cT2*sA2*cA3*d4)-(sT1*cT2*r3*cT3)+(cT1*sA1*cA2*cA3*d4)-(cT1*cA1*sT2*r3*cT3)+(sT1*sT2*cA2*r3*sT3)-(cT1*cA1*cT2*cA2*r3*sT3)+(cT1*sA1*sA2*r3*sT3)-(sT1*sT2*sA2*d3)+(cT1*cA1*cT2*sA2*d3)+(cT1*sA1*cA2*d3)-(sT1*r2*cT2)-(cT1*cA1*r2*sT2)+(cT1*sA1*d2)-(r1*sT1);
      j(0,1) = -(cT1*sT2*cT3*r4*cT4)-(sT1*cA1*cT2*cT3*r4*cT4)-(cT1*cT2*cA2*sT3*r4*cT4)+(sT1*cA1*sT2*cA2*sT3*r4*cT4)+(cT1*sT2*sT3*cA3*r4*sT4)+(sT1*cA1*cT2*sT3*cA3*r4*sT4)-(cT1*cT2*cA2*cT3*cA3*r4*sT4)+(sT1*cA1*sT2*cA2*cT3*cA3*r4*sT4)+(cT1*cT2*sA2*sA3*r4*sT4)-(sT1*cA1*sT2*sA2*sA3*r4*sT4)-(cT1*sT2*sT3*sA3*d4)-(sT1*cA1*cT2*sT3*sA3*d4)+(cT1*cT2*cA2*cT3*sA3*d4)-(sT1*cA1*sT2*cA2*cT3*sA3*d4)+(cT1*cT2*sA2*cA3*d4)-(sT1*cA1*sT2*sA2*cA3*d4)-(cT1*sT2*r3*cT3)-(sT1*cA1*cT2*r3*cT3)-(cT1*cT2*cA2*r3*sT3)+(sT1*cA1*sT2*cA2*r3*sT3)+(cT1*cT2*sA2*d3)-(sT1*cA1*sT2*sA2*d3)-(cT1*r2*sT2)-(sT1*cA1*r2*cT2);
      j(0,2) = -(cT1*cT2*sT3*r4*cT4)+(sT1*cA1*sT2*sT3*r4*cT4)-(cT1*sT2*cA2*cT3*r4*cT4)-(sT1*cA1*cT2*cA2*cT3*r4*cT4)+(sT1*sA1*sA2*cT3*r4*cT4)-(cT1*cT2*cT3*cA3*r4*sT4)+(sT1*cA1*sT2*cT3*cA3*r4*sT4)+(cT1*sT2*cA2*sT3*cA3*r4*sT4)+(sT1*cA1*cT2*cA2*sT3*cA3*r4*sT4)-(sT1*sA1*sA2*sT3*cA3*r4*sT4)+(cT1*cT2*cT3*sA3*d4)-(sT1*cA1*sT2*cT3*sA3*d4)-(cT1*sT2*cA2*sT3*sA3*d4)-(sT1*cA1*cT2*cA2*sT3*sA3*d4)+(sT1*sA1*sA2*sT3*sA3*d4)-(cT1*cT2*r3*sT3)+(sT1*cA1*sT2*r3*sT3)-(cT1*sT2*cA2*r3*cT3)-(sT1*cA1*cT2*cA2*r3*cT3)+(sT1*sA1*sA2*r3*cT3);
      j(0,3) = -(cT1*cT2*cT3*r4*sT4)+(sT1*cA1*sT2*cT3*r4*sT4)+(cT1*sT2*cA2*sT3*r4*sT4)+(sT1*cA1*cT2*cA2*sT3*r4*sT4)-(sT1*sA1*sA2*sT3*r4*sT4)-(cT1*cT2*sT3*cA3*r4*cT4)+(sT1*cA1*sT2*sT3*cA3*r4*cT4)-(cT1*sT2*cA2*cT3*cA3*r4*cT4)-(sT1*cA1*cT2*cA2*cT3*cA3*r4*cT4)+(sT1*sA1*sA2*cT3*cA3*r4*cT4)+(cT1*sT2*sA2*sA3*r4*cT4)+(sT1*cA1*cT2*sA2*sA3*r4*cT4)+(sT1*sA1*cA2*sA3*r4*cT4);
      j(1,0) = (cT1*cT2*cT3*r4*cT4)-(sT1*cA1*sT2*cT3*r4*cT4)-(cT1*sT2*cA2*sT3*r4*cT4)-(sT1*cA1*cT2*cA2*sT3*r4*cT4)+(sT1*sA1*sA2*sT3*r4*cT4)-(cT1*cT2*sT3*cA3*r4*sT4)+(sT1*cA1*sT2*sT3*cA3*r4*sT4)-(cT1*sT2*cA2*cT3*cA3*r4*sT4)-(sT1*cA1*cT2*cA2*cT3*cA3*r4*sT4)+(sT1*sA1*sA2*cT3*cA3*r4*sT4)+(cT1*sT2*sA2*sA3*r4*sT4)+(sT1*cA1*cT2*sA2*sA3*r4*sT4)+(sT1*sA1*cA2*sA3*r4*sT4)+(cT1*cT2*sT3*sA3*d4)-(sT1*cA1*sT2*sT3*sA3*d4)+(cT1*sT2*cA2*cT3*sA3*d4)+(sT1*cA1*cT2*cA2*cT3*sA3*d4)-(sT1*sA1*sA2*cT3*sA3*d4)+(cT1*sT2*sA2*cA3*d4)+(sT1*cA1*cT2*sA2*cA3*d4)+(sT1*sA1*cA2*cA3*d4)+(cT1*cT2*r3*cT3)-(sT1*cA1*sT2*r3*cT3)-(cT1*sT2*cA2*r3*sT3)-(sT1*cA1*cT2*cA2*r3*sT3)+(sT1*sA1*sA2*r3*sT3)+(cT1*sT2*sA2*d3)+(sT1*cA1*cT2*sA2*d3)+(sT1*sA1*cA2*d3)+(cT1*r2*cT2)-(sT1*cA1*r2*sT2)+(sT1*sA1*d2)+(r1*cT1);
      j(1,1) = -(sT1*sT2*cT3*r4*cT4)+(cT1*cA1*cT2*cT3*r4*cT4)-(sT1*cT2*cA2*sT3*r4*cT4)-(cT1*cA1*sT2*cA2*sT3*r4*cT4)+(sT1*sT2*sT3*cA3*r4*sT4)-(cT1*cA1*cT2*sT3*cA3*r4*sT4)-(sT1*cT2*cA2*cT3*cA3*r4*sT4)-(cT1*cA1*sT2*cA2*cT3*cA3*r4*sT4)+(sT1*cT2*sA2*sA3*r4*sT4)+(cT1*cA1*sT2*sA2*sA3*r4*sT4)-(sT1*sT2*sT3*sA3*d4)+(sT1*cT2*cA2*cT3*sA3*d4)+(cT1*cA1*sT2*cA2*cT3*sA3*d4)+(sT1*cT2*sA2*cA3*d4)+(cT1*cA1*sT2*sA2*cA3*d4)-(sT1*sT2*r3*cT3)+(cT1*cA1*cT2*r3*cT3)-(sT1*cT2*cA2*r3*sT3)-(cT1*cA1*sT2*cA2*r3*sT3)+(sT1*cT2*sA2*d3)+(cT1*cA1*sT2*sA2*d3)-(sT1*r2*sT2)+(cT1*cA1*r2*cT2);
      j(1,2) = -(sT1*cT2*sT3*r4*cT4)-(cT1*cA1*sT2*sT3*r4*cT4)-(sT1*sT2*cA2*cT3*r4*cT4)+(cT1*cA1*cT2*cA2*cT3*r4*cT4)-(cT1*sA1*sA2*cT3*r4*cT4)-(sT1*cT2*cT3*cA3*r4*sT4)-(cT1*cA1*sT2*cT3*cA3*r4*sT4)+(sT1*sT2*cA2*sT3*cA3*r4*sT4)-(cT1*cA1*cT2*cA2*sT3*cA3*r4*sT4)+(cT1*sA1*sA2*sT3*cA3*r4*sT4)+(sT1*cT2*cT3*sA3*d4)+(cT1*cA1*sT2*cT3*sA3*d4)-(sT1*sT2*cA2*sT3*sA3*d4)+(cT1*cA1*cT2*cA2*sT3*sA3*d4)-(cT1*sA1*sA2*sT3*sA3*d4)-(sT1*cT2*r3*sT3)-(cT1*cA1*sT2*r3*sT3)-(sT1*sT2*cA2*r3*cT3)+(cT1*cA1*cT2*cA2*r3*cT3)-(cT1*sA1*sA2*r3*cT3);
      j(1,3) = -(sT1*cT2*cT3*r4*sT4)-(cT1*cA1*sT2*cT3*r4*sT4)+(sT1*sT2*cA2*sT3*r4*sT4)-(cT1*cA1*cT2*cA2*sT3*r4*sT4)+(cT1*sA1*sA2*sT3*r4*sT4)-(sT1*cT2*sT3*cA3*r4*cT4)-(cT1*cA1*sT2*sT3*cA3*r4*cT4)-(sT1*sT2*cA2*cT3*cA3*r4*cT4)+(cT1*cA1*cT2*cA2*cT3*cA3*r4*cT4)-(cT1*sA1*sA2*cT3*cA3*r4*cT4)+(sT1*sT2*sA2*sA3*r4*cT4)-(cT1*cA1*cT2*sA2*sA3*r4*cT4)-(cT1*sA1*cA2*sA3*r4*cT4);
      j(2,0) = 0;
      j(2,1) = (sA1*cT2*cT3*r4*cT4)-(sA1*sT2*cA2*sT3*r4*cT4)-(sA1*sT2*cA2*cT3*cA3*r4*sT4)+(sA1*sT2*sA2*sA3*r4*sT4)+(sA1*cT2*sT3*sA3*d4)+(sA1*sT2*cA2*cT3*sA3*d4)+(sA1*sT2*sA2*cA3*d4)+(sA1*cT2*r3*cT3)-(sA1*sT2*cA2*r3*sT3)+(sA1*sT2*sA2*d3)+(sA1*r2*cT2);
      j(2,2) = -(sA1*sT2*sT3*r4*cT4)+(sA1*cT2*cA2*cT3*r4*cT4)+(cA1*sA2*cT3*r4*cT4)-(sA1*sT2*cT3*cA3*r4*sT4)-(sA1*cT2*cA2*sT3*cA3*r4*sT4)-(cA1*sA2*sT3*cA3*r4*sT4)+(sA1*sT2*cT3*sA3*d4)+(sA1*cT2*cA2*sT3*sA3*d4)+(cA1*sA2*sT3*sA3*d4)-(sA1*sT2*r3*sT3)+(sA1*cT2*cA2*r3*cT3)+(cA1*sA2*r3*cT3);
      j(2,3) = -(sA1*sT2*cT3*r4*sT4)-(sA1*cT2*cA2*sT3*r4*sT4)-(cA1*sA2*sT3*r4*sT4)-(sA1*sT2*sT3*cA3*r4*cT4)+(sA1*cT2*cA2*cT3*cA3*r4*cT4)+(cA1*sA2*cT3*cA3*r4*cT4)-(sA1*cT2*sA2*sA3*r4*cT4)+(cA1*cA2*sA3*r4*cT4);
      return j;
    }
    default:
      return MatrixXd::Identity(3,3);
  };
}

inline Vector3d maxVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(max(a[0], b[0]), max(a[1], b[1]), max(a[2], b[2]));
}

inline Vector3d minVector(const Vector3d &a, const Vector3d &b)
{
  return Vector3d(min(a[0], b[0]), min(a[1], b[1]), min(a[2], b[2]));
}

inline Vector3d absVector(const Vector3d &a)
{
  return Vector3d(abs(a[0]), abs(a[1]), abs(a[2]));
}

inline double minMax(const double val, const double minimum, const double maximum)
{
  return (val > (minimum + maximum) / 2) ? min(maximum, val) : max(minimum, val);
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
  double magSqr = value.norm();
  if (magSqr > magnitude)
    return value * (magnitude / magSqr);
  return value;
}

inline int roundToInt(double x)
{
  if (x >= 0)
	{
    return int(x + 0.5);
	}
  return -int(0.5 - x);
}

inline int roundToEvenInt(double x)
{
	int x_int = int(x);
  return (x_int % 2 == 0) ? x_int : x_int + 1; 
}

inline double setPrecision(double value, int precision)
{
  double return_value = roundToInt(value*pow(10, precision))/pow(10, precision);
  return return_value;
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
  result.reserve((((end + 1) - start) / step) + 1);
  for (int i = start; i <= end; i += step)
    result.push_back(i);
  return result;
}

/// Increasing list of doubles from start to end inclusive, increasing by step
inline vector<double> doubles(double start, double end, double step = 1.0)
{
  ASSERT(end >= start && step > 0);
  vector<double> result;

  // The complication here is to make sure that you get the right result by passing in double(start, end,
  // (end-start)/integerCount)

  unsigned int count = (unsigned int)(0.5 + (end - start) / step);  // round
  if ((end - start) / double(count) != step)  // for non-simple cases, just use the normal count calculation
    count = (unsigned int)((end - start) / step);
  result.reserve(count + 1);
  for (unsigned int i = 0; i <= count; ++i)
    result.push_back(start + double(i) * step);  // I avoid incrementing by step due to drift for very small step
  return result;
}

/// Return mean of elements in the list
template <class T>
T mean(const vector<T> &list)
{
  ASSERT(list.size() > 0);
  T result = list[0];
  for (unsigned int i = 1; i < list.size(); i++)
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
  nth_element(first, middle, last);  // can specify comparator as optional 4th arg
  if (list.size() % 2)               // odd
    return *middle;
  else
  {
    typename vector<T>::iterator middle2 = middle + 1;
    nth_element(first, middle2, last);
    return (*middle + *middle2) / 2.0;
  }
}

template <typename T>
std::string numberToString ( T number )
{
  ostringstream ss;
  ss << number;
  return ss.str();
}

#endif /* SIMPLE_HEXAPOD_CONTROLLER_STANDARD_INCLUDES_H */
