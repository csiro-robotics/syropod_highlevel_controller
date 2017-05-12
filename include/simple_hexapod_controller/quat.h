#ifndef SIMPLE_HEXAPOD_CONTROLLER_QUAT_H
#define SIMPLE_HEXAPOD_CONTROLLER_QUAT_H
/*******************************************************************************************************************//**
 *  \file    quat.h
 *  \brief   Custom class definition of Quaternion data type. Part of simple hexapod controller.
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

#include "quat.h"

/*******************************************************************************************************************//**
 * Quaternion class. (Eigen's Quaterniond is missing many useful functions)
***********************************************************************************************************************/
class Quat
{
public:
  inline Quat() {}
  inline Quat(double w, double x, double y, double z)
  {
    w_ = w;
    x_ = x;
    y_ = y;
    z_ = z;
  }
  inline Quat(const Vector3d &rotationVector)
  {
    double ha = 0.5 * rotationVector.norm();
    w_ = cos(ha);
    Vector3d vector_part = (ha > 1e-20 ? sin(ha) * rotationVector.normalized() : Vector3d(0, 0, 0));
    setVectorPart(vector_part);
  }

  /// Operators
  inline Quat operator~() const { return Quat(w_, -x_, -y_, -z_); } // conjugate
  inline Quat operator-() const { return Quat(-w_, -x_, -y_, -z_); }
  inline Quat operator+(const Quat &quat) const { return Quat(w_ + quat.w_, x_ + quat.x_, y_ + quat.y_, z_ + quat.z_); }
  inline Quat operator-(const Quat &quat) const { return Quat(w_ - quat.w_, x_ - quat.x_, y_ - quat.y_, z_ - quat.z_); }
  inline Quat operator*(double d) const { return Quat(w_ * d, x_ * d, y_ * d, z_ * d); }
  inline void operator+=(const Quat &quat) { *this = *this + quat; }
  inline void operator-=(const Quat &quat) { *this = *this - quat; }
  inline void operator*=(double d) { *this = (*this) * d; }
  inline void operator/=(double d) { *this = (*this) * (1 / d); }
  inline void operator*=(const Quat &quat) { *this = (*this) * quat; }
  inline double &operator[](int index) const { return *((double *)&w_ + index); }
  inline bool operator==(const Quat &b) const { return (w_ == b.w_) && (x_ == b.x_) && (y_ == b.y_) && (z_ == b.z_); }
  inline bool operator!=(const Quat &quat) const { return !((*this) == quat); }
  inline Quat operator/(double d) const { return *this * (1 / d); }
  inline Quat operator*(const Quat &b) const
  {
    return Quat(w_ * b.w_ - x_ * b.x_ - y_ * b.y_ - z_ * b.z_, w_ * b.x_ + x_ * b.w_ + y_ * b.z_ - z_ * b.y_,
                w_ * b.y_ + y_ * b.w_ + z_ * b.x_ - x_ * b.z_, w_ * b.z_ + z_ * b.w_ + x_ * b.y_ - y_ * b.x_);
  }

  /// Functions
  inline double dot(const Quat &quat) const { return x_ * quat.x_ + y_ * quat.y_ + z_ * quat.z_ + w_ * quat.w_; }
  inline double angle() const { return 2.0 * atan2(vectorPart().norm(), w_); }
  //inline Vector3d toRotationVector() const { return (vectorPart().norm() ? vectorPart().norm() : Vector3d::Zero()); };
  inline Vector3d toEulerAngles() const
  {
    double q[4] = { w_, x_, y_, z_ };
    Vector3d eulerAngles;
    eulerAngles[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    eulerAngles[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
    eulerAngles[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    return eulerAngles;
  }
  inline Matrix3d toRotationMatrix() const
  {
    Matrix3d result;
    double q[4] = { w_, x_, y_, z_ };
    result(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    result(0, 1) = 2.0 * (q[1] * q[2] - q[0] * q[3]);
    result(0, 2) = 2.0 * (q[1] * q[3] + q[0] * q[2]);
    result(1, 0) = 2.0 * (q[2] * q[1] + q[0] * q[3]);
    result(1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    result(1, 2) = 2.0 * (q[2] * q[3] - q[0] * q[1]);
    result(2, 0) = 2.0 * (q[3] * q[1] - q[0] * q[2]);
    result(2, 1) = 2.0 * (q[3] * q[2] + q[0] * q[1]);
    result(2, 2) = (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    return result;
  }
  inline Vector3d rotateVector(const Vector3d &v) const { return (toRotationMatrix() * v); }
  inline Vector3d inverseRotateVector(const Vector3d &v) const { return (~*this).rotateVector(v); }
  inline double magnitudeSquared() const { return w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_; }
  inline double magnitude() const { return sqrt(magnitudeSquared()); }
  inline void normalize() { if (magnitude()) *this *= 1/magnitude(); }
  inline bool isNormalized() const { return abs(magnitudeSquared() - 1.0) < 0.0002; }
  inline Quat normalized() const 
  { 
    Quat ret(w_, x_, y_, z_);
    ret.normalize();
    return ret;
  }
  
  inline Vector3d vectorPart() const { return Vector3d(x_, y_, z_); }
  inline void setVectorPart(const Vector3d &vec)
  {
    x_ = vec[0];
    y_ = vec[1];
    z_ = vec[2];
  }
  inline Quat inverse() const { return ~*this / magnitudeSquared(); }
  inline static Quat Identity(void) { return Quat(1, 0, 0, 0); }
  inline static Quat Zero(void) { return Quat(0, 0, 0, 0); }

  Quat slerpTo(Quat targetRotation, double t)
  {
    Quaterniond a((*this).w_, (*this).x_, (*this).y_, (*this).z_);
    Quaterniond b(targetRotation.w_, targetRotation.x_, targetRotation.y_, targetRotation.z_);
    Quaterniond qres = a.slerp(t, b);
    Quat res(qres.w(), qres.x(), qres.y(), qres.z());
    return res;
  }
  
  double w_; // real valued part cos(2*angle)
  double x_; // rotation axis, scaled by_ sin(2*angle)
  double y_;
  double z_;
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SIMPLE_HEXAPOD_CONTROLLER_QUAT_H */
