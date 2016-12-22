#pragma once
#include "standardIncludes.h"

/// Quaternion class. (Eigen's Quaterniond is missing many useful functions)
struct Quat
{
  // real valued part cos(2*angle)
  double w;
  // rotation axis, scaled by sin(2*angle)
  double x;
  double y;
  double z;

  /// Constructors
  /**Default constructor. Does not initialize members.(For speed)*/
  Quat()
  {
  }
  Quat(double w, double x, double y, double z)
  {
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
  }

  Quat(const Vector3d &rotationVector)
  {
    double ha = 0.5 * rotationVector.norm();
    w = cos(ha);
    if (ha > 1e-20)
      setVectorPart(sin(ha) * rotationVector.normalized());
    else
      setVectorPart(Vector3d(0, 0, 0));
  }

  Quat(const Matrix3d &mat)
  {
    // This algorithm comes from  "Quat Calculus and Fast Animation",
    // Ken Shoemake, 1987 SIGGRAPH course notes
    double t = mat.trace();
    if (t > 0)
    {
      t = sqrt(t + 1.0);
      w = 0.5 * t;
      t = 0.5 / t;
      x = (mat(2, 1) - mat(1, 2)) * t;
      y = (mat(0, 2) - mat(2, 0)) * t;
      z = (mat(1, 0) - mat(0, 1)) * t;
    }
    else
    {
      int i = 0;
      if (mat(1, 1) > mat(0, 0))
        i = 1;
      if (mat(2, 2) > mat(i, i))
        i = 2;
      int j = (i + 1) % 3;
      int k = (j + 1) % 3;

      t = sqrt(double(mat(i, i) - mat(j, j) - mat(k, k) + 1.0));
      double *vec = &x;  // assume x,y,z are in order in memory
      vec[i] = 0.5 * t;
      t = 0.5 / t;
      w = (mat(k, j) - mat(j, k)) * t;
      vec[j] = (mat(j, i) + mat(i, j)) * t;
      vec[k] = (mat(k, i) + mat(i, k)) * t;
    }
    if (w < 0.0)
    {
      *this = -*this;
    }
  }

  /// Operators
  Quat operator~() const  // conjugate, can use as a fast inverse if known to be unit length
  {
    return Quat(w, -x, -y, -z);
  }
  Quat operator-() const
  {
    return Quat(-w, -x, -y, -z);
  }
  inline Quat operator+(const Quat &quat) const
  {
    return Quat(w + quat.w, x + quat.x, y + quat.y, z + quat.z);
  }
  inline Quat operator-(const Quat &quat) const
  {
    return Quat(w - quat.w, x - quat.x, y - quat.y, z - quat.z);
  }
  inline Quat operator*(double d) const
  {
    return Quat(w * d, x * d, y * d, z * d);
  }
  inline Quat operator/(double d) const
  {
    ASSERT(d != 0);
    double invD = 1 / d;
    return *this * invD;
  }
  Quat operator*(const Quat &b) const
  {
    return Quat(w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y + y * b.w + z * b.x - x * b.z, w * b.z + z * b.w + x * b.y - y * b.x);
  }
  inline void operator+=(const Quat &quat)
  {
    *this = *this + quat;
  }
  inline void operator-=(const Quat &quat)
  {
    *this = *this - quat;
  }
  void operator*=(double d)
  {
    *this = (*this) * d;
  }
  void operator/=(double d)
  {
    ASSERT(d != 0);
    *this = (*this) * (1 / d);
  }
  void operator*=(const Quat &quat)
  {
    *this = (*this) * quat;
  }
  inline double &operator[](int index) const
  {
    return *((double *)&w + index);
  }
  inline bool operator==(const Quat &b) const
  {
    return (w == b.w) && (x == b.x) && (y == b.y) && (z == b.z);
  }
  inline bool operator!=(const Quat &quat) const
  {
    return quat.x != x || quat.y != y || quat.z != z || quat.w != w;
  }

  /// Functions
  inline double dot(const Quat &quat) const
  {
    return x * quat.x + y * quat.y + z * quat.z + w * quat.w;
  }
  inline double angle() const  // a little slow
  {
    return 2.0 * atan2(vectorPart().norm(), w);
  }
  Vector3d toRotationVector() const
  {
    Vector3d axis = vectorPart();
    double mag = axis.norm();
    if (mag)
      return axis * (2 * atan2(mag, w) / mag);
    return Vector3d::Zero();
  }

  Vector3d toEulerAngles() const
  {
    double q[4] = { w, x, y, z };
    Vector3d eulerAngles;
    eulerAngles[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    eulerAngles[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
    eulerAngles[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
    return eulerAngles;
  }

  inline Matrix3d toRotationMatrix() const
  {
    Matrix3d result;
    double q[4] = { w, x, y, z };
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

  Vector3d rotateVector(const Vector3d &v) const
  {
    Matrix3d mat = toRotationMatrix();  // matches matlab... above is simpler but not perfectly the same result
    return mat * v;
  }

  Vector3d inverseRotateVector(const Vector3d &v) const
  {
    return (~*this).rotateVector(v);
  }

  double magnitudeSquared() const
  {
    return w * w + x * x + y * y + z * z;
  }
  double magnitude() const
  {
    return sqrt(magnitudeSquared());
  }
  void normalize()
  {
    double mag = magnitude();
    if (mag)
      *this *= 1 / mag;
  }
  Quat normalized() const
  {
    Quat result(w, x, y, z);
    result.normalize();
    return result;
  }
  bool isNormalized() const
  {
    return abs(magnitudeSquared() - 1.0) < 0.0002;
  }
  Vector3d vectorPart() const
  {
    return Vector3d(x, y, z);
  }
  void setVectorPart(const Vector3d &vec)
  {
    x = vec[0];
    y = vec[1];
    z = vec[2];
  }
  Quat inverse() const
  {
    return ~*this / magnitudeSquared();
  }
  static Quat Identity()
  {
    return Quat(1, 0, 0, 0);
  }
  static Quat Zero()
  {
    return Quat(0, 0, 0, 0);
  }

  Quat slerpTo(Quat targetRotation, double t)
  {
    Quaterniond a((*this).w, (*this).x, (*this).y, (*this).z);
    Quaterniond b(targetRotation.w, targetRotation.x, targetRotation.y, targetRotation.z);

    Quaterniond qres = a.slerp(t, b);

    Quat res(qres.w(), qres.x(), qres.y(), qres.z());

    return res;
  }
};

/// Double * Quat
inline Quat operator*(double d, const Quat &quat)
{
  return quat * d;
}

/// Specialisation of basic template function
inline vector<Quat> relativeQuatList(const vector<Quat> &list)
{
  vector<Quat> result;
  for (unsigned int i = 0; i < list.size() - 1; i++)
  {
    result.push_back(list[i].inverse() * list[i + 1]);
  }
  return result;
}

/// Keep first quaternion the same, and make subsequent quaternions only differ by less than 360 degrees
inline vector<Quat> unwrapQuats(const vector<Quat> &quats)
{
  vector<Quat> result = quats;
  for (unsigned int i = 1; i < quats.size(); i++)
  {
    double dp = quats[i].dot(result[i - 1]);
    if (dp < 0)
      result[i] *= -1.0;
  }
  return result;
}
