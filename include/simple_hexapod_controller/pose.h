#pragma once
#include "standardIncludes.h"
#include "quat.h"

/// 3d orientation and position
struct Pose
{
  Vector3d position;
  Quat rotation;

  /**Default constructor. Does not initialize members.(For speed)*/
  Pose() {}
  /// Constructors
  Pose(const Vector3d &position, const Quat &rotation)
  {
    this->position = position;
    this->rotation = rotation;
  }
  void set(const Vector3d &position, const Quat &rotation)
  {
    this->position = position;
    this->rotation = rotation;
  }
  
  /// Operators
  Pose operator *(const Quat &quat) const
  {
    return Pose(position, rotation * quat);
  }
  
  Vector3d operator *(const Vector3d &vec) const
  {
    return position + rotation.rotateVector(vec);
  }
  
  
  Pose operator *(const Pose &pose) const
  {
    return Pose(position + rotation.rotateVector(pose.position), rotation * pose.rotation);
  }
  
  Pose operator *(double scale) const
  {
    return Pose(position*scale, rotation * scale);
  }
  Pose operator /(double scale) const
  {
    return Pose(position / scale, rotation / scale);
  }
  Pose operator +(const Pose &pose) const
  {
    return Pose(position + pose.position, rotation + pose.rotation);
  }
  Pose operator -(const Pose &pose) const
  {
    return Pose(position - pose.position, rotation - pose.rotation);
  }
 
  void operator *=(const Pose &pose) 
  {
    position += rotation.rotateVector(pose.position);
    rotation *= pose.rotation;
  }

  void operator +=(const Pose &pose) 
  {
    position += pose.position;
    rotation += pose.rotation;
  }
  void operator /=(double x) 
  {
    position /= x;
    rotation /= x;
  }
  bool operator ==(const Pose &pose)
  {
    return ((position == pose.position) && (rotation == pose.rotation));
  }
  bool operator !=(const Pose &pose)
  {
    return ((position != pose.position) || (rotation != pose.rotation));
  }
  
  Pose operator ~() const
  {
    Quat inv = ~rotation;
    return Pose( inv.rotateVector(-position), inv);
  }
  double &operator [](int index)
  {
    ASSERT(index >=0 && index < 7);
    if (index < 3)
      return position[index];
    return rotation[index - 3];
  }
  
  /// Normalise in-place
  void normalize()
  {
    rotation.normalize();
  }
  
  /// Return normalized pose
  Pose normalized() const
  {
    Pose result = *this;
    result.normalize();
    return result;
  }  
  
  Vector3d transformVector(const Vector3d &vec) const
  {
    return position + rotation.rotateVector(vec);
  }
  Vector3d inverseTransformVector(const Vector3d &vec) const
  {
    return (~*this).transformVector(vec);
  }
  
  
  static Pose identity()
  {
    return Pose(Vector3d(0,0,0), Quat(1,0,0,0));
  }
  static Pose zero()
  {
    return Pose(Vector3d(0,0,0), Quat(0,0,0,0));
  }
};

