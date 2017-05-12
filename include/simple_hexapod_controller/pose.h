#ifndef SIMPLE_HEXAPOD_CONTROLLER_POSE_H
#define SIMPLE_HEXAPOD_CONTROLLER_POSE_H
/** 
 *  \file    pose.h
 *  \brief   Custom class definition of hexapod pose. Part of simple hexapod controller.
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
#include "standard_includes.h"
#include "quat.h"

/// 3d orientation and position
class Pose
{
  public:
    Vector3d position_;
    Quat rotation_;

    /**Default constructor. Does not initialize members.(For speed)*/
    Pose()
    {
    }
    /// Constructors
    Pose(const Vector3d &position, const Quat &rotation)
    {
      this->position_ = position;
      this->rotation_ = rotation;
    }
    void set(const Vector3d &position, const Quat &rotation)
    {
      this->position_ = position;
      this->rotation_ = rotation;
    }

    /// Operators
    Pose operator*(const Quat &quat) const
    {
      return Pose(position_, rotation_ * quat);
    }

    Vector3d operator*(const Vector3d &vec) const
    {
      return position_ + rotation_.rotateVector(vec);
    }

    Pose operator*(const Pose &pose) const
    {
      return Pose(position_ + rotation_.rotateVector(pose.position_), rotation_ * pose.rotation_);
    }

    Pose operator*(double scale) const
    {
      return Pose(position_ * scale, rotation_ * scale);
    }
    Pose operator/(double scale) const
    {
      return Pose(position_ / scale, rotation_ / scale);
    }
    Pose operator+(const Pose &pose) const
    {
      return Pose(position_ + pose.position_, rotation_ + pose.rotation_);
    }
    Pose operator-(const Pose &pose) const
    {
      return Pose(position_ - pose.position_, rotation_ - pose.rotation_);
    }

    void operator*=(const Pose &pose)
    {
      position_ += rotation_.rotateVector(pose.position_);
      rotation_ *= pose.rotation_;
    }

    void operator+=(const Pose &pose)
    {
      position_ += pose.position_;
      rotation_ += pose.rotation_;
    }
    void operator/=(double x)
    {
      position_ /= x;
      rotation_ /= x;
    }
    bool operator==(const Pose &pose)
    {
      return ((position_ == pose.position_) && (rotation_ == pose.rotation_));
    }
    bool operator!=(const Pose &pose)
    {
      return ((position_ != pose.position_) || (rotation_ != pose.rotation_));
    }

    Pose operator~() const
    {
      Quat inv = ~rotation_;
      return Pose(inv.rotateVector(-position_), inv);
    }
    double &operator[](int index)
    {
      ASSERT(index >= 0 && index < 7);
      if (index < 3)
	return position_[index];
      return rotation_[index - 3];
    }

    /// Normalise in-place
    void normalize()
    {
      rotation_.normalize();
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
      return position_ + rotation_.rotateVector(vec);
    }
    Vector3d inverseTransformVector(const Vector3d &vec) const
    {
      return (~*this).transformVector(vec);
    }

    static Pose identity()
    {
      return Pose(Vector3d(0, 0, 0), Quat(1, 0, 0, 0));
    }
    static Pose zero()
    {
      return Pose(Vector3d(0, 0, 0), Quat(0, 0, 0, 0));
    }
};

#endif /* SIMPLE_HEXAPOD_CONTROLLER_POSE_H */
