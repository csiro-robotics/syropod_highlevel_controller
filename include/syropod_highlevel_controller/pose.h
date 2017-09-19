#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_POSE_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_POSE_H
/*******************************************************************************************************************//**
 *  @file    pose.h
 *  @brief   Custom class definition of Syropod pose.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    September 2017
 *  @version 0.5.4
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
 * 3d orientation and position
***********************************************************************************************************************/
class Pose
{
public:
  inline Pose(void) {};
  inline Pose(const Vector3d& position, const Quaterniond& rotation)
  {
    position_ = position;
    rotation_ = rotation.normalized();
  };
  
  inline Pose operator*(const Quaterniond& quat) const 
  { 
    return Pose(position_, rotation_ * quat);
  }
  
  inline Vector3d operator*(const Vector3d& vector) const 
  { 
    return position_ + rotation_._transformVector(vector); 
  }
  
  inline bool operator==(const Pose& pose)
  { 
    return (position_ == pose.position_ && rotation_.isApprox(pose.rotation_));
  }

  inline bool operator!=(const Pose& pose)
  {
    return (position_ != pose.position_ || !rotation_.isApprox(pose.rotation_));
  }

  inline Pose operator~(void) const 
  { 
    return Pose((rotation_.conjugate())._transformVector(-position_), rotation_.conjugate());
  }
  
  inline Vector3d transformVector(const Vector3d& vec) const 
  { 
    return position_ + rotation_._transformVector(vec);
  };
  
  inline Vector3d inverseTransformVector(const Vector3d& vec) const 
  { 
    return (~*this).transformVector(vec);
  };
  inline void normalize(void) { rotation_.normalize(); }; // Normalise in-place
  inline Pose addPose(const Pose& pose) { return Pose(position_ + pose.position_, rotation_ * pose.rotation_); };
  inline Pose removePose (const Pose& pose)
  { 
    return Pose(position_ - pose.position_, rotation_ * pose.rotation_.inverse());
  };
  
  inline static Pose identity(void) { return Pose(Vector3d(0, 0, 0), Quaterniond(1, 0, 0, 0)); };
  inline static Pose zero(void) { return Pose(Vector3d(0, 0, 0), Quaterniond(0, 0, 0, 0)); };
  
  Vector3d position_;
  Quaterniond rotation_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_POSE_H */
