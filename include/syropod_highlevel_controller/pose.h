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

/*******************************************************************************************************************//**
 * Class defining a 3d position and rotation using Eigen Vector3 and Quaternion classes
***********************************************************************************************************************/
class Pose
{
public:
  
  /**
   * Blank contructor
   */
  inline Pose(void) {};
  
  /**
   * Pose class constructor
   */
  inline Pose(const Vector3d& position, const Quaterniond& rotation)
  {
    position_ = position;
    rotation_ = rotation.normalized();
  };
  
  /**
   * Operator to check if two poses are equivalent
   * @params[in] pose The pose that is checked for equivalency against *this
   * @return Bool defining if input and *this pose are equivalent.
   */
  inline bool operator==(const Pose& pose)
  { 
    return (position_ == pose.position_ && rotation_.isApprox(pose.rotation_));
  }

  /**
   * Operator to check if two poses are NOT equivalent
   * @params[in] pose The pose that is checked for non-equivalency against *this
   * @return Bool defining if input and *this pose are non-equivalent.
   */
  inline bool operator!=(const Pose& pose)
  {
    return (position_ != pose.position_ || !rotation_.isApprox(pose.rotation_));
  }

  /**
   * Returns inverse of pose
   * @return The inverse of *this pose
   */
  inline Pose operator~(void) const 
  { 
    return Pose((rotation_.conjugate())._transformVector(-position_), rotation_.conjugate());
  }
  
  /**
   * Transforms an input position into the reference frame of this pose.
   * @params[in] vec The input vector to be transformed into this pose's reference frame.
   * @return The transformed vector.
   */
  inline Vector3d transformVector(const Vector3d& vec) const 
  { 
    return position_ + rotation_._transformVector(vec);
  };
  
  /**
   * Transforms an input position vector from the reference frame of this pose.
   * @params[in] vec The input vector to be transformed from this pose's reference frame.
   * @return The inversly transformed vector
   */
  inline Vector3d inverseTransformVector(const Vector3d& vec) const 
  { 
    return (~*this).transformVector(vec);
  };
  
  /**
   * Adds input pose to *this pose
   * @params[in] pose The pose to add from *this pose
   * @return The combination of *this pose and input pose
   */
  inline Pose addPose(const Pose& pose) 
  {
    return Pose(position_ + pose.position_, rotation_ * pose.rotation_);
  }
  
  /**
   * Removes input pose from *this pose
   * @params[in] pose The pose to remove from *this pose
   * @return The resultant pose after removing input pose from *this pose
   */
  inline Pose removePose (const Pose& pose)
  { 
    return Pose(position_ - pose.position_, rotation_ * pose.rotation_.inverse());
  };
  
  /**
   * Returns pose with position and rotation elements set to identity values.
   * @return The Identity Pose
   */
  inline static Pose identity(void) 
  { 
    return Pose(Vector3d(0, 0, 0), Quaterniond(1, 0, 0, 0)); 
  }
  
  /**
   * Returns pose with position and rotation elements all set to zero.
   * @return The Zero Pose
   */
  inline static Pose zero(void) 
  { 
    return Pose(Vector3d(0, 0, 0), Quaterniond(0, 0, 0, 0));
  }
  
  Vector3d position_;     ///< The position element of the pose class
  Quaterniond rotation_;  ///< The rotation element of the pose class

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/***********************************************************************************************************************
***********************************************************************************************************************/
#endif /* SYROPOD_HIGHLEVEL_CONTROLLER_POSE_H */
