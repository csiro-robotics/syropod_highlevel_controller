////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../include/syropod_highlevel_controller/walk_controller.h"
#include "../include/syropod_highlevel_controller/pose_controller.h"
#include "../include/syropod_highlevel_controller/debug_visualiser.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

WalkController::WalkController(std::shared_ptr<Model> model, const Parameters &params)
    : model_(model), params_(params)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::init(void)
{
  time_delta_ = params_.time_delta.data;
  walk_state_ = STOPPED;
  walk_plane_ = Eigen::Vector3d::Zero();
  walk_plane_normal_ = Eigen::Vector3d::UnitZ();
  odometry_ideal_ = Pose::Identity();

  // Set default stance tip positions from parameters
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    double x_position = params_.leg_stance_positions[leg->getIDNumber()].data.at("x");
    double y_position = params_.leg_stance_positions[leg->getIDNumber()].data.at("y");
    Eigen::Quaterniond identity_tip_rotation = UNDEFINED_ROTATION;
    if (leg->getJointCount() > 3 && params_.gravity_aligned_tips.data)
    {
      identity_tip_rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitZ());
      identity_tip_rotation = correctRotation(identity_tip_rotation, Eigen::Quaterniond::Identity());
    }
    Pose identity_tip_pose(Eigen::Vector3d(x_position, y_position, 0.0), identity_tip_rotation);
    leg->setLegStepper(std::allocate_shared<LegStepper>(Eigen::aligned_allocator<LegStepper>(),
                                                        shared_from_this(), leg, identity_tip_pose));
  }

  // Init velocity input variables
  desired_linear_velocity_ = Eigen::Vector2d(0, 0);
  desired_angular_velocity_ = 0;

  // Generate step timing
  generateStepCycle();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::generateWalkspace(void)
{
  // Initially populate walkspace with maximum values (without overlapping between adjacent legs)
  walkspace_.clear();
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    // Get positions of adjacent legs
    int leg_count = model_->getLegCount();
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(mod(leg->getIDNumber() + 1, leg_count));
    std::shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(mod(leg->getIDNumber() - 1, leg_count));
    Eigen::Vector3d default_tip_position = leg->getLegStepper()->getDefaultTipPose().position_;
    Eigen::Vector3d adjacent_1_tip_position = adjacent_leg_1->getLegStepper()->getDefaultTipPose().position_;
    Eigen::Vector3d adjacent_2_tip_position = adjacent_leg_2->getLegStepper()->getDefaultTipPose().position_;

    // Get distance and bearing to adjacent legs from this leg
    double distance_to_adjacent_leg_1 = Eigen::Vector3d(default_tip_position - adjacent_1_tip_position).norm() / 2.0;
    double distance_to_adjacent_leg_2 = Eigen::Vector3d(default_tip_position - adjacent_2_tip_position).norm() / 2.0;
    int bearing_to_adjacent_leg_1 = radiansToDegrees(atan2(adjacent_1_tip_position[1] - default_tip_position[1],
                                                           adjacent_1_tip_position[0] - default_tip_position[0]));
    int bearing_to_adjacent_leg_2 = radiansToDegrees(atan2(adjacent_2_tip_position[1] - default_tip_position[1],
                                                           adjacent_2_tip_position[0] - default_tip_position[0]));

    // Populate walkspace
    for (int bearing = 0; bearing <= 360; bearing += BEARING_STEP)
    {
      int bearing_diff_1 = abs(mod(bearing_to_adjacent_leg_1, 360) - bearing);
      int bearing_diff_2 = abs(mod(bearing_to_adjacent_leg_2, 360) - bearing);
      double distance_to_overlap_1 = UNASSIGNED_VALUE;
      double distance_to_overlap_2 = UNASSIGNED_VALUE;
      if ((bearing_diff_1 < 90 || bearing_diff_1 > 270) && distance_to_adjacent_leg_1 > 0.0)
      {
        distance_to_overlap_1 = distance_to_adjacent_leg_1 / cos(degreesToRadians(bearing_diff_1));
      }
      if ((bearing_diff_2 < 90 || bearing_diff_2 > 270) && distance_to_adjacent_leg_2 > 0.0)
      {
        distance_to_overlap_2 = distance_to_adjacent_leg_2 / cos(degreesToRadians(bearing_diff_2));
      }
      bool overlapping = params_.overlapping_walkspaces.data;
      double min_distance = overlapping ? MAX_WORKSPACE_RADIUS : std::min(distance_to_overlap_1, distance_to_overlap_2);
      min_distance = std::min(min_distance, MAX_WORKSPACE_RADIUS);
      if (walkspace_.find(bearing) != walkspace_.end() && min_distance < walkspace_[bearing])
      {
        walkspace_[bearing] = min_distance;
      }
      else
      {
        walkspace_.insert(LimitMap::value_type(bearing, min_distance));
      }
    }
  }

  // Generate walkspace for each leg whilst ensuring symmetry and minimum values
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

    // Calculate target height of plane within workspace
    Pose current_pose = model_->getCurrentPose();
    Eigen::Vector3d identity_tip_position =
        current_pose.inverseTransformVector(leg_stepper->getIdentityTipPose().position_);
    Eigen::Vector3d default_tip_position =
        current_pose.inverseTransformVector(leg_stepper->getDefaultTipPose().position_);
    Eigen::Vector3d default_shift = default_tip_position - identity_tip_position;
    double target_workplane_height = default_shift[2];
    LimitMap workplane = leg->getWorkplane(target_workplane_height); // Interpolated workplane
    if (workplane.empty())
    {
      continue;
    }

    // Generate walkspace radii
    LimitMap::iterator walkspace_it;
    for (walkspace_it = walkspace_.begin(); walkspace_it != walkspace_.end(); ++walkspace_it)
    {
      int bearing = walkspace_it->first;
      double radius = walkspace_it->second;

      // If default tip position is equal to identity tip position skip default shift radius generation
      if (default_shift.norm() == 0.0)
      {
        radius = workplane.at(bearing);
      }
      // Generate radius from interpolated workplane for shifted default tip position within plane.
      else
      {
        // Generate new point in walkspace
        Eigen::Vector3d new_point = Eigen::Vector3d::UnitX() * MAX_WORKSPACE_RADIUS;
        new_point = Eigen::AngleAxisd(degreesToRadians(bearing), Eigen::Vector3d::UnitZ())._transformVector(new_point);
        new_point = setPrecision(new_point, 3);

        // Generate radius from finding intersection of new point direction vector and existing workplane limits
        LimitMap::iterator workplane_it;
        for (workplane_it = workplane.begin(); workplane_it != workplane.end(); ++workplane_it)
        {
          // Generate reference point 1
          int bearing_1 = workplane_it->first;
          double radius_1 = workplane_it->second;
          Eigen::Vector3d point_1 = Eigen::Vector3d::UnitX() * radius_1;
          point_1 = Eigen::AngleAxisd(degreesToRadians(bearing_1), Eigen::Vector3d::UnitZ())._transformVector(point_1);
          point_1 -= default_shift;
          point_1[2] = 0.0;
          point_1 = setPrecision(point_1, 3);

          // Unable to find reference points which bound new walkspace point direction vector therefore set zero radius
          if (bearing_1 == workplane.rbegin()->first)
          {
            ROS_WARN("\n[SHC] Unable to generate radius at bearing %d for leg %s and workplane at height %f.\n",
                     bearing, leg->getIDName().c_str(), target_workplane_height);
            radius = 0.0;
            break;
          }

          // Generate reference point 2
          int bearing_2 = next(workplane_it)->first;
          double radius_2 = next(workplane_it)->second;
          Eigen::Vector3d point_2 = Eigen::Vector3d::UnitX() * radius_2;
          point_2 = Eigen::AngleAxisd(degreesToRadians(bearing_2), Eigen::Vector3d::UnitZ())._transformVector(point_2);
          point_2 -= default_shift;
          point_2[2] = 0.0;
          point_2 = setPrecision(point_2, 3);

          // Reference point 1 in same direction as new point
          if (point_1.cross(new_point).norm() == 0.0)
          {
            radius = point_1.norm();
            break;
          }
          // Reference point 2 in same direction as new point
          else if (point_2.cross(new_point).norm() == 0.0)
          {
            radius = point_2.norm();
            break;
          }
          // New point direction is between reference points - calculate distance to line connecting reference points
          // Ref: stackoverflow.com/questions/13640931/how-to-determine-if-a-vector-is-between-two-other-vectors
          else if (point_1.cross(new_point).dot(point_1.cross(point_2)) >= 0.0 &&
                   point_2.cross(new_point).dot(point_2.cross(point_1)) >= 0.0)
          {
            // Calculate vector (with same direction as new point) normal to line connecting p1 & p2 on horizontal plane
            double dx = point_2[0] - point_1[0];
            double dy = point_2[1] - point_1[1];
            Eigen::Vector3d normal_1 = Eigen::Vector3d(dy, -dx, 0.0).normalized();
            Eigen::Vector3d normal_2 = Eigen::Vector3d(-dy, dx, 0.0).normalized();
            bool same_direction_as_new_point = getProjection(new_point, normal_1).dot(normal_1) >= 0.0;
            Eigen::Vector3d normal = (same_direction_as_new_point ? normal_1 : normal_2);

            // Use normal to calculate intersection distance of new point vector on line connecting p1 & p2.
            Eigen::Vector3d new_point_projection = getProjection(new_point, normal);
            Eigen::Vector3d point_1_projection = getProjection(point_1, normal);
            double ratio = point_1_projection.norm() / new_point_projection.norm();
            radius = ratio * MAX_WORKSPACE_RADIUS;
            break;
          }
        }
      }

      // Add calculated radius to walkspace whilst ensuring symmetry and min values.
      int opposite_bearing = mod(bearing + 180, 360);
      if (radius < walkspace_.at(bearing))
      {
        walkspace_[bearing] = radius;
        walkspace_[opposite_bearing] = radius;
      }
    }
  }
  walkspace_[360] = walkspace_[0];
  regenerate_walkspace_ = false;
  generateLimits();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::generateLimits(StepCycle step,
                                    LimitMap *max_linear_speed_ptr,
                                    LimitMap *max_angular_speed_ptr,
                                    LimitMap *max_linear_acceleration_ptr,
                                    LimitMap *max_angular_acceleration_ptr)
{
  int base_step_period = params_.stance_phase.data + params_.swing_phase.data;
  int normaliser = step.period_ / base_step_period;
  int base_step_offset = int(params_.phase_offset.data * normaliser);

  bool set_limits = (!max_linear_speed_ptr && !max_linear_acceleration_ptr &&
                     !max_angular_speed_ptr && !max_angular_acceleration_ptr);
  if (set_limits)
  {
    max_linear_speed_ptr = &max_linear_speed_;
    max_angular_speed_ptr = &max_angular_speed_;
    max_linear_acceleration_ptr = &max_linear_acceleration_;
    max_angular_acceleration_ptr = &max_angular_acceleration_;
  }

  if (max_linear_speed_ptr)
  {
    max_linear_speed_ptr->clear();
  };
  if (max_linear_acceleration_ptr)
  {
    max_linear_acceleration_ptr->clear();
  };
  if (max_angular_speed_ptr)
  {
    max_angular_speed_ptr->clear();
  };
  if (max_angular_acceleration_ptr)
  {
    max_angular_acceleration_ptr->clear();
  };

  // Set step offset and check if leg starts in swing period (i.e. forced to stance for the 1st step cycle)
  // If so find this max 'stance extension' period which is used in acceleration calculations
  int max_stance_extension = 0;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    ROS_ASSERT(params_.offset_multiplier.data.count(leg->getIDName()));
    int multiplier = params_.offset_multiplier.data.at(leg->getIDName());
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    int step_offset = (base_step_offset * multiplier) % step.period_;
    leg_stepper->setPhaseOffset(step_offset);
    if (step_offset > step.swing_start_ && step_offset < step.swing_end_) // SWING STATE
    {
      max_stance_extension = std::max(max_stance_extension, step.swing_end_ - step_offset);
    }
  }

  // Set max stride (i.e. max body velocity) to occur at end of 1st swing of leg with maximum stance period extension
  double time_to_max_stride = (max_stance_extension + step.stance_period_ + step.swing_period_) * time_delta_;

  // Calculate initial max speed and acceleration of body
  LimitMap::iterator it;
  for (it = walkspace_.begin(); it != walkspace_.end(); ++it)
  {
    double walkspace_radius = it->second;
    double on_ground_ratio = double(step.stance_period_) / step.period_;
    double max_speed = (walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
    double max_acceleration = max_speed / time_to_max_stride;

    // Calculates max overshoot of tip (in stance period) outside walkspace
    double stance_overshoot = 0;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      std::shared_ptr<Leg> leg = leg_it_->second;
      std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      // All referenced swings are the LAST swing period BEFORE the max velocity (stride length) is reached
      double step_offset = leg_stepper->getPhaseOffset();
      double t = step_offset * time_delta_; // Time between swing end and max velocity being reached
      double time_to_swing_end = time_to_max_stride - t;
      double v0 = max_acceleration * time_to_swing_end; // Tip velocity at time of swing end
      double stride_length = v0 * (on_ground_ratio / step.frequency_);
      double d0 = -stride_length / 2.0;                                          // Distance to default tip position at time of swing end
      double d1 = d0 + v0 * t + 0.5 * max_acceleration * sqr(t);                 // Distance from default tip position at max velocity
      double d2 = max_speed * (step.stance_period_ * time_delta_ - t);           // Distance from default position at stance end
      stance_overshoot = std::max(stance_overshoot, d1 + d2 - walkspace_radius); // Max overshoot past walkspace limits
    }

    // Scale walkspace to accomodate stance overshoot and normal swing overshoot
    double swing_overshoot = 0.5 * max_speed * step.swing_period_ / (2.0 * step.period_ * step.frequency_);
    double scaled_walkspace_radius =
        (walkspace_radius / (walkspace_radius + stance_overshoot + swing_overshoot)) * walkspace_radius;

    // Stance radius based around front right leg to ensure positive values
    std::shared_ptr<Leg> reference_leg = model_->getLegByIDNumber(0);
    std::shared_ptr<LegStepper> reference_leg_stepper = reference_leg->getLegStepper();
    double x_position = reference_leg_stepper->getDefaultTipPose().position_[0];
    double y_position = reference_leg_stepper->getDefaultTipPose().position_[1];
    double stance_radius = Eigen::Vector2d(x_position, y_position).norm();

    // Distance: scaled_walkspace_radius*2.0 (i.e. max stride length)
    // Time: on_ground_ratio*(1/step_frequency_) where step frequency is FULL step cycles/s)
    double max_linear_speed = (scaled_walkspace_radius * 2.0) / (on_ground_ratio / step.frequency_);
    double max_linear_acceleration = max_linear_speed / time_to_max_stride;
    double max_angular_speed = max_linear_speed / stance_radius;
    double max_angular_acceleration = max_angular_speed / time_to_max_stride;

    // Handle zero walkspace
    if (walkspace_radius == 0.0)
    {
      max_linear_speed = 0.0;
      max_linear_acceleration = UNASSIGNED_VALUE;
      max_angular_speed = 0.0;
      max_angular_acceleration = UNASSIGNED_VALUE;
    }

    // Populate limit maps
    if (max_linear_speed_ptr)
    {
      max_linear_speed_ptr->insert(LimitMap::value_type(it->first, max_linear_speed));
    }
    if (max_linear_acceleration_ptr)
    {
      max_linear_acceleration_ptr->insert(LimitMap::value_type(it->first, max_linear_acceleration));
    }
    if (max_angular_speed_ptr)
    {
      max_angular_speed_ptr->insert(LimitMap::value_type(it->first, max_angular_speed));
    }
    if (max_angular_acceleration_ptr)
    {
      max_angular_acceleration_ptr->insert(LimitMap::value_type(it->first, max_angular_acceleration));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

StepCycle WalkController::generateStepCycle(const bool set_step_cycle)
{
  StepCycle step;
  step.stance_end_ = params_.stance_phase.data * 0.5;
  step.swing_start_ = step.stance_end_;
  step.swing_end_ = step.swing_start_ + params_.swing_phase.data;
  step.stance_start_ = step.swing_end_;

  // Normalises the step period to match the total number of iterations over a full step
  int base_step_period = params_.stance_phase.data + params_.swing_phase.data;
  double swing_ratio = double(params_.swing_phase.data) / double(base_step_period); // Modifies step frequency

  // Ensure step period is even and divisible by base step period and therefore gives whole even normaliser value
  double raw_step_period = ((1.0 / params_.step_frequency.current_value) / time_delta_) / swing_ratio;
  step.period_ = roundToEvenInt(raw_step_period / base_step_period) * base_step_period;

  step.frequency_ = 1.0 / (step.period_ * time_delta_); // adjust step frequency to match corrected step period
  int normaliser = step.period_ / base_step_period;
  step.stance_end_ *= normaliser;
  step.swing_start_ *= normaliser;
  step.swing_end_ *= normaliser;
  step.stance_start_ *= normaliser;

  step.stance_period_ = mod(step.stance_end_ - step.stance_start_, step.period_);
  step.swing_period_ = step.swing_end_ - step.swing_start_;

  // Ensure stance and swing periods are divisible by two
  ROS_ASSERT(step.stance_period_ % 2 == 0);
  ROS_ASSERT(step.swing_period_ % 2 == 0);

  // Set step cycle in walk controller and update phase in leg steppers for new parameters if required
  if (set_step_cycle)
  {
    step_ = step;
    if (walk_state_ == MOVING)
    {
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        std::shared_ptr<Leg> leg = leg_it_->second;
        std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        leg_stepper->updatePhase();
      }
    }
  }
  return step;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double WalkController::getLimit(const Eigen::Vector2d &linear_velocity_input,
                                const double &angular_velocity_input,
                                const LimitMap &limit)
{
  double min_limit = UNASSIGNED_VALUE;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    Eigen::Vector3d tip_position = leg_stepper->getCurrentTipPose().position_;
    Eigen::Vector2d rotation_normal = Eigen::Vector2d(-tip_position[1], tip_position[0]);
    Eigen::Vector2d stride_vector = linear_velocity_input + angular_velocity_input * rotation_normal;
    int bearing = mod(roundToInt(radiansToDegrees(atan2(stride_vector[1], stride_vector[0]))), 360);
    int upper_bound = limit.lower_bound(bearing)->first;
    int lower_bound = mod(upper_bound - BEARING_STEP, 360);
    bearing += (bearing < lower_bound) ? 360 : 0;
    upper_bound += (upper_bound < lower_bound) ? 360 : 0;
    double control_input = (bearing - lower_bound) / (upper_bound - lower_bound);
    double limit_interpolation = interpolate(limit.at(lower_bound), limit.at(mod(upper_bound, 360)), control_input);
    min_limit = std::min(min_limit, limit_interpolation);
  }
  return min_limit;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::updateWalk(const Eigen::Vector2d &linear_velocity_input, const double &angular_velocity_input)
{
  Eigen::Vector2d new_linear_velocity;
  double new_angular_velocity;

  double max_linear_speed = getLimit(linear_velocity_input, angular_velocity_input, max_linear_speed_);
  double max_angular_speed = getLimit(linear_velocity_input, angular_velocity_input, max_angular_speed_);
  double max_linear_acceleration = getLimit(linear_velocity_input, angular_velocity_input, max_linear_acceleration_);
  double max_angular_acceleration = getLimit(linear_velocity_input, angular_velocity_input, max_angular_acceleration_);

  // Calculate desired angular/linear velocities according to input mode and max limits
  if (walk_state_ != STOPPING)
  {
    if (params_.velocity_input_mode.data == "throttle")
    {
      new_linear_velocity = clamped(linear_velocity_input, 1.0) * max_linear_speed; // Forces input between -1.0/1.0
      new_angular_velocity = clamped(angular_velocity_input, -1.0, 1.0) * max_angular_speed;

      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (1.0 - abs(angular_velocity_input));
    }
    else if (params_.velocity_input_mode.data == "real")
    {
      new_linear_velocity = clamped(linear_velocity_input, max_linear_speed);
      new_angular_velocity = clamped(angular_velocity_input, -max_angular_speed, max_angular_speed);

      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (max_angular_speed != 0.0 ? (1.0 - abs(new_angular_velocity / max_angular_speed)) : 0.0);

      if (linear_velocity_input.norm() > max_linear_speed)
      {
        ROS_WARN_THROTTLE(THROTTLE_PERIOD,
                          "\nInput linear speed (%f) exceeds maximum linear speed (%f) and has been clamped.\n",
                          linear_velocity_input.norm(), max_linear_speed);
      }
      if (abs(angular_velocity_input) > max_angular_speed)
      {
        ROS_WARN_THROTTLE(THROTTLE_PERIOD,
                          "\nInput angular velocity (%f) exceeds maximum angular speed (%f) and has been clamped.\n",
                          abs(angular_velocity_input), max_angular_speed);
      }
    }
  }
  else
  {
    new_linear_velocity = Eigen::Vector2d(0.0, 0.0);
    new_angular_velocity = 0.0;
  }

  bool has_velocity_command = linear_velocity_input.norm() || angular_velocity_input;

  // Check that all legs are in WALKING state
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg->getLegState() != WALKING)
    {
      if (linear_velocity_input.norm())
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD,
                          "\nUnable to walk whilst manually manipulating legs, ensure each leg is in walking state.\n");
      }
      return;
    }
  }

  // Update linear velocity according to acceleration limits
  Eigen::Vector2d linear_acceleration = new_linear_velocity - desired_linear_velocity_;
  if (linear_acceleration.norm() < max_linear_acceleration * time_delta_)
  {
    desired_linear_velocity_ += linear_acceleration;
  }
  else
  {
    desired_linear_velocity_ += linear_acceleration.normalized() * max_linear_acceleration * time_delta_;
  }

  // Update angular velocity according to acceleration limits
  double angular_acceleration = new_angular_velocity - desired_angular_velocity_;
  if (abs(angular_acceleration) < max_angular_acceleration * time_delta_)
  {
    desired_angular_velocity_ += angular_acceleration;
  }
  else
  {
    desired_angular_velocity_ += sign(angular_acceleration) * max_angular_acceleration * time_delta_;
  }

  // State transitions for Walk State Machine
  // State transition: STOPPED->STARTING
  int leg_count = model_->getLegCount();
  if (walk_state_ == STOPPED && has_velocity_command)
  {
    walk_state_ = STARTING;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      std::shared_ptr<Leg> leg = leg_it_->second;
      std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      leg_stepper->setAtCorrectPhase(false);
      leg_stepper->setCompletedFirstStep(false);
      leg_stepper->setStepState(STANCE);
      leg_stepper->setPhase(leg_stepper->getPhaseOffset());
      leg_stepper->updateStepState();
    }
    return; // Skips iteration of phase so auto posing can catch up
  }
  // State transition: STARTING->MOVING
  else if (walk_state_ == STARTING && legs_at_correct_phase_ == leg_count && legs_completed_first_step_ == leg_count)
  {
    legs_at_correct_phase_ = 0;
    legs_completed_first_step_ = 0;
    walk_state_ = MOVING;
  }
  // State transition: MOVING->STOPPING
  else if (walk_state_ == MOVING && !has_velocity_command)
  {
    walk_state_ = STOPPING;
  }
  // State transition: STOPPING->STOPPED
  else if (walk_state_ == STOPPING && legs_at_correct_phase_ == leg_count && pose_state_ == POSING_COMPLETE)
  {
    legs_at_correct_phase_ = 0;
    walk_state_ = STOPPED;
  }

  // Update walk/step state and tip position along trajectory for each leg
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

    // Walk State Machine
    if (walk_state_ == STARTING)
    {
      // Check if all legs have completed one step
      if (legs_at_correct_phase_ == leg_count)
      {
        if (leg_stepper->getPhase() == step_.swing_end_ && !leg_stepper->hasCompletedFirstStep())
        {
          leg_stepper->setCompletedFirstStep(true);
          legs_completed_first_step_++;
        }
      }
      // Force any leg state into STANCE if it starts offset in a mid-swing state but has not finished swing end
      if (!leg_stepper->isAtCorrectPhase())
      {
        if (leg_stepper->getPhaseOffset() > step_.swing_start_ &&
            leg_stepper->getPhaseOffset() < step_.swing_end_ && // SWING STATE
            leg_stepper->getPhase() != step_.swing_end_)
        {
          leg_stepper->setStepState(FORCE_STANCE);
        }
        else
        {
          legs_at_correct_phase_++;
          leg_stepper->setAtCorrectPhase(true);
        }
      }
    }
    else if (walk_state_ == MOVING)
    {
      leg_stepper->setAtCorrectPhase(false);
    }
    else if (walk_state_ == STOPPING)
    {
      // All legs must attempt at least one step to achieve default tip position after ending a swing
      bool zero_body_velocity = leg_stepper->getStrideVector().norm() == 0;
      Eigen::Vector3d walk_plane_normal = leg_stepper->getWalkPlaneNormal();
      Eigen::Vector3d error = (leg_stepper->getCurrentTipPose().position_ - leg_stepper->getTargetTipPose().position_);
      error = getRejection(error, walk_plane_normal);
      bool at_target_tip_position = (error.norm() < TIP_TOLERANCE);
      if (zero_body_velocity && !leg_stepper->isAtCorrectPhase() && leg_stepper->getPhase() == step_.swing_end_)
      {
        if (at_target_tip_position || return_to_default_attempted_)
        {
          return_to_default_attempted_ = false;
          leg_stepper->updateDefaultTipPosition();
          leg_stepper->setStepState(FORCE_STOP);
          leg_stepper->setAtCorrectPhase(true);
          legs_at_correct_phase_++;
        }
        else
        {
          return_to_default_attempted_ = true;
        }
      }
    }
    else if (walk_state_ == STOPPED)
    {
      leg_stepper->setStepState(FORCE_STOP);
      leg_stepper->setPhase(0.0);
    }

    // Update tip positions
    if (/*walk_state_ != STOPPED && */ leg->getLegState() == WALKING)
    {
      leg_stepper->updateTipPosition(); // Updates current tip position through step cycle
      leg_stepper->updateTipRotation();
      leg_stepper->iteratePhase();
    }
  }
  updateWalkPlane();
  odometry_ideal_ = odometry_ideal_.addPose(calculateOdometry(time_delta_));
  if (regenerate_walkspace_)
  {
    generateWalkspace();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::updateManual(const int &primary_leg_selection_ID,
                                  const Eigen::Vector3d &primary_tip_velocity_input,
                                  const int &secondary_leg_selection_ID,
                                  const Eigen::Vector3d &secondary_tip_velocity_input)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg->getLegState() == MANUAL)
    {
      Eigen::Vector3d tip_velocity_input;
      int selected_leg_ID = leg->getIDNumber();
      if (selected_leg_ID == primary_leg_selection_ID)
      {
        tip_velocity_input = primary_tip_velocity_input;
      }
      else if (selected_leg_ID == secondary_leg_selection_ID)
      {
        tip_velocity_input = secondary_tip_velocity_input;
      }

      if (tip_velocity_input.norm() != 0.0)
      {
        // Joint control works only for 3DOF legs as velocity inputs for x/y/z axes mapped to positions for joints 1/2/3
        if (params_.leg_manipulation_mode.data == "joint_control" && leg->getJointCount() == 3) // HACK
        {
          double coxa_joint_velocity = tip_velocity_input[1] * params_.max_rotation_velocity.data * time_delta_;
          double tibia_joint_velocity = tip_velocity_input[0] * params_.max_rotation_velocity.data * time_delta_;
          double coxa_joint_position = leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->desired_position_;
          double tibia_joint_position = leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->desired_position_;
          leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->prev_desired_position_ = coxa_joint_position;
          leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->prev_desired_position_ = tibia_joint_position;
          leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->desired_position_ += coxa_joint_velocity;
          leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->desired_position_ += tibia_joint_velocity;
          Pose new_tip_pose = leg->applyFK(false);
          leg_stepper->setCurrentTipPose(new_tip_pose);
        }
        else if (params_.leg_manipulation_mode.data == "tip_control")
        {
          Eigen::Vector3d ik_error = leg->getDesiredTipPose().position_ - leg->getCurrentTipPose().position_;
          Eigen::Vector3d tip_position_change =
              tip_velocity_input * params_.max_translation_velocity.data * time_delta_;
          if (ik_error.norm() >= IK_TOLERANCE)
          {
            tip_position_change = tip_position_change.norm() * -ik_error.normalized();
            ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nCannot move leg %s any further due to IK or joint limits.\n",
                              leg->getIDName().c_str());
          }
          Eigen::Vector3d new_tip_position = leg_stepper->getCurrentTipPose().position_ + tip_position_change;
          leg_stepper->setCurrentTipPose(Pose(new_tip_position, UNDEFINED_ROTATION));
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::updateManual(const int &primary_leg_selection_ID, const Pose &primary_tip_pose_input,
                                  const int &secondary_leg_selection_ID, const Pose &secondary_tip_pose_input)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    std::shared_ptr<Leg> leg = leg_it_->second;
    std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg->getLegState() == MANUAL)
    {
      Eigen::Vector3d tip_position_input;
      Eigen::Quaterniond tip_rotation_input;
      int selected_leg_ID = leg->getIDNumber();
      if (selected_leg_ID == primary_leg_selection_ID)
      {
        tip_position_input = primary_tip_pose_input.position_;
        tip_rotation_input = primary_tip_pose_input.rotation_;
      }
      else if (selected_leg_ID == secondary_leg_selection_ID)
      {
        tip_position_input = secondary_tip_pose_input.position_;
        tip_rotation_input = secondary_tip_pose_input.rotation_;
      }
      if (tip_position_input.norm() != 0.0)
      {
        if (params_.leg_manipulation_mode.data == "tip_control")
        {
          
          // TODO add orientation control
          // leg_stepper->setCurrentTipPose(Pose(tip_position_input, tip_rotation_input));
          leg_stepper->setCurrentTipPose(Pose(tip_position_input, UNDEFINED_ROTATION));
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void WalkController::updateWalkPlane(void)
{
  std::vector<double> raw_A;
  std::vector<double> raw_B;
  if (model_->getLegCount() >= 3) // Minimum for plane estimation
  {
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      std::shared_ptr<Leg> leg = leg_it_->second;
      std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      raw_A.push_back(leg_stepper->getDefaultTipPose().position_[0]);
      raw_A.push_back(leg_stepper->getDefaultTipPose().position_[1]);
      raw_A.push_back(1.0);
      raw_B.push_back(leg_stepper->getDefaultTipPose().position_[2]);
    }

    // Estimate walk plane
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A(raw_A.data(),
                                                                                         model_->getLegCount(), 3);
    Eigen::Map<Eigen::VectorXd> B(raw_B.data(), model_->getLegCount());
    Eigen::MatrixXd pseudo_inverse_A = (A.transpose() * A).inverse() * A.transpose();
    walk_plane_ = (pseudo_inverse_A * B);
    walk_plane_normal_ = Eigen::Vector3d(-walk_plane_[0], -walk_plane_[1], 1.0).normalized();
    ROS_ASSERT(walk_plane_.norm() < UNASSIGNED_VALUE);
    ROS_ASSERT(walk_plane_normal_.norm() < UNASSIGNED_VALUE);
  }
  else
  {
    walk_plane_ = Eigen::Vector3d::Zero();
    walk_plane_normal_ = Eigen::Vector3d::UnitZ();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pose WalkController::calculateOdometry(const double &time_period)
{
  Eigen::Vector3d desired_linear_velocity =
      Eigen::Vector3d(desired_linear_velocity_[0], desired_linear_velocity_[1], 0);
  Eigen::Vector3d position_delta = desired_linear_velocity * time_period;
  Eigen::Quaterniond rotation_delta =
      Eigen::Quaterniond(Eigen::AngleAxisd(desired_angular_velocity_ * time_period, Eigen::Vector3d::UnitZ()));
  return Pose(position_delta, rotation_delta);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LegStepper::LegStepper(std::shared_ptr<WalkController> walker, std::shared_ptr<Leg> leg, const Pose &identity_tip_pose)
    : walker_(walker), leg_(leg), identity_tip_pose_(identity_tip_pose), default_tip_pose_(identity_tip_pose), current_tip_pose_(default_tip_pose_), origin_tip_pose_(current_tip_pose_), target_tip_pose_(default_tip_pose_)
{
  walk_plane_ = Eigen::Vector3d::Zero();
  walk_plane_normal_ = Eigen::Vector3d::UnitZ();
  stride_vector_ = Eigen::Vector3d::Zero();
  current_tip_velocity_ = Eigen::Vector3d::Zero();
  swing_origin_tip_position_ = default_tip_pose_.position_;
  stance_origin_tip_position_ = default_tip_pose_.position_;
  swing_clearance_ = Eigen::Vector3d(0.0, 0.0, walker->getStepClearance());

  // Iterate through and initialise control nodes (5 control nodes for quartic (4th order) bezier curves)
  for (int i = 0; i < 5; ++i)
  {
    swing_1_nodes_[i] = Eigen::Vector3d::Zero();
    swing_2_nodes_[i] = Eigen::Vector3d::Zero();
    stance_nodes_[i] = Eigen::Vector3d::Zero();
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

LegStepper::LegStepper(std::shared_ptr<LegStepper> leg_stepper)
    : walker_(leg_stepper->walker_), leg_(leg_stepper->leg_), identity_tip_pose_(leg_stepper->identity_tip_pose_), default_tip_pose_(leg_stepper->default_tip_pose_), current_tip_pose_(leg_stepper->current_tip_pose_), origin_tip_pose_(leg_stepper->origin_tip_pose_), target_tip_pose_(leg_stepper->target_tip_pose_)
{
  walk_plane_ = leg_stepper->walk_plane_;
  walk_plane_normal_ = leg_stepper->walk_plane_normal_;
  stride_vector_ = leg_stepper->stride_vector_;
  current_tip_velocity_ = leg_stepper->current_tip_velocity_;
  swing_origin_tip_position_ = leg_stepper->swing_origin_tip_position_;
  swing_origin_tip_velocity_ = leg_stepper->swing_origin_tip_velocity_;
  stance_origin_tip_position_ = leg_stepper->stance_origin_tip_position_;
  swing_clearance_ = leg_stepper->swing_clearance_;
  at_correct_phase_ = leg_stepper->at_correct_phase_;
  completed_first_step_ = leg_stepper->completed_first_step_;
  phase_ = leg_stepper->phase_;
  phase_offset_ = leg_stepper->phase_offset_;
  stance_progress_ = leg_stepper->stance_progress_;
  swing_progress_ = leg_stepper->swing_progress_;
  stance_progress_ = leg_stepper->stance_progress_;
  step_state_ = leg_stepper->step_state_;
  swing_delta_t_ = leg_stepper->swing_delta_t_;
  stance_delta_t_ = leg_stepper->stance_delta_t_;

  // Iterate through and initialise control nodes (5 control nodes for quartic (4th order) bezier curves)
  for (int i = 0; i < 5; ++i)
  {
    swing_1_nodes_[i] = leg_stepper->swing_1_nodes_[i];
    swing_2_nodes_[i] = leg_stepper->swing_2_nodes_[i];
    stance_nodes_[i] = leg_stepper->stance_nodes_[i];
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updatePhase(void)
{
  StepCycle step = walker_->getStepCycle();
  phase_ = step_progress_ * step.period_;
  updateStepState();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::iteratePhase(void)
{
  StepCycle step = walker_->getStepCycle();
  phase_ = (phase_ + 1) % (step.period_);
  updateStepState();

  // Calculate progress of stance/swing periods (0.0->1.0 or -1.0 if not in specific state)
  step_progress_ = double(phase_) / step.period_;
  if (step_state_ == SWING)
  {
    swing_progress_ = double(phase_ - step.swing_start_ + 1) / double(step.swing_end_ - step.swing_start_);
    swing_progress_ = clamped(swing_progress_, 0.0, 1.0);
    stance_progress_ = -1.0;
  }
  else if (step_state_ == STANCE)
  {
    stance_progress_ = double(mod(phase_ + (step.period_ - step.stance_start_), step.period_) + 1) /
                       double(mod(step.stance_end_ - step.stance_start_, step.period_));
    stance_progress_ = clamped(stance_progress_, 0.0, 1.0);
    swing_progress_ = -1.0;
  }
  else if (step_state_ == FORCE_STOP)
  {
    stance_progress_ = 0.0;
    swing_progress_ = -1.0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updateStepState(void)
{
  // Update step state from phase unless force stopped
  StepCycle step = walker_->getStepCycle();
  if (step_state_ == FORCE_STOP)
  {
    return;
  }
  else if (phase_ >= step.swing_start_ && phase_ < step.swing_end_ && step_state_ != FORCE_STANCE)
  {
    step_state_ = SWING;
  }
  else if (phase_ < step.stance_end_ || phase_ >= step.stance_start_)
  {
    step_state_ = STANCE;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updateStride(void)
{
  walk_plane_ = walker_->getWalkPlane();
  walk_plane_normal_ = walker_->getWalkPlaneNormal();
  ROS_ASSERT(walk_plane_.norm() < UNASSIGNED_VALUE);
  ROS_ASSERT(walk_plane_normal_.norm() < UNASSIGNED_VALUE);

  // Linear stride vector
  Eigen::Vector2d velocity = walker_->getDesiredLinearVelocity();
  Eigen::Vector3d stride_vector_linear(Eigen::Vector3d(velocity[0], velocity[1], 0.0));

  // Angular stride vector
  Eigen::Vector3d radius = getRejection(current_tip_pose_.position_, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d angular_velocity = walker_->getDesiredAngularVelocity() * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d stride_vector_angular = angular_velocity.cross(radius);

  // Combination and scaling
  stride_vector_ = stride_vector_linear + stride_vector_angular;
  StepCycle step = walker_->getStepCycle();
  double on_ground_ratio = double(step.stance_period_) / step.period_;
  stride_vector_ *= (on_ground_ratio / step.frequency_);

  // Swing clearance
  swing_clearance_ = walker_->getStepClearance() * walk_plane_normal_.normalized();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d LegStepper::calculateStanceSpanChange(void)
{
  // Calculate target height of plane within workspace
  Eigen::Vector3d default_shift = default_tip_pose_.position_ - identity_tip_pose_.position_;
  double target_workplane_height = setPrecision(default_shift[2], 3);

  // Get bounding existing workplanes within workspace
  Workspace workspace = leg_->getWorkspace();
  Workspace::iterator upper_bound_it = workspace.upper_bound(target_workplane_height);
  Workspace::iterator lower_bound_it = prev(upper_bound_it);
  double upper_workplane_height = setPrecision(upper_bound_it->first, 3);
  double lower_workplane_height = setPrecision(lower_bound_it->first, 3);
  LimitMap upper_workplane = upper_bound_it->second;
  LimitMap lower_workplane = lower_bound_it->second;

  // Calculate interpolation of radii for target workplane height between existing workspace planes
  double i = (target_workplane_height - lower_workplane_height) / (upper_workplane_height - lower_workplane_height);
  double stance_span_modifier = walker_->getParameters().stance_span_modifier.current_value;
  bool positive_y_axis = (Eigen::Vector3d::UnitY().dot(identity_tip_pose_.position_) > 0.0);
  int bearing = (positive_y_axis ^ (stance_span_modifier > 0.0)) ? 270 : 90;
  stance_span_modifier *= (positive_y_axis ? 1.0 : -1.0);
  double radius = 0.0;
  if (workspace.size() == 1)
  {
    radius = workspace.at(0.0).at(bearing);
  }
  else
  {
    radius = lower_workplane.at(bearing) * (1.0 - i) + upper_workplane.at(bearing) * i;
  }
  return Eigen::Vector3d(0.0, radius * stance_span_modifier, 0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updateDefaultTipPosition(void)
{
  // Generate new default tip pose from external request transformed based on robot movement since request
  Pose new_default_tip_pose;
  if (external_default_.defined_)
  {
    new_default_tip_pose = external_default_.pose_.removePose(external_default_.transform_);
  }
  // Generate new default tip pose based on tip position at end of swing period
  else
  {
    // Modify identity tip positions according to desired stance span modifier parameter
    Eigen::Vector3d identity_tip_position = identity_tip_pose_.position_;
    identity_tip_position += calculateStanceSpanChange();

    // Update default tip position as projection of tip position at beginning of STANCE period onto walk plane
    identity_tip_position = leg_->getDefaultBodyPose().transformVector(identity_tip_position);
    Eigen::Vector3d identity_to_stance_origin = stance_origin_tip_position_ - identity_tip_position;
    Eigen::Vector3d projection_to_walk_plane = getProjection(identity_to_stance_origin, walk_plane_normal_);
    new_default_tip_pose = Pose(identity_tip_position + projection_to_walk_plane, UNDEFINED_ROTATION);
  }

  // Update default tip pose and set walkspace to be regenerated if required
  double default_tip_position_delta = (default_tip_pose_.position_ - new_default_tip_pose.position_).norm();
  ROS_ASSERT(new_default_tip_pose.isValid());
  default_tip_pose_ = new_default_tip_pose;
  if (default_tip_position_delta > IK_TOLERANCE)
  {
    //walker_->setRegenerateWalkspace();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updateTipPosition(void)
{
  bool rough_terrain_mode = walker_->getParameters().rough_terrain_mode.data;
  bool force_normal_touchdown = walker_->getParameters().force_normal_touchdown.data;
  double time_delta = walker_->getTimeDelta();
  StepCycle step = walker_->getStepCycle();

  bool standard_stance_period = (step_state_ == SWING || completed_first_step_);
  int modified_stance_start = standard_stance_period ? step.stance_start_ : phase_offset_;
  int modified_stance_period = mod(step.stance_end_ - modified_stance_start, step.period_);
  if (step.stance_end_ == modified_stance_start)
  {
    modified_stance_period = step.period_;
  }
  ROS_ASSERT(modified_stance_period != 0);

  // Calculates number of iterations for ENTIRE swing period and time delta used for EACH bezier curve time input
  int swing_iterations = (double(step.swing_period_) / step.period_) / (step.frequency_ * time_delta);
  swing_iterations = roundToEvenInt(swing_iterations); // Must be even
  swing_delta_t_ = 1.0 / (swing_iterations / 2.0);     // 1 sec divided by number of iterations for each bezier curve

  // Calculates number of iterations for stance period and time delta used for bezier curve time input
  int stance_iterations = (double(modified_stance_period) / step.period_) / (step.frequency_ * time_delta);
  stance_delta_t_ = 1.0 / stance_iterations; // 1 second divided by number of iterations

  // Generate default target
  target_tip_pose_.position_ = default_tip_pose_.position_ + 0.5 * stride_vector_;

  // Swing Period
  if (step_state_ == SWING)
  {
    updateStride();
    int iteration = phase_ - step.swing_start_ + 1;
    bool first_half = iteration <= swing_iterations / 2;

    // Save initial tip position/velocity
    if (iteration == 1)
    {
      swing_origin_tip_position_ = current_tip_pose_.position_;
      swing_origin_tip_velocity_ = current_tip_velocity_;
      if (rough_terrain_mode)
      {
        updateDefaultTipPosition();
      }
    }

    // Update target to externally defined position OR update default to meet step surface
    if (rough_terrain_mode)
    {
      // Update target tip pose to externally requested target tip pose transformed based on movement since request
      if (external_target_.defined_)
      {
        target_tip_pose_ = external_target_.pose_.removePose(external_target_.transform_);
        swing_clearance_ = swing_clearance_.normalized() * external_target_.swing_clearance_;
        // Add lead to compensate for moving target
        if (external_target_.frame_id_ == "odom_ideal")
        {
          double time_to_swing_end = (swing_iterations - iteration) * time_delta;
          Eigen::Vector3d target_lead = walker_->calculateOdometry(time_to_swing_end).position_;
          target_tip_pose_.position_ -= target_lead;
        }
      }
      // Update default target to meet step surface either proactively or reactively
      else if (touchdown_detection_)
      {
        // Shift target to step surface according to data from tip state (PROACTIVE)
        // TODO Move proactive target shifting to seperate node and use external target API
        Pose step_plane_pose = leg_->getStepPlanePose();
        if (step_plane_pose != Pose::Undefined())
        {
          // Calculate target tip position based on step plane estimate
          Eigen::Vector3d step_plane_position = step_plane_pose.position_ - leg_->getCurrentTipPose().position_;
          Eigen::Vector3d target_tip_position = current_tip_pose_.position_ + step_plane_position;

          // Correct target by projecting vector along walk plane normal at default target tip position
          Eigen::Vector3d difference = target_tip_position - target_tip_pose_.position_;
          target_tip_pose_.position_ += getProjection(difference, walk_plane_normal_);
        }
        // Shift target toward step surface by defined distance and rely on step surface contact detection (REACTIVE)
        else
        {
          target_tip_pose_.position_ -= walker_->getStepDepth() * Eigen::Vector3d::UnitZ();
        }
      }
      else
      {
        ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\n[SHC] Rough terrain mode is enabled but SHC is not receiving"
                                           "any tip state messages used for touchdown detection.\n");
      }
    }

    // Generate swing control nodes (once at beginning of 1st half and continuously for 2nd half)
    bool ground_contact = (leg_->getStepPlanePose() != Pose::Undefined() && rough_terrain_mode);
    generatePrimarySwingControlNodes();
    generateSecondarySwingControlNodes(!first_half && ground_contact);
    // Adjust control nodes to force touchdown normal to walk plane
    if (force_normal_touchdown && !ground_contact)
    {
      forceNormalTouchdown();
    }

    Eigen::Vector3d delta_pos(0, 0, 0);
    double time_input = 0;
    if (first_half)
    {
      time_input = swing_delta_t_ * iteration;
      delta_pos = swing_delta_t_ * quarticBezierDot(swing_1_nodes_, time_input);
    }
    else
    {
      time_input = swing_delta_t_ * (iteration - swing_iterations / 2);
      delta_pos = swing_delta_t_ * quarticBezierDot(swing_2_nodes_, time_input);
    }

    ROS_ASSERT(time_input <= 1.0);
    ROS_ASSERT(delta_pos.norm() < UNASSIGNED_VALUE);
    current_tip_pose_.position_ += delta_pos;
    current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    ROS_DEBUG_COND(walker_->getParameters().debug_swing_trajectory.data && leg_->getIDNumber() == 0,
                   "SWING TRAJECTORY_DEBUG - ITERATION: %d\t\t"
                   "TIME: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "POS: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   iteration, setPrecision(time_input, 3),
                   swing_origin_tip_position_[0], swing_origin_tip_position_[1], swing_origin_tip_position_[2],
                   current_tip_pose_.position_[0], current_tip_pose_.position_[1], current_tip_pose_.position_[2],
                   target_tip_pose_.position_[0], target_tip_pose_.position_[1], target_tip_pose_.position_[2]);
  }
  // Stance period
  else if (step_state_ == STANCE || step_state_ == FORCE_STANCE)
  {
    updateStride();

    int iteration = mod(phase_ + (step.period_ - modified_stance_start), step.period_) + 1;

    // Save initial tip position at beginning of stance
    if (iteration == 1)
    {
      stance_origin_tip_position_ = current_tip_pose_.position_;
      external_target_.defined_ = false; // Reset external target after every swing period
      if (rough_terrain_mode)
      {
        updateDefaultTipPosition();
      }
    }

    // Scales stride vector according to stance period specifically for STARTING state of walker
    double stride_scaler = double(modified_stance_period) / (mod(step.stance_end_ - step.stance_start_, step.period_));
    generateStanceControlNodes(stride_scaler);

    // Uses derivative of bezier curve to ensure correct velocity along ground, this means the position may not
    // reach the target but this is less important than ensuring correct velocity according to stride vector
    double time_input = iteration * stance_delta_t_;
    Eigen::Vector3d delta_pos = stance_delta_t_ * quarticBezierDot(stance_nodes_, time_input);
    ROS_ASSERT(delta_pos.norm() < UNASSIGNED_VALUE);
    current_tip_pose_.position_ += delta_pos;
    current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    ROS_DEBUG_COND(walker_->getParameters().debug_stance_trajectory.data && leg_->getIDNumber() == 0,
                   "STANCE TRAJECTORY_DEBUG - ITERATION: %d\t\t"
                   "TIME: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "POS: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   iteration, setPrecision(time_input, 3),
                   stance_origin_tip_position_[0], stance_origin_tip_position_[1], stance_origin_tip_position_[2],
                   current_tip_pose_.position_[0], current_tip_pose_.position_[1], current_tip_pose_.position_[2],
                   stance_nodes_[4][0], stance_nodes_[4][1], stance_nodes_[4][2]);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::updateTipRotation(void)
{
  if (leg_->getJointCount() > 3 && (stance_progress_ >= 0.0 || swing_progress_ >= 0.5))
  {
    // Set target tip rotation to align with gravity if parameter is set and target currently undefined
    if (walker_->getParameters().gravity_aligned_tips.data && target_tip_pose_.rotation_.isApprox(UNDEFINED_ROTATION))
    {
      // WALK PLANE NORMAL ALIGNMENT TBD
      //target_tip_pose_.rotation_ = correctRotation(target_tip_rotation, origin_tip_pose_.rotation_);

      // GRAVITY ALIGNMENT
      target_tip_pose_.rotation_ =
          Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), walker_->estimateGravity());
    }

    // Target is undefined so set current tip rotation to undefined
    if (target_tip_pose_.rotation_.isApprox(UNDEFINED_ROTATION))
    {
      current_tip_pose_.rotation_ = target_tip_pose_.rotation_;
    }
    // Target is defined so transition current tip rotation to target during back half of swing period
    else
    {
      current_tip_pose_.rotation_ = correctRotation(target_tip_pose_.rotation_, origin_tip_pose_.rotation_);
      if (swing_progress_ >= 0.5)
      {
        double c = smoothStep(std::min(1.0, 2.0 * (swing_progress_ - 0.5))); // Control input (0.0 -> 1.0)
        Eigen::Vector3d origin_tip_direction = origin_tip_pose_.rotation_._transformVector(Eigen::Vector3d::UnitX());
        Eigen::Vector3d target_tip_direction = target_tip_pose_.rotation_._transformVector(Eigen::Vector3d::UnitX());
        Eigen::Vector3d new_tip_direction = interpolate(origin_tip_direction, target_tip_direction, c);
        Eigen::Quaterniond new_tip_rotation =
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), new_tip_direction.normalized());
        current_tip_pose_.rotation_ = correctRotation(new_tip_rotation, current_tip_pose_.rotation_);
      }
    }
  }
  else
  {
    origin_tip_pose_.rotation_ = leg_->getCurrentTipPose().rotation_;
    current_tip_pose_.rotation_ = UNDEFINED_ROTATION;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::generatePrimarySwingControlNodes(void)
{
  Eigen::Vector3d mid_tip_position = (swing_origin_tip_position_ + target_tip_pose_.position_) / 2.0;
  mid_tip_position[2] = std::max(swing_origin_tip_position_[2], target_tip_pose_.position_[2]);
  mid_tip_position += swing_clearance_;
  double mid_lateral_shift = walker_->getParameters().swing_width.current_value;
  bool positive_y_axis = (Eigen::Vector3d::UnitY().dot(identity_tip_pose_.position_) > 0.0);
  mid_tip_position[1] += positive_y_axis ? mid_lateral_shift : -mid_lateral_shift;
  Eigen::Vector3d stance_node_seperation =
      0.25 * swing_origin_tip_velocity_ * (walker_->getTimeDelta() / swing_delta_t_);

  // Control nodes for primary swing quartic bezier curves
  // Set for position continuity at transition between stance and primary swing curves (C0 Smoothness)
  swing_1_nodes_[0] = swing_origin_tip_position_;
  // Set for velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing_1_nodes_[1] = swing_origin_tip_position_ + stance_node_seperation;
  // Set for acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing_1_nodes_[2] = swing_origin_tip_position_ + 2.0 * stance_node_seperation;
  // Set for acceleration continuity at transition between swing curves (C2 Smoothness for symetric curves)
  swing_1_nodes_[3] = (mid_tip_position + swing_1_nodes_[2]) / 2.0;
  swing_1_nodes_[3][1] = mid_tip_position[1];
  swing_1_nodes_[3][2] = mid_tip_position[2];
  // Set to default tip position so max swing height and transition to 2nd swing curve occurs at default tip position
  swing_1_nodes_[4] = mid_tip_position;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::generateSecondarySwingControlNodes(const bool &ground_contact)
{
  Eigen::Vector3d final_tip_velocity = -stride_vector_ * (stance_delta_t_ / walker_->getTimeDelta());
  Eigen::Vector3d stance_node_seperation = 0.25 * final_tip_velocity * (walker_->getTimeDelta() / swing_delta_t_);

  // Control nodes for secondary swing quartic bezier curves
  // Set for position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing_2_nodes_[0] = swing_1_nodes_[4];
  // Set for velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_2_nodes_[1] = swing_1_nodes_[4] - (swing_1_nodes_[3] - swing_1_nodes_[4]);
  // Set for acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing_2_nodes_[2] = target_tip_pose_.position_ - 2.0 * stance_node_seperation;
  // Set for velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
  swing_2_nodes_[3] = target_tip_pose_.position_ - stance_node_seperation;
  // Set for position continuity at transition between secondary swing and stance curves (C0 Smoothness)
  swing_2_nodes_[4] = target_tip_pose_.position_;

  // Stops further movement of tip position in direction normal to walk plane
  if (ground_contact)
  {
    swing_2_nodes_[0] = current_tip_pose_.position_ + 0.0 * stance_node_seperation;
    swing_2_nodes_[1] = current_tip_pose_.position_ + 1.0 * stance_node_seperation;
    swing_2_nodes_[2] = current_tip_pose_.position_ + 2.0 * stance_node_seperation;
    swing_2_nodes_[3] = current_tip_pose_.position_ + 3.0 * stance_node_seperation;
    swing_2_nodes_[4] = current_tip_pose_.position_ + 4.0 * stance_node_seperation;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::generateStanceControlNodes(const double &stride_scaler)
{
  Eigen::Vector3d stance_node_seperation = -stride_vector_ * stride_scaler * 0.25;

  // Control nodes for stance quartic bezier curve
  // Set as initial tip position
  stance_nodes_[0] = stance_origin_tip_position_ + 0.0 * stance_node_seperation;
  // Set for constant velocity in stance period
  stance_nodes_[1] = stance_origin_tip_position_ + 1.0 * stance_node_seperation;
  // Set for constant velocity in stance period
  stance_nodes_[2] = stance_origin_tip_position_ + 2.0 * stance_node_seperation;
  // Set for constant velocity in stance period
  stance_nodes_[3] = stance_origin_tip_position_ + 3.0 * stance_node_seperation;
  // Set as target tip position
  stance_nodes_[4] = stance_origin_tip_position_ + 4.0 * stance_node_seperation;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LegStepper::forceNormalTouchdown(void)
{
  Eigen::Vector3d final_tip_velocity = -stride_vector_ * (stance_delta_t_ / walker_->getTimeDelta());
  Eigen::Vector3d stance_node_seperation = 0.25 * final_tip_velocity * (walker_->getTimeDelta() / swing_delta_t_);

  Eigen::Vector3d bezier_target = target_tip_pose_.position_;
  Eigen::Vector3d bezier_origin = target_tip_pose_.position_ - 4.0 * stance_node_seperation;
  bezier_origin[2] = std::max(swing_origin_tip_position_[2], target_tip_pose_.position_[2]);
  bezier_origin += swing_clearance_;

  swing_1_nodes_[4] = bezier_origin;
  swing_2_nodes_[0] = bezier_origin;
  swing_2_nodes_[2] = bezier_target - 2.0 * stance_node_seperation;
  swing_1_nodes_[3] = swing_2_nodes_[0] - (swing_2_nodes_[2] - bezier_origin) / 2.0;
  swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[2] - bezier_origin) / 2.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
