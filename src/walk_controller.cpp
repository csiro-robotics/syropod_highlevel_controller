/*******************************************************************************************************************//**
 *  @file    walk_controller.cpp
 *  @brief   Handles control of Syropod walking.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    October 2017
 *  @version 0.5.6
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
 **********************************************************************************************************************/

#include "../include/syropod_highlevel_controller/walk_controller.h"
#include "../include/syropod_highlevel_controller/pose_controller.h"
#include "../include/syropod_highlevel_controller/debug_visualiser.h"

/*******************************************************************************************************************//**
 * Constructor for the walk controller.
 * @param[in] model A pointer to the robot model.
 * @param[in] params A copy of the parameter data structure
 * @param[in] debug_visualiser A pointer to the debug visualisation object
***********************************************************************************************************************/
WalkController::WalkController(shared_ptr<Model> model, const Parameters& params,
                               shared_ptr<DebugVisualiser> debug_visualiser)
  : model_(model)
  , params_(params)
  , debug_visualiser_(debug_visualiser)
{
}

/*******************************************************************************************************************//**
 * Initialises walk controller by setting desired default walking stance tip positions from parameters and creating
 * LegStepper objects for each leg. Also populates workspace map with initial values by finding bisector line between
 * adjacent leg tip positions.
***********************************************************************************************************************/
void WalkController::init(void)
{
  time_delta_ = params_.time_delta.data;
  step_clearance_ = params_.step_clearance.current_value;
  body_clearance_ = params_.body_clearance.current_value;
  bool debug = params_.debug_workspace_calc.data;
  walk_state_ = STOPPED;

  // Set default stance tip positions from parameters
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    double x_position = params_.leg_stance_positions[leg->getIDNumber()].data.at("x");
    double y_position = params_.leg_stance_positions[leg->getIDNumber()].data.at("y");
    Vector3d identity_tip_position(x_position, y_position, -body_clearance_);
    leg->setLegStepper(allocate_shared<LegStepper>(aligned_allocator<LegStepper>(), shared_from_this(), leg, identity_tip_position));
  }

  // Populate workspace map using leg tip position overlaps
  workspace_map_.clear();
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<Leg> adjacent_leg_1 = model_->getLegByIDNumber(mod(leg->getIDNumber() + 1, model_->getLegCount()));
    shared_ptr<Leg> adjacent_leg_2 = model_->getLegByIDNumber(mod(leg->getIDNumber() - 1, model_->getLegCount()));

    Vector3d identity_tip_position = leg->getLegStepper()->getDefaultTipPosition();
    Vector3d adjacent_1_tip_position = adjacent_leg_1->getLegStepper()->getDefaultTipPosition();
    Vector3d adjacent_2_tip_position = adjacent_leg_2->getLegStepper()->getDefaultTipPosition();

    double distance_to_adjacent_leg_1 = Vector3d(identity_tip_position - adjacent_1_tip_position).norm() / 2.0;
    double distance_to_adjacent_leg_2 = Vector3d(identity_tip_position - adjacent_2_tip_position).norm() / 2.0;

    int bearing_to_adjacent_leg_1 = radiansToDegrees(atan2(adjacent_1_tip_position[1] - identity_tip_position[1],
                                    adjacent_1_tip_position[0] - identity_tip_position[0]));
    int bearing_to_adjacent_leg_2 = radiansToDegrees(atan2(adjacent_2_tip_position[1] - identity_tip_position[1],
                                    adjacent_2_tip_position[0] - identity_tip_position[0]));

    ROS_DEBUG_COND(debug, "\nLeg %s has adjacent legs:\n"
                   "\tLeg %s at %d bearing & %f distance.\n"
                   "\tLeg %s at %d bearing & %f distance.\n",
                   leg->getIDName().c_str(),
                   adjacent_leg_1->getIDName().c_str(), bearing_to_adjacent_leg_1, distance_to_adjacent_leg_1,
                   adjacent_leg_2->getIDName().c_str(), bearing_to_adjacent_leg_2, distance_to_adjacent_leg_2);

    //Populate workspace map if empty
    for (int bearing = 0; bearing < 360; bearing += BEARING_STEP)
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

      double min_distance = min(distance_to_overlap_1, distance_to_overlap_2);
      if (workspace_map_.find(bearing) != workspace_map_.end() && min_distance < workspace_map_[bearing])
      {
        workspace_map_[bearing] = min_distance;
        ROS_DEBUG_COND(debug, "Workspace: Bearing %d has modified min distance %f.", bearing, min_distance);
      }
      else
      {
        workspace_map_.insert(map<int, double>::value_type(bearing, min_distance));
        ROS_DEBUG_COND(debug, "Workspace: Bearing %d has new min distance of %f.", bearing, min_distance);
      }
    }
  }

  // Stance radius based around front right leg to ensure positive values
  shared_ptr<Leg> reference_leg = model_->getLegByIDNumber(0);
  shared_ptr<LegStepper> reference_leg_stepper = reference_leg->getLegStepper();
  double x_position = reference_leg_stepper->getDefaultTipPosition()[0];
  double y_position = reference_leg_stepper->getDefaultTipPosition()[1];
  stance_radius_ = Vector2d(x_position, y_position).norm();

  // Init velocity input variables
  desired_linear_velocity_ = Vector2d(0, 0);
  desired_angular_velocity_ = 0;

  // Init gait parameters
  setGaitParams();
}

/*******************************************************************************************************************//**
 * Generates universal workspace (map of radii for range of search bearings) for all legs by having each leg search
 * for joint limitations through bearings ranging from zero to 360 degrees. Workspace may be asymetrical, symetrical
 * (all opposite bearing pairs having equal distance) or circular (all search bearings having equal distance).
 * @todo Parameterise symmetric/circular workspace constraint flags.
 * @todo Parameterise BEARING_STEP and SEARCH_VELOCITY constants
***********************************************************************************************************************/
void WalkController::generateWorkspace(void)
{
  bool debug = params_.debug_workspace_calc.data;
  bool symmetric_workspace = true; //TODO
  bool circular_workspace = false; //TODO

  int workspace_generation_start = clock();
  double absolute_min = UNASSIGNED_VALUE;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

    // Create copy of leg at default state
    sensor_msgs::JointState default_joint_states;
    leg->generateDesiredJointStateMsg(&default_joint_states);

    // Iterate through search bearings
    int opposite_bearing = 0;
    for (int search_bearing = 0; search_bearing < 360; search_bearing += BEARING_STEP) // TODO
    {
      if (debug)
      {
        double progress = 100.0 * (search_bearing + (leg->getIDNumber() * 360.0)) / (360.0 * model_->getLegCount());
        ROS_DEBUG_THROTTLE(THROTTLE_PERIOD, "\nCalculating workspace (%d%%) . . .\n", roundToInt(progress));
      }

      double current_min = workspace_map_.at(search_bearing);
      if (circular_workspace)
      {
        current_min = min(current_min, absolute_min);
      }
      else if (symmetric_workspace)
      {
        opposite_bearing = mod(search_bearing + 180, 360);
        current_min = min(current_min, workspace_map_.at(opposite_bearing));
      }

      // Calculate target tip position along search bearing
      Vector3d origin_tip_position = leg->getCurrentTipPosition();
      Vector3d target_tip_position = origin_tip_position;
      target_tip_position[0] += current_min * cos(degreesToRadians(search_bearing));
      target_tip_position[1] += current_min * sin(degreesToRadians(search_bearing));

      // Move tip position linearly along search bearing in search of workspace limit defined by joint limits.
      bool within_limits = true;
      double distance_from_default = 0.0;
      int iteration = 0;
      while (within_limits && distance_from_default < current_min &&
             iteration <= WORKSPACE_GENERATION_MAX_ITERATIONS)
      {
        iteration++;
        double i = double(iteration) / WORKSPACE_GENERATION_MAX_ITERATIONS; // Interpolation control variable
        Vector3d desired_tip_position = origin_tip_position * (1.0-i) + target_tip_position * i; //Linearly interpolate
        leg->setDesiredTipPosition(desired_tip_position, false);
        
        within_limits = (leg->applyIK(true, true) != 0.0);
        distance_from_default = Vector3d(leg->getCurrentTipPosition() - leg_stepper->getDefaultTipPosition()).norm();

        // Display robot model whilst performing workspace limitation search
        if (debug && params_.debug_rviz.data && params_.debug_rviz_static_display.data)
        {
          debug_visualiser_->updatePose(Vector2d(0, 0), 0, body_clearance_);
          debug_visualiser_->generateRobotModel(model_);
          ros::spinOnce();
        }
      }

      // Save minimum distance from default tip position (of each leg) as workspace radius for defined search bearing.
      workspace_map_[search_bearing] = min(current_min, distance_from_default);
      if (symmetric_workspace)
      {
        workspace_map_[opposite_bearing] = min(current_min, distance_from_default);
      }
      absolute_min = min(workspace_map_[search_bearing], absolute_min);
      leg->reInit(default_joint_states); // Reinitialise leg state to default tip position
      leg->setDesiredTipVelocity(Vector3d(0, 0, 0));

      // Display workspace generation
      if (params_.debug_rviz.data && params_.debug_rviz_static_display.data)
      {
        for (int i = 0; i <= leg->getIDNumber(); ++i)
        {
          debug_visualiser_->generateWorkspace(model_->getLegByIDNumber(i), workspace_map_);
        }
        ros::spinOnce();
      }
    }
  }

  // Send debugging info to rosconsole
  if (debug)
  {
    string debug_string = "\nWorkspace:\n";
    map<int, double>::iterator it;
    for (it = workspace_map_.begin(); it != workspace_map_.end(); ++it)
    {
      debug_string += stringFormat("\tBearing: %d\t\tDistance: %f\n", it->first, it->second);
    }
    double elapsed_total_time = (clock() - workspace_generation_start) / double(CLOCKS_PER_SEC);
    debug_string += stringFormat("\nWorkspace calculations completed in %f seconds.\n", elapsed_total_time);
    ROS_DEBUG("%s", debug_string.c_str());
  }

  workspace_generated_ = true;
  calculateMaxSpeed();
}

/*******************************************************************************************************************//**
 * Calculate maximum linear and angular speed/acceleration for each workspace radius in workspace map. These calculated
 * values will accomodate overshoot of tip outside defined workspace whilst body accelerates, effectively scaling usable
 * workspace.
***********************************************************************************************************************/
void WalkController::calculateMaxSpeed(void)
{
  int base_phase_length = params_.stance_phase.data + params_.swing_phase.data;
  int normaliser = phase_length_ / base_phase_length;
  int base_phase_offset = int(params_.phase_offset.data * normaliser);

  // Set phase offset and check if leg starts in swing phase (i.e. forced to stance for the 1st step cycle)
  // If so find this max 'stance extension' phase length which is used in acceleration calculations
  int max_stance_extension = 0;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    ROS_ASSERT(params_.offset_multiplier.data.count(leg->getIDName()));
    int multiplier = params_.offset_multiplier.data.at(leg->getIDName());
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    int phase_offset = (base_phase_offset * multiplier) % phase_length_;
    leg_stepper->setPhaseOffset(phase_offset);
    if (phase_offset > swing_start_ && phase_offset < swing_end_)  // SWING STATE
    {
      max_stance_extension = max(max_stance_extension, swing_end_ - phase_offset);
    }
  }

  // Set max stride (i.e. max body velocity) to occur at end of 1st swing of leg with maximum stance length extension
  double time_to_max_stride = (max_stance_extension + stance_length_ + swing_length_) * time_delta_;

  max_linear_speed_.clear();
  max_linear_acceleration_.clear();
  max_angular_speed_.clear();
  max_angular_acceleration_.clear();

  // Calculate initial max speed and acceleration of body
  map<int, double>::iterator it;
  for (it = workspace_map_.begin(); it != workspace_map_.end(); ++it)
  {
    double workspace_radius = it->second;
    double on_ground_ratio = double(stance_length_) / double(phase_length_);
    double max_speed = (workspace_radius * 2.0) / (on_ground_ratio / step_frequency_);
    double max_acceleration = max_speed / time_to_max_stride;

    // Calculates max overshoot of tip (in stance phase) outside workspace
    double stance_overshoot = 0;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      shared_ptr<Leg> leg = leg_it_->second;
      shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      // All referenced swings are the LAST swing phase BEFORE the max velocity (stride length) is reached
      double phase_offset = leg_stepper->getPhaseOffset();
      double t = phase_offset * time_delta_; // Time between swing end and max velocity being reached
      double time_to_swing_end = time_to_max_stride - t;
      double v0 = max_acceleration * time_to_swing_end; // Tip velocity at time of swing end
      double stride_length = v0 * (on_ground_ratio / step_frequency_);
      double d0 = -stride_length / 2.0; // Distance to default tip position at time of swing end
      double d1 = d0 + v0 * t + 0.5 * max_acceleration * sqr(t); // Distance from default tip position at max velocity
      double d2 = max_speed * (stance_length_ * time_delta_ - t); // Distance from default tip position at end of stance
      stance_overshoot = max(stance_overshoot, d1 + d2 - workspace_radius); // Max overshoot past workspace limitations
    }

    // Scale workspace to accomodate stance overshoot
    double scaled_workspace_radius = (workspace_radius / (workspace_radius + stance_overshoot)) * workspace_radius;

    // Further scale workspace to accomodate normal max swing overshoot
    double swing_overshoot = 0.5 * max_speed * swing_length_ / (2.0 * phase_length_ * step_frequency_); // From swing node 1
    scaled_workspace_radius *= (scaled_workspace_radius / (scaled_workspace_radius + swing_overshoot));

    // Distance: scaled_workspace_radius*2.0 (i.e. max stride length)
    // Time: on_ground_ratio*(1/step_frequency_) where step frequency is FULL step cycles/s)
    double max_linear_speed = (scaled_workspace_radius * 2.0) / (on_ground_ratio / step_frequency_);
    double max_linear_acceleration = max_linear_speed / time_to_max_stride;
    double max_angular_speed = max_linear_speed / stance_radius_;
    double max_angular_acceleration = max_angular_speed / time_to_max_stride;

    max_linear_speed_.insert(map<int, double>::value_type(it->first, max_linear_speed));
    max_linear_acceleration_.insert(map<int, double>::value_type(it->first, max_linear_acceleration));
    max_angular_speed_.insert(map<int, double>::value_type(it->first, max_angular_speed));
    max_angular_acceleration_.insert(map<int, double>::value_type(it->first, max_angular_acceleration));
  }
}

/*******************************************************************************************************************//**
 * Calculates walk controller walk cycle parameters, normalising base parameters according to step frequency.
***********************************************************************************************************************/
void WalkController::setGaitParams(void)
{
  stance_end_ = params_.stance_phase.data * 0.5;
  swing_start_ = stance_end_;
  swing_end_ = swing_start_ + params_.swing_phase.data;
  stance_start_ = swing_end_;

  // Normalises the step phase length to match the total number of iterations over a full step
  int base_phase_length = params_.stance_phase.data + params_.swing_phase.data;
  double swing_ratio = double(params_.swing_phase.data) / double(base_phase_length); // Modifies step frequency

  // Ensure phase length is even and divisible by base phase length and therefore gives whole even normaliser value
  double raw_phase_length = ((1.0 / params_.step_frequency.current_value) / time_delta_) / swing_ratio;
  phase_length_ = roundToEvenInt(raw_phase_length / base_phase_length) * base_phase_length;

  step_frequency_ = 1.0 / (phase_length_ * time_delta_);  // adjust step frequency to match corrected phase length
  int normaliser = phase_length_ / base_phase_length;
  stance_end_ *= normaliser;
  swing_start_ *= normaliser;
  swing_end_ *= normaliser;
  stance_start_ *= normaliser;

  stance_length_ = mod(stance_end_ - stance_start_, phase_length_);
  swing_length_ = swing_end_ - swing_start_;

  // Ensure stance and swing phase lengths are divisible by two
  ROS_ASSERT(stance_length_ % 2 == 0);
  ROS_ASSERT(swing_length_ % 2 == 0);
}

/*******************************************************************************************************************//**
 * Updates all legs in the walk cycle. Calculates stride vectors for all legs from robot body velocity inputs and calls
 * trajectory update functions for each leg to update individual tip positions. Also manages the overall walk state via
 * state machine and input velocities as well as the individual step state of each leg as they progress through stance
 * and swing states.
 * @params[in] linear_velocity_input An input for the desired linear velocity of the robot body in the x/y plane.
 * @params[in] angular_velocity_input An input for the desired angular velocity of the robot body about the z axis.
***********************************************************************************************************************/
void WalkController::updateWalk(const Vector2d& linear_velocity_input, const double& angular_velocity_input)
{
  double on_ground_ratio = double(stance_length_) / double(phase_length_);

  Vector2d new_linear_velocity;
  double new_angular_velocity;

  double max_linear_speed = UNASSIGNED_VALUE;
  double max_angular_speed = UNASSIGNED_VALUE;
  double max_linear_acceleration = UNASSIGNED_VALUE;
  double max_angular_acceleration = UNASSIGNED_VALUE;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    Vector3d tip_position = leg_stepper->getCurrentTipPosition();
    Vector2d rotation_normal = Vector2d(-tip_position[1], tip_position[0]);
    Vector2d stride_vector = linear_velocity_input + angular_velocity_input * rotation_normal;
    double bearing = radiansToDegrees(atan2(stride_vector[1], stride_vector[0]));
    int lower_bound = workspace_map_.lower_bound(roundToInt(bearing))->first;
    int upper_bound = mod(lower_bound - BEARING_STEP, 360);
    int closest_bearing = (abs(lower_bound - bearing) < abs(upper_bound - bearing)) ? lower_bound : upper_bound;
    max_linear_speed = min(max_linear_speed, max_linear_speed_.at(closest_bearing));
    max_angular_speed = min(max_angular_speed, max_angular_speed_.at(closest_bearing));
    max_linear_acceleration = min(max_linear_acceleration, max_linear_acceleration_.at(closest_bearing));
    max_angular_acceleration = min(max_angular_acceleration, max_angular_acceleration_.at(closest_bearing));
  }

  // Calculate desired angular/linear velocities according to input mode and max limits
  if (walk_state_ != STOPPING)
  {
    if (params_.velocity_input_mode.data == "throttle")
    {
      new_linear_velocity = clamped(linear_velocity_input, 1.0) * max_linear_speed;  // Forces input between -1.0/1.0
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
    new_linear_velocity = Vector2d(0.0, 0.0);
    new_angular_velocity = 0.0;
  }

  // Update linear velocity according to acceleration limits
  Vector2d linear_acceleration = new_linear_velocity - desired_linear_velocity_;
  if (linear_acceleration.norm() > 0.0)
  {
    desired_linear_velocity_ +=
      linear_acceleration * min(1.0, max_linear_acceleration  * time_delta_ / linear_acceleration.norm());
  }

  // Update angular velocity according to acceleration limits
  double angular_acceleration = new_angular_velocity - desired_angular_velocity_;
  if (abs(angular_acceleration) > 0.0)
  {
    desired_angular_velocity_ +=
      angular_acceleration * min(1.0, max_angular_acceleration * time_delta_ / abs(angular_acceleration));
  }

  bool has_velocity_command = linear_velocity_input.norm() || angular_velocity_input;

  // Check that all legs are in WALKING state
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    if (has_velocity_command && leg->getLegState() != WALKING)
    {
      has_velocity_command = false;
      if (angular_velocity_input == 0)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD,
                          "\nUnable to walk whilst manually manipulating legs, ensure each leg is in walking state.\n");
      }
    }
  }

  // State transitions for Walk State Machine
  // State transition: STOPPED->STARTING
  int leg_count = model_->getLegCount();
  if (walk_state_ == STOPPED && has_velocity_command)
  {
    walk_state_ = STARTING;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      shared_ptr<Leg> leg = leg_it_->second;
      shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      leg_stepper->setPhase(leg_stepper->getPhaseOffset() - 1);
      leg_stepper->setAtCorrectPhase(false);
      leg_stepper->setCompletedFirstStep(false);
      leg_stepper->setStepState(STANCE);
    }
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
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    leg_stepper->iteratePhase();

    // Walk State Machine
    if (walk_state_ == STARTING)
    {
      // Check if all legs have completed one step
      if (legs_at_correct_phase_ == leg_count)
      {
        if (leg_stepper->getPhase() == swing_end_ && !leg_stepper->hasCompletedFirstStep())
        {
          leg_stepper->setCompletedFirstStep(true);
          legs_completed_first_step_++;
        }
      }
      // Force any leg state into STANCE if it starts offset in a mid-swing state but has not finished swing end
      if (!leg_stepper->isAtCorrectPhase())
      {
        if (leg_stepper->getPhaseOffset() > swing_start_ && leg_stepper->getPhaseOffset() < swing_end_ && // SWING STATE
            leg_stepper->getPhase() != swing_end_)
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
      // All legs must be at default tip positions after ending a swing before called 'at correct phase'
      bool zero_body_velocity = leg_stepper->getStrideVector().norm() == 0;
      Vector3d diff = leg_stepper->getCurrentTipPosition() - leg_stepper->getDefaultTipPosition();
      bool at_default_tip_position = abs(diff[0]) < TIP_TOLERANCE &&
                                     abs(diff[1]) < TIP_TOLERANCE &&
                                     abs(diff[2]) < TIP_TOLERANCE;
      if (zero_body_velocity && !leg_stepper->isAtCorrectPhase() &&
          leg_stepper->getPhase() == swing_end_ && at_default_tip_position)
      {
        leg_stepper->setStepState(FORCE_STOP);
        leg_stepper->setAtCorrectPhase(true);
        legs_at_correct_phase_++;
      }
    }
    else if (walk_state_ == STOPPED)
    {
      leg_stepper->setStepState(FORCE_STOP);
    }

    // Step State Machine
    int phase = leg_stepper->getPhase();
    StepState step_state = leg_stepper->getStepState();
    if (step_state == FORCE_STANCE)
    {
      leg_stepper->setStepState(STANCE); // Force STANCE for STARTING walk state
    }
    else if (step_state == FORCE_STOP)
    {
      leg_stepper->setStepState(FORCE_STOP); // Force STOP for STOPPING walk state
    }
    else if (phase >= swing_start_ && phase < swing_end_)
    {
      leg_stepper->setStepState(SWING);
    }
    else if (phase < stance_end_ || phase >= stance_start_)
    {
      leg_stepper->setStepState(STANCE);
    }

    // Update tip positions
    Vector3d tip_position = leg_stepper->getCurrentTipPosition();
    Vector2d rotation_normal = Vector2d(-tip_position[1], tip_position[0]);
    Vector2d stride_vector = desired_linear_velocity_ + desired_angular_velocity_ * rotation_normal;
    stride_vector *= (on_ground_ratio / step_frequency_);
    leg_stepper->updateStride(stride_vector);
    if (leg->getLegState() == WALKING && walk_state_ != STOPPED)
    {
      leg_stepper->updatePosition();  // updates current tip position through step cycle
    }
  }
}

/*******************************************************************************************************************//**
 * Updates the tip position for legs in the manual state from tip velocity inputs. Two modes are available: joint
 * control allows manipulation of joint positions directly but only works for 3DOF legs; tip control allows
 * manipulation of the tip in cartesian space in the robot frame.
 * @params[in] primary_leg_selection_ID The designation of a leg selected (in the primary role) for manipulation.
 * @params[in] primary_tip_velocity_input The velocity input to move the 1st leg tip position in the robot frame.
 * @params[in] secondary_leg_selection_ID The designation of a leg selected (in the secondary role) for manipulation.
 * @params[in] secondary_tip_velocity_input The velocity input to move the 2nd leg tip position in the robot frame.
***********************************************************************************************************************/
void WalkController::updateManual(const int& primary_leg_selection_ID, const Vector3d& primary_tip_velocity_input,
                                  const int& secondary_leg_selection_ID, const Vector3d& secondary_tip_velocity_input)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    if (leg->getLegState() == MANUAL)
    {
      Vector3d tip_velocity_input;
      int selected_leg_ID = leg->getIDNumber();
      if (selected_leg_ID == primary_leg_selection_ID)
      {
        tip_velocity_input = primary_tip_velocity_input;
      }
      else if (selected_leg_ID == secondary_leg_selection_ID)
      {
        tip_velocity_input = secondary_tip_velocity_input;
      }

      // Joint control works only for 3DOF legs as velocity inputs for x/y/z axes mapped to positions for joints 1/2/3
      if (params_.leg_manipulation_mode.data == "joint_control" && leg->getJointCount() == 3)
      {
        double coxa_joint_velocity = tip_velocity_input[0] * params_.max_rotation_velocity.data * time_delta_;
        double femur_joint_velocity = tip_velocity_input[1] * params_.max_rotation_velocity.data * time_delta_;
        double tibia_joint_velocity = tip_velocity_input[2] * params_.max_rotation_velocity.data * time_delta_;
        double coxa_joint_position = leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->desired_position_;
        double femur_joint_position = leg->getJointByIDName(leg->getIDName() + "_femur_joint")->desired_position_;
        double tibia_joint_position = leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->desired_position_;
        leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->prev_desired_position_ = coxa_joint_position;
        leg->getJointByIDName(leg->getIDName() + "_femur_joint")->prev_desired_position_ = femur_joint_position;
        leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->prev_desired_position_ = tibia_joint_position;
        leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->desired_position_ += coxa_joint_velocity;
        leg->getJointByIDName(leg->getIDName() + "_femur_joint")->desired_position_ += femur_joint_velocity;
        leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->desired_position_ += tibia_joint_velocity;
        Vector3d new_tip_position = leg->applyFK(false);
        leg_stepper->setCurrentTipPosition(new_tip_position);
      }
      else if (params_.leg_manipulation_mode.data == "tip_control")
      {
        Vector3d tip_position_change = tip_velocity_input * params_.max_translation_velocity.data * time_delta_;
        Vector3d new_tip_position = leg_stepper->getCurrentTipPosition() + tip_position_change;
        leg_stepper->setCurrentTipPosition(new_tip_position);
      }
    }
  }
}

/*******************************************************************************************************************//**
 * Leg stepper object constructor, initialises member variables from walk controller.
 * @param[in] walker A pointer to the walk controller.
 * @param[in] leg A pointer to the parent leg object.
 * @param[in] identity_tip_position The default walking stance tip position about which the step cycle is based.
***********************************************************************************************************************/
LegStepper::LegStepper(shared_ptr<WalkController> walker, shared_ptr<Leg> leg, const Vector3d& identity_tip_position)
  : walker_(walker)
  , leg_(leg)
  , default_tip_position_(identity_tip_position)
  , current_tip_position_(default_tip_position_)
{
  current_tip_velocity_ = Vector3d(0.0, 0.0, 0.0);
  stride_vector_ = Vector3d(0.0, 0.0, 0.0);
  swing_height_ = walker->getStepClearance();

  // Iterate through and initialise control nodes (5 control nodes for quartic (4th order) bezier curves)
  for (int i = 0; i < 5; ++i)
  {
    swing_1_nodes_[i] = Vector3d(0, 0, 0);
    swing_2_nodes_[i] = Vector3d(0, 0, 0);
    stance_nodes_[i] = Vector3d(0, 0, 0);
  }
};

/*******************************************************************************************************************//**
 * Iterates the step phase and updates the progress variables
***********************************************************************************************************************/
void LegStepper::iteratePhase(void)
{
  int phase_length = walker_->getPhaseLength();
  int swing_start = walker_->getSwingStart();
  int swing_end = walker_->getSwingEnd();
  int stance_start = walker_->getStanceStart();
  int stance_end = walker_->getStanceEnd();

  phase_ = (phase_ + 1) % (phase_length);

  // Calculate progress of stance/swing periods (0.0->1.0 or -1.0 if not in specific state)
  if (step_state_ == SWING)
  {
    swing_progress_ = double(phase_ - swing_start + 1) / double(swing_end - swing_start);
    swing_progress_ = clamped(swing_progress_, 0.0, 1.0);
    stance_progress_ = -1.0;
  }
  else if (step_state_ == STANCE)
  {
    stance_progress_ = double(mod(phase_ + (phase_length - stance_start), phase_length) + 1) /
                       double(mod(stance_end - stance_start, phase_length));
    stance_progress_ = clamped(stance_progress_, 0.0, 1.0);
    swing_progress_ = -1.0;
  }
}

/*******************************************************************************************************************//**
 * Updates position of tip using three quartic bezier curves to generate the tip trajectory. Calculates change in tip
 * position using two bezier curves for swing phase and one for stance phase. Each Bezier curve uses 5 control nodes
 * designed specifically to give a C2 smooth trajectory for the entire step cycle.
***********************************************************************************************************************/
void LegStepper::updatePosition(void)
{
  double step_frequency = walker_->getStepFrequency();
  double time_delta = walker_->getTimeDelta();
  int phase_length = walker_->getPhaseLength();
  int swing_start = walker_->getSwingStart();
  int swing_end = walker_->getSwingEnd();
  int swing_length = swing_end - swing_start;

  bool standard_stance_length = (step_state_ == SWING || completed_first_step_);
  int stance_start = standard_stance_length ? walker_->getStanceStart() : phase_offset_;
  int stance_end = walker_->getStanceEnd();
  int stance_length = mod(stance_end - stance_start, phase_length);

  // Calculates number of iterations for ENTIRE swing phase and time delta used for EACH bezier curve time input
  int swing_iterations = (double(swing_length) / phase_length) / (step_frequency * time_delta);
  swing_iterations = roundToEvenInt(swing_iterations); // Must be even
  swing_delta_t_ = 1.0 / (swing_iterations / 2.0); // 1 sec divided by number of iterations for each bezier curve

  // Calculates number of iterations for stance phase and time delta used for bezier curve time input
  int stance_iterations = (double(stance_length) / phase_length) / (step_frequency * time_delta);
  stance_delta_t_ = 1.0 / stance_iterations; // 1 second divided by number of iterations

  // Swing Phase
  if (step_state_ == SWING)
  {
    int iteration = phase_ - swing_start + 1;

    // Save initial tip position and generate swing control nodes at beginning of swing
    if (iteration == 1)
    {
      swing_origin_tip_position_ = current_tip_position_;
      generatePrimarySwingControlNodes(current_tip_velocity_);
    }

    double time_input_1 = 0;
    double time_input_2 = 0;

    // First swing curve - swing control nodes stay static since not updated each iteration
    if (iteration <= swing_iterations / 2)
    {
      time_input_1 = iteration * swing_delta_t_;
      Vector3d delta_pos = swing_delta_t_ * quarticBezierDot(swing_1_nodes_, time_input_1);
      current_tip_position_ += delta_pos;
      current_tip_velocity_ = delta_pos / walker_->getTimeDelta();
    }
    // Second swing curve - swing control nodes dynamic since updated each iteration to handle changing stride vector
    else
    {
      generateSecondarySwingControlNodes(-stride_vector_ * (stance_delta_t_ / walker_->getTimeDelta()));

      time_input_2 = (iteration - swing_iterations / 2) * swing_delta_t_;
      Vector3d delta_pos = swing_delta_t_ * quarticBezierDot(swing_2_nodes_, time_input_2);
      current_tip_position_ += delta_pos;
      current_tip_velocity_ = delta_pos / walker_->getTimeDelta();
    }

    ROS_DEBUG_COND(walker_->getParameters().debug_swing_trajectory.data && leg_->getIDNumber() == 0,
                   "SWING TRAJECTORY_DEBUG - ITERATION: %d\t\t"
                   "TIME: %f:%f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "POS: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   iteration, setPrecision(time_input_1, 3), setPrecision(time_input_2, 3),
                   swing_origin_tip_position_[0], swing_origin_tip_position_[1], swing_origin_tip_position_[2],
                   current_tip_position_[0], current_tip_position_[1], current_tip_position_[2],
                   swing_2_nodes_[4][0], swing_2_nodes_[4][1], swing_2_nodes_[4][2]);
  }
  // Stance phase
  else if (step_state_ == STANCE)
  {
    int iteration = mod(phase_ + (phase_length - stance_start), phase_length) + 1;

    // Save initial tip position at beginning of stance
    if (iteration == 1)
    {
      stance_origin_tip_position_ = current_tip_position_;
    }

    // Scales stride vector according to stance length specifically for STARTING state of walker
    double stride_scaler = double(stance_length) / (mod(stance_end - walker_->getStanceStart(), phase_length));
    generateStanceControlNodes(stride_vector_ * stride_scaler);

    // Uses derivative of bezier curve to ensure correct velocity along ground, this means the position may not
    // reach the target but this is less important than ensuring correct velocity according to stride vector
    double time_input = iteration * stance_delta_t_;
    Vector3d delta_pos = stance_delta_t_ * quarticBezierDot(stance_nodes_, time_input);
    current_tip_position_ += delta_pos;
    current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    ROS_DEBUG_COND(walker_->getParameters().debug_stance_trajectory.data && leg_->getIDNumber() == 0,
                   "STANCE TRAJECTORY_DEBUG - ITERATION: %d\t\t"
                   "TIME: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "POS: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   iteration, setPrecision(time_input, 3),
                   stance_origin_tip_position_[0], stance_origin_tip_position_[1], stance_origin_tip_position_[2],
                   current_tip_position_[0], current_tip_position_[1], current_tip_position_[2],
                   stance_nodes_[4][0], stance_nodes_[4][1], stance_nodes_[4][2]);
  }
}

/*******************************************************************************************************************//**
 * Generates control nodes for quartic bezier curve of 1st half of the swing tip trajectory calculation.
 * @param[in] initial_tip_velocity Tip velocity vector representing the expected velocity of the tip at the beginning
 * of swing trajectory generation.
***********************************************************************************************************************/
void LegStepper::generatePrimarySwingControlNodes(const Vector3d& initial_tip_velocity)
{
  // Control nodes for primary swing quartic bezier curves
  // Set for position continuity at transition between stance and primary swing curves (C0 Smoothness)
  swing_1_nodes_[0] = swing_origin_tip_position_;
  // Set to default tip position so max swing height and transition to 2nd swing curve occurs at default tip position
  swing_1_nodes_[4] = default_tip_position_;
  // Set for velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing_1_nodes_[1] = swing_1_nodes_[0] + 0.25 * initial_tip_velocity * (walker_->getTimeDelta() / swing_delta_t_);
  // Set for acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing_1_nodes_[2] = swing_1_nodes_[1] + (swing_1_nodes_[1] - swing_1_nodes_[0]);
  // Set for acceleration continuity at transition between swing curves (C2 Smoothness for symetric curves)
  swing_1_nodes_[3] = (swing_1_nodes_[2] + swing_1_nodes_[4]) / 2.0;

  // Control nodes for primary swing quartic bezier curves - vertical plane
  // Set to default tip position plus swing height so max swing height and transition to 2nd swing curve occurs here
  swing_1_nodes_[4][2] = default_tip_position_[2] + swing_height_;
  // Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_1_nodes_[3][2] = default_tip_position_[2] + swing_height_;
}

/*******************************************************************************************************************//**
 * Generates control nodes for quartic bezier curve of the 2nd half of the swing tip trajectory calculation.
 * @param[in] final_tip_velocity Tip velocity vector representing the expected velocity of the tip at the end
 * of swing trajectory generation.
***********************************************************************************************************************/
void LegStepper::generateSecondarySwingControlNodes(const Vector3d& final_tip_velocity)
{
  // Control nodes for secondary swing quartic bezier curves
  // Set for position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing_2_nodes_[0] = default_tip_position_;
  // Set for position continuity at transition between secondary swing and stance curves (C0 Smoothness)
  swing_2_nodes_[4] = default_tip_position_ + 0.5 * stride_vector_;
  // Set for velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
  swing_2_nodes_[3] = swing_2_nodes_[4] - 0.25 * final_tip_velocity * (walker_->getTimeDelta() / swing_delta_t_);
  // Set for acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing_2_nodes_[2] = swing_2_nodes_[3] + (swing_2_nodes_[3] - swing_2_nodes_[4]);
  // Set for velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[0] - swing_1_nodes_[3]);

  // Control nodes for secondary swing quartic bezier curves - vertical plane
  // Set for vertical position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing_2_nodes_[0][2] = default_tip_position_[2] + swing_height_;
  // Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_2_nodes_[1][2] = default_tip_position_[2] + swing_height_;
}

/*******************************************************************************************************************//**
 * Generates control nodes for quartic bezier curve of stance tip trajectory calculation.
 * @param[in] stride_vector A vector defining the stride which the leg is to step as part of the step cycle trajectory.
***********************************************************************************************************************/
void LegStepper::generateStanceControlNodes(const Vector3d& stride_vector)
{
  // Control nodes for stance quartic bezier curve
  // Set as initial tip position
  stance_nodes_[0] = stance_origin_tip_position_;
  // Set as target tip position
  stance_nodes_[4] = stance_origin_tip_position_ - stride_vector;
  // Set for constant velocity in stance phase
  stance_nodes_[1] = stance_nodes_[4] + 0.75 * (stance_nodes_[0] - stance_nodes_[4]);
  // Set for constant velocity in stance phase
  stance_nodes_[2] = stance_nodes_[4] + 0.5 * (stance_nodes_[0] - stance_nodes_[4]);
  // Set for constant velocity in stance phase;
  stance_nodes_[3] = stance_nodes_[4] + 0.25 * (stance_nodes_[0] - stance_nodes_[4]);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
