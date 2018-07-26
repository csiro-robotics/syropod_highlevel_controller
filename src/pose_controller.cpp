/*******************************************************************************************************************//**
 *  @file    pose_controller.cpp
 *  @brief   Handles control of Syropod body posing.
 *
 *  @author  Fletcher Talbot (fletcher.talbot@csiro.au)
 *  @date    April 2018
 *  @version 0.5.10
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

#include "../include/syropod_highlevel_controller/pose_controller.h"
#include "../include/syropod_highlevel_controller/walk_controller.h"

/*******************************************************************************************************************//**
 * PoseController class constructor. Initialises member variables.
 * @param[in] model Pointer to the robot model class object
 * @param[in] params Pointer to the parameter struct object
***********************************************************************************************************************/
PoseController::PoseController(shared_ptr<Model> model, const Parameters& params)
  : model_(model)
  , params_(params)
{
  resetAllPosing();

  rotation_absement_error_ = Vector3d::Zero();
  rotation_position_error_ = Vector3d::Zero();
  rotation_velocity_error_ = Vector3d::Zero();
  
  translation_velocity_input_ = Vector3d::Zero();
  rotation_velocity_input_ = Vector3d::Zero();
}

/*******************************************************************************************************************//**
 * Iterates through legs in robot model and generates and assigns a leg poser object. Calls function to initialise auto
 * pose objects. Seperated from constructor due to shared_from_this constraints.
***********************************************************************************************************************/
void PoseController::init(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    leg->setLegPoser(allocate_shared<LegPoser>(aligned_allocator<LegPoser>(), shared_from_this(), leg));
  }
  setAutoPoseParams();
  walk_plane_pose_.position_ = Vector3d(0.0, 0.0, params_.body_clearance.data);
  origin_walk_plane_pose_ = walk_plane_pose_;
}

/*******************************************************************************************************************//**
 * Initialises auto poser container and populates with auto poser class objects as defined by auto poser parameters.
 * Also sets auto pose parameters for the leg poser object of each leg object in robot model.
***********************************************************************************************************************/
void PoseController::setAutoPoseParams(void)
{
  double raw_phase_length;
  int base_phase_length;
  pose_frequency_ = params_.pose_frequency.data;

  // Calculate posing phase length and normalisation values based off gait/posing cycle parameters
  if (pose_frequency_ == -1.0) //Use step cycle parameters
  {
    base_phase_length = params_.stance_phase.data + params_.swing_phase.data;
    double swing_ratio = double(params_.swing_phase.data) / base_phase_length;
    raw_phase_length = ((1.0 / params_.step_frequency.current_value) / params_.time_delta.data) / swing_ratio;
  }
  else
  {
    base_phase_length = params_.pose_phase_length.data;
    raw_phase_length = ((1.0 / pose_frequency_) / params_.time_delta.data);
  }
  pose_phase_length_ = roundToEvenInt(raw_phase_length / base_phase_length) * base_phase_length;
  normaliser_ = pose_phase_length_ / base_phase_length;

  // Set posing negation phase variables according to auto posing parameters
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    leg_poser->setPoseNegationPhaseStart(params_.pose_negation_phase_starts.data.at(leg->getIDName()));
    leg_poser->setPoseNegationPhaseEnd(params_.pose_negation_phase_ends.data.at(leg->getIDName()));
    leg_poser->setNegationTransitionRatio(params_.negation_transition_ratio.data.at(leg->getIDName()));

    // Set reference leg for auto posing system to that which has zero phase offset
    if (params_.offset_multiplier.data.at(leg->getIDName()) == 0)
    {
      auto_pose_reference_leg_ = leg;
    }
  }

  // Clear any old auto-poser objects and re-populate container
  auto_poser_container_.clear();
  for (int i = 0; i < int(params_.pose_phase_starts.data.size()); ++i)
  {
    auto_poser_container_.push_back(allocate_shared<AutoPoser>(aligned_allocator<AutoPoser>(), shared_from_this(), i));
  }

  // For each auto-poser object set control variables from auto_posing parameters
  AutoPoserContainer::iterator auto_poser_it;
  for (auto_poser_it = auto_poser_container_.begin(); auto_poser_it != auto_poser_container_.end(); ++auto_poser_it)
  {
    shared_ptr<AutoPoser> auto_poser = *auto_poser_it;
    int id = auto_poser->getIDNumber();
    auto_poser->setStartPhase(params_.pose_phase_starts.data[id]);
    auto_poser->setEndPhase(params_.pose_phase_ends.data[id]);
    auto_poser->setXAmplitude(params_.x_amplitudes.data[id]);
    auto_poser->setYAmplitude(params_.y_amplitudes.data[id]);
    auto_poser->setZAmplitude(params_.z_amplitudes.data[id]);
    auto_poser->setGravityAmplitude(params_.gravity_amplitudes.data[id]);
    auto_poser->setRollAmplitude(params_.roll_amplitudes.data[id]);
    auto_poser->setPitchAmplitude(params_.pitch_amplitudes.data[id]);
    auto_poser->setYawAmplitude(params_.yaw_amplitudes.data[id]);
    auto_poser->resetChecks();
  }
}

/*******************************************************************************************************************//**
 * Iterates through legs in robot model and updates each Leg Poser tip position. This new tip position is the tip
 * position defined from the Leg Stepper, posed using the current desired pose. The applied pose is dependent on the
 * state of the Leg and Leg Poser specific auto posing.
***********************************************************************************************************************/
void PoseController::updateStance(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    Pose current_pose = model_->getCurrentPose();
    LegState leg_state = leg->getLegState();

    if (leg_state == WALKING || leg_state == MANUAL_TO_WALKING)
    {
      // Remove auto posing from current pose under correct conditions and add leg specific auto pose
      current_pose = current_pose.removePose(auto_pose_);
      current_pose = current_pose.addPose(leg_poser->getAutoPose());

      // Apply pose to current walking tip position to calculate new 'posed' tip position
      Vector3d new_tip_position = current_pose.inverseTransformVector(leg_stepper->getCurrentTipPose().position_);
      Quaterniond new_tip_rotation = current_pose.rotation_.inverse() * leg_stepper->getCurrentTipPose().rotation_;
      Pose new_pose(new_tip_position, new_tip_rotation);
      ROS_ASSERT(new_pose.isValid());
      leg_poser->setCurrentTipPose(new_pose);
    }
    // Do not apply any posing to manually manipulated legs
    else if (leg_state == MANUAL || leg_state == WALKING_TO_MANUAL)
    {
      leg_poser->setCurrentTipPose(leg_stepper->getCurrentTipPose());
    }
  }
}

/*******************************************************************************************************************//**
 * Executes saved transition sequence in direction defined by 'sequence' (START_UP or SHUT_DOWN) through the use of the
 * function StepToPosition() to move to pre-defined tip positions for each leg in the robot model. If no sequence exists
 * for target stance, it generates one iteratively by checking workspace limitations.
 * @param[in] sequence The requested sequence - either START_UP or SHUT_DOWN
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
 * @todo Make sequential leg stepping coordination an option instead of only simultaneous (direct) & groups (tripod)
***********************************************************************************************************************/
int PoseController::executeSequence(const SequenceSelection& sequence)
{
  bool debug = params_.debug_execute_sequence.data;

  // Initialise/Reset any saved transition sequence
  if (reset_transition_sequence_ && sequence == START_UP)
  {
    reset_transition_sequence_ = false;
    first_sequence_execution_ = true;
    transition_step_ = 0;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      shared_ptr<Leg> leg = leg_it_->second;
      shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
      leg_poser->resetTransitionSequence();
      leg_poser->addTransitionPose(leg->getCurrentTipPose()); // Initial transition position
    }
  }

  int progress = 0; // Percentage progress (0%->100%)
  int normalised_progress;

  // Setup sequence type sepecific variables (transition type, direction and target)
  int next_transition_step;
  int transition_step_target;
  bool execute_horizontal_transition;
  bool execute_vertical_transition;
  int total_progress;
  if (sequence == START_UP)
  {
    execute_horizontal_transition = !(transition_step_ % 2); // Even steps
    execute_vertical_transition = transition_step_ % 2; // Odd steps
    next_transition_step = transition_step_ + 1;
    transition_step_target = transition_step_count_;
    total_progress = transition_step_ * 100 / max(transition_step_count_, 1);
  }
  else if (sequence == SHUT_DOWN)
  {
    execute_horizontal_transition = transition_step_ % 2; // Odd steps
    execute_vertical_transition = !(transition_step_ % 2); // Even steps
    next_transition_step = transition_step_ - 1;
    transition_step_target = 0;
    total_progress = 100 - transition_step_ * 100 / max(transition_step_count_, 1);
  }

  //Determine if this transition is the last one before end of sequence
  bool final_transition;
  bool sequence_complete = false;
  if (first_sequence_execution_)
  {
    final_transition = (horizontal_transition_complete_ || vertical_transition_complete_);
  }
  else
  {
    final_transition = (next_transition_step == transition_step_target);
  }

  // Safety factor during first sequence execution decreases for each successive transition
  double safety_factor = (first_sequence_execution_ ? SAFETY_FACTOR / (transition_step_ + 1) : 0.0);

  // Attempt to step (in specific coordination) along horizontal plane to transition positions
  if (execute_horizontal_transition)
  {
    if (set_target_)
    {
      // Set horizontal target
      set_target_ = false;
      ROS_DEBUG_COND(debug, "\nTRANSITION STEP: %d (HORIZONTAL):\n", transition_step_);
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        shared_ptr<Leg> leg = leg_it_->second;
        shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
        leg_poser->setLegCompletedStep(false);

        Vector3d target_tip_position;
        if (leg_poser->hasTransitionPose(next_transition_step))
        {
          ROS_DEBUG_COND(debug, "\nLeg %s targeting transition position %d.\n",
                         leg->getIDName().c_str(), next_transition_step);
          target_tip_position = leg_poser->getTransitionPose(next_transition_step).position_;
        }
        else
        {
          ROS_DEBUG_COND(debug, "\nNo transition pose found for leg %s - targeting default stance pose.\n",
                         leg->getIDName().c_str());
          Vector3d default_tip_position = leg_stepper->getDefaultTipPose().position_;
          target_tip_position = model_->getCurrentPose().inverseTransformVector(default_tip_position);
        }
        
        //Maintain horizontal position
        target_tip_position[2] = leg->getCurrentTipPose().position_[2];
        
        Quaterniond target_tip_rotation = leg_stepper->getTargetTipPose().rotation_;
        leg_poser->setTargetTipPose(Pose(target_tip_position, target_tip_rotation));
      }
    }

    // Step to target
    bool direct_step = !model_->legsBearingLoad();
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      shared_ptr<Leg> leg = leg_it_->second;
      shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
      if (!leg_poser->getLegCompletedStep())
      {
        // Step leg if leg is in stepping group OR simultaneous direct stepping is allowed
        if (leg->getGroup() == current_group_ || direct_step)
        {
          Pose target_tip_pose = leg_poser->getTargetTipPose();
          bool apply_delta = (sequence == START_UP && final_transition); //Only add delta at end of StartUp sequence
          double step_height = direct_step ? 0.0 : params_.swing_height.current_value;
          double time_to_step = HORIZONTAL_TRANSITION_TIME / params_.step_frequency.current_value;
          time_to_step *= (first_sequence_execution_ ? 2.0 : 1.0); // Double time for initial sequence
          progress = leg_poser->stepToPosition(target_tip_pose, Pose::Identity(), step_height, time_to_step, apply_delta);
          leg->setDesiredTipPose(leg_poser->getCurrentTipPose());
          double limit_proximity = leg->applyIK();
          bool exceeded_workspace = limit_proximity < safety_factor; // Leg attempted to move beyond safe workspace

          // Leg has attempted to move beyond workspace so stop transition early
          if (first_sequence_execution_ && exceeded_workspace)
          {
            leg_poser->setTargetTipPose(leg_poser->getCurrentTipPose());
            progress = leg_poser->resetStepToPosition(); // Skips to 'complete' progress and resets
            proximity_alert_ = true;
          }

          if (progress == PROGRESS_COMPLETE)
          {
            leg_poser->setLegCompletedStep(true);
            legs_completed_step_ ++;
            if (first_sequence_execution_)
            {
              // Send sequence optimisation debug message
              if (debug && transition_step_ == 0)
              {
                string joint_position_string;
                for (joint_it_ = leg->getJointContainer()->begin();
                     joint_it_ != leg->getJointContainer()->end(); ++joint_it_)
                {
                  shared_ptr<Joint> joint = joint_it_->second;
                  joint_position_string += stringFormat("\tJoint: %s\tPosition: %f\n",
                                                        joint->id_name_.c_str(), joint->desired_position_);
                }
                ROS_DEBUG("\nLeg %s has completed first transition.\n"
                          "Optimise sequence by setting 'unpacked' joint positions to the following:\n%s", 
                          leg->getIDName().c_str(), joint_position_string.c_str());
              }
              bool reached_target = !exceeded_workspace;
              Pose target_tip_pose = leg_poser->getTargetTipPose();
              Pose current_tip_pose = leg_poser->getCurrentTipPose();
              Pose transition_pose = (reached_target ? target_tip_pose : current_tip_pose);
              leg_poser->addTransitionPose(transition_pose);
              ROS_DEBUG_COND(debug, "\nAdded transition pose %d for leg %s.\n",
                             next_transition_step, leg->getIDName().c_str());
            }
          }
        }
        // Leg not designated to step so set step completed
        else
        {
          legs_completed_step_++;
          leg_poser->setLegCompletedStep(true);
        }
      }
    }

    // Normalise transition progress for use in calculation of total sequence progress
    if (direct_step)
    {
      normalised_progress = progress / max(transition_step_count_, 1);
    }
    else
    {
      normalised_progress = (progress / 2 + (current_group_ == 0 ? 0 : 50)) / max(transition_step_count_, 1);
    }

    // Check if legs have completed steps and if transition has completed without a proximity alert
    // TODO Future work - make sequential leg stepping coordination an option
    if (legs_completed_step_ == model_->getLegCount())
    {
      set_target_ = true;
      legs_completed_step_ = 0;
      if (current_group_ == 1 || direct_step)
      {
        current_group_ = 0;
        transition_step_ = next_transition_step;
        horizontal_transition_complete_ = !proximity_alert_;
        sequence_complete = final_transition;
        proximity_alert_ = false;
      }
      else if (current_group_ == 0)
      {
        current_group_ = 1;
      }
    }
  }

  // Attempt to step directly along vertical trajectory to transition positions
  if (execute_vertical_transition)
  {
    if (set_target_)
    {
      // Set vertical target
      set_target_ = false;
      ROS_DEBUG_COND(debug, "\nTRANSITION STEP: %d (VERTICAL):\n", transition_step_);
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        shared_ptr<Leg> leg = leg_it_->second;
        shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
        Vector3d target_tip_position;
        if (leg_poser->hasTransitionPose(next_transition_step))
        {
          ROS_DEBUG_COND(debug, "\nLeg %s targeting transition position %d.\n",
                         leg->getIDName().c_str(), next_transition_step);
          target_tip_position = leg_poser->getTransitionPose(next_transition_step).position_;
        }
        else
        {
          ROS_DEBUG_COND(debug, "\nNo transition position found for leg %s - targeting default stance position.\n",
                         leg->getIDName().c_str());
          Vector3d default_tip_position = leg_stepper->getDefaultTipPose().position_;
          target_tip_position = model_->getCurrentPose().inverseTransformVector(default_tip_position);
        }

        //Maintain horizontal position
        target_tip_position[0] = leg->getCurrentTipPose().position_[0];
        target_tip_position[1] = leg->getCurrentTipPose().position_[1];
        
        Quaterniond target_tip_rotation = leg_stepper->getTargetTipPose().rotation_;
        leg_poser->setTargetTipPose(Pose(target_tip_position, target_tip_rotation));
      }
    }

    // Step to target
    bool all_legs_within_workspace = true;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      shared_ptr<Leg> leg = leg_it_->second;
      shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
      Pose target_tip_pose = leg_poser->getTargetTipPose();
      bool apply_delta = (sequence == START_UP && final_transition);
      double time_to_step = VERTICAL_TRANSITION_TIME / params_.step_frequency.current_value;
      time_to_step *= (first_sequence_execution_ ? 2.0 : 1.0);
      progress = leg_poser->stepToPosition(target_tip_pose, Pose::Identity(), 0.0, time_to_step, apply_delta);
      leg->setDesiredTipPose(leg_poser->getCurrentTipPose(), false);
      double limit_proximity = leg->applyIK();
      all_legs_within_workspace = all_legs_within_workspace && !(limit_proximity < safety_factor);
      ROS_DEBUG_COND(debug && limit_proximity < safety_factor,
                     "\nLeg %s exceeded safety factor\n", leg->getIDName().c_str());
    }

    // All legs have completed vertical transition (either by reaching target or exceeding safe workspace)
    if ((!all_legs_within_workspace && first_sequence_execution_) || progress == PROGRESS_COMPLETE)
    {
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        shared_ptr<Leg> leg = leg_it_->second;
        shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
        progress = leg_poser->resetStepToPosition();
        if (first_sequence_execution_)
        {
          bool reached_target = all_legs_within_workspace; // Assume reached target if all are within safe workspace
          Pose target_tip_pose = leg_poser->getTargetTipPose();
          Pose current_tip_pose = leg_poser->getCurrentTipPose();
          Pose transition_pose = (reached_target ? target_tip_pose : current_tip_pose);
          leg_poser->addTransitionPose(transition_pose);
          ROS_DEBUG_COND(debug, "\nAdded transition position %d for leg %s.\n",
                         next_transition_step, leg->getIDName().c_str());
        }
      }

      vertical_transition_complete_ = all_legs_within_workspace;
      transition_step_ = next_transition_step;
      sequence_complete = final_transition; //Sequence is complete if this transition was the final one
      set_target_ = true;
    }

    // Normalise transition progress for use in calculation of total sequence progress
    normalised_progress = progress / max(transition_step_count_, 1);
  }

  // Update count of transition steps as first sequence executes
  if (first_sequence_execution_)
  {
    transition_step_count_ = transition_step_;
    transition_step_target = transition_step_;
  }

  // Check for excessive transition steps
  if (transition_step_ > TRANSITION_STEP_THRESHOLD)
  {
    ROS_FATAL("\nUnable to execute sequence, shutting down controller.\n");
    ros::shutdown();
  }

  // Check if sequence has completed
  if (sequence_complete)
  {
    set_target_ = true;
    vertical_transition_complete_ = false;
    horizontal_transition_complete_ = false;
    first_sequence_execution_ = false;
    return  PROGRESS_COMPLETE;
  }
  // If sequence has not completed return percentage estimate of completion (i.e. < 100%)
  else
  {
    total_progress = min(total_progress + normalised_progress, PROGRESS_COMPLETE - 1);
    return (first_sequence_execution_ ? -1 : total_progress);
  }
}

/*******************************************************************************************************************//**
 * Iterates through legs in robot model and, in simulation, moves them in a linear trajectory directly from
 * their current tip position to its default tip position (as defined by the walk controller). This motion completes
 * in a time limit defined by the parameter time_to_start.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::directStartup(void) //Simultaneous leg coordination
{
  int progress = 0; // Percentage progress (0%->100%)
  double time_to_start = params_.time_to_start.data;

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();

    // Run model in simulation to find joint positions for default stance.
    if (!executing_transition_)
    {
      // Create copy of leg at initial state
      shared_ptr<Leg> test_leg = allocate_shared<Leg>(aligned_allocator<Leg>(), leg);
      test_leg->generate(leg);
      test_leg->init(true);

      // Move tip linearly to default stance position
      Pose default_tip_pose = leg_stepper->getDefaultTipPose();
      while (progress != PROGRESS_COMPLETE)
      {
        shared_ptr<LegPoser> test_leg_poser = test_leg->getLegPoser();
        progress = test_leg_poser->stepToPosition(default_tip_pose, model_->getCurrentPose(), 0.0, time_to_start);
        test_leg->setDesiredTipPose(test_leg_poser->getCurrentTipPose(), true);
        test_leg->applyIK(true);
      }

      /// Create empty configuration
      sensor_msgs::JointState default_configuration;
      default_configuration.name.assign(test_leg->getJointCount(), "");
      default_configuration.position.assign(test_leg->getJointCount(), UNASSIGNED_VALUE);

      // Populate configuration with default values
      JointContainer::iterator joint_it;
      for (joint_it = test_leg->getJointContainer()->begin(); joint_it != test_leg->getJointContainer()->end(); ++joint_it)
      {
        shared_ptr<Joint> joint = joint_it->second;
        int joint_index = joint->id_number_ - 1;
        default_configuration.name[joint_index] = joint->id_name_;
        default_configuration.position[joint_index] = joint->desired_position_;
      }
      leg_poser->setDesiredConfiguration(default_configuration);
    }

    // Transition to Default configuration
    progress = leg_poser->transitionConfiguration(time_to_start);
  }
  
  executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
  return progress;
}

/*******************************************************************************************************************//**
 * Iterates through legs in robot model and attempts to step each from their current tip position to their default tip
 * position (as defined by the walk controller). The stepping motion is coordinated such that half of the legs execute
 * the step at any one time (for a hexapod this results in a Tripod stepping coordination). The time period and
 * height of the stepping maneuver is controlled by the user parameters step_frequency and step_clearance.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::stepToNewStance(void) //Tripod leg coordination
{
  int progress = 0; // Percentage progress (0%->100%)
  int leg_count = model_->getLegCount();
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    if (leg->getGroup() == current_group_)
    {
      shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
      shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
      double step_height = params_.swing_height.current_value;
      double step_time = 1.0 / params_.step_frequency.current_value;
      Pose target_tip_pose = leg_stepper->getDefaultTipPose();
      progress = leg_poser->stepToPosition(target_tip_pose, model_->getCurrentPose(), step_height, step_time);
      leg->setDesiredTipPose(leg_poser->getCurrentTipPose());
      leg->applyIK();
      legs_completed_step_ += int(progress == PROGRESS_COMPLETE);
    }
  }

  // Normalise progress in terms of total procedure
  progress = progress / 2 + current_group_ * 50;

    current_group_ = legs_completed_step_ / (leg_count / 2);

  if (legs_completed_step_ == leg_count)
  {
    legs_completed_step_ = 0;
    current_group_ = 0;
  }

  // Set flag to reset any stored transition sequences and generate new sequence for new stance
  reset_transition_sequence_ = true;

  return progress;
}

/*******************************************************************************************************************//**
 * Iterates through the legs in the robot model and generates a pose for each that is best for leg manipulation. This
 * pose is generated to attempt to move the centre of gravity within the support polygon of the load bearing legs. All
 * legs simultaneously step to each new generated pose and the time period and height of the stepping maneuver is
 * controlled by the user parameters step_frequency and step_clearance.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::poseForLegManipulation(void) //Simultaneous leg coordination
{
  Pose target_pose;
  int min_progress = UNASSIGNED_VALUE; // Percentage progress (0%->100%)
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    double step_height = params_.swing_height.current_value;
    double step_time = 1.0 / params_.step_frequency.current_value;

    // Set up target pose for legs depending on state
    if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      target_pose = Pose::Identity();
      target_pose.position_ += inclination_pose_.position_; // Apply inclination control to lifted leg
      target_pose.position_[2] -= step_height; // Pose leg at step height to begin manipulation
    }
    else
    {
      target_pose = model_->getCurrentPose();
      target_pose.position_ -= manual_pose_.position_; // Remove manual pose
      target_pose.position_ += default_pose_.position_; // Add default pose as estimated from new loading pattern
    }

    Pose target_tip_pose = Pose::Undefined();
    target_tip_pose.position_ = target_pose.inverseTransformVector(leg_stepper->getDefaultTipPose().position_);

    // Set walker tip position for use in manual or walking mode
    if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      leg_stepper->setCurrentTipPose(target_tip_pose);
      step_height = 0.0; // Zero step height in transition from WALKING to MANUAL
    }
    else if (leg->getLegState() == MANUAL_TO_WALKING)
    {
      leg_stepper->setCurrentTipPose(leg_stepper->getDefaultTipPose());
    }

    int progress = leg_poser->stepToPosition(target_tip_pose, Pose::Identity(), step_height, step_time);
    min_progress = min(progress, min_progress);
    if (progress != PROGRESS_COMPLETE)
    {
      leg->setDesiredTipPose(leg_poser->getCurrentTipPose());
      leg->applyIK();
    }
  }

  return min_progress;
}

/*******************************************************************************************************************//**
 * Iterate through legs in robot model and directly move joints into 'packed' configuration as defined by joint
 * parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
 * @param[in] time_to_pack The time period in which to execute the packing maneuver.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::packLegs(const double& time_to_pack) //Simultaneous leg coordination
{
  int progress = 0; //Percentage progress (0%->100%)
  transition_step_ = 0; //Reset for startUp/ShutDown sequences
  int number_pack_steps = 1;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    number_pack_steps = model_->getLegByIDNumber(0)->getJointByIDNumber(1)->packed_positions_.size();
    
    // Generate unpacked configuration
    if (!executing_transition_)
    {
      // Create empty configuration
      sensor_msgs::JointState packed_configuration;
      packed_configuration.name.assign(leg->getJointCount(), "");
      packed_configuration.position.assign(leg->getJointCount(), UNASSIGNED_VALUE);

      // Populate configuration with packed values
      JointContainer::iterator joint_it;
      for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
      {
        int joint_index = joint_it->second->id_number_ - 1;
        packed_configuration.name[joint_index] = joint_it->second->id_name_;
        packed_configuration.position[joint_index] = joint_it->second->packed_positions_[pack_step_];
      }
      leg_poser->setDesiredConfiguration(packed_configuration);
    }
    
    // Transition to Unpacked configuration
    progress = leg_poser->transitionConfiguration(time_to_pack);
  }
  
  executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
  if (progress == PROGRESS_COMPLETE && pack_step_ < number_pack_steps - 1)
  {
    executing_transition_ = false;
    pack_step_++;
    progress = 0;
  }
  
  return progress;
}

/*******************************************************************************************************************//**
 * Iterate through legs in robot model and directly move joints into 'unpacked' configuration as defined by joint
 * parameters. This maneuver occurs simultaneously for all legs in a time period defined by the input argument.
 * @param[in] time_to_unpack The time period in which to execute the packing maneuver.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::unpackLegs(const double& time_to_unpack) //Simultaneous leg coordination
{
  int progress = 0; //Percentage progress (0%->100%)

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    
    // Generate unpacked configuration
    if (!executing_transition_)
    {
      // Create empty configuration
      sensor_msgs::JointState unpacked_configuration;
      unpacked_configuration.name.assign(leg->getJointCount(), "");
      unpacked_configuration.position.assign(leg->getJointCount(), UNASSIGNED_VALUE);
      
      // Populate configuration with unpacked values
      JointContainer::iterator joint_it;
      for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
      {
        shared_ptr<Joint> joint = joint_it->second;
        int joint_index = joint->id_number_ - 1;
        unpacked_configuration.name[joint_index] = joint->id_name_;
        double target_position = (pack_step_ > 0) ? joint->packed_positions_.at(pack_step_ - 1) : joint->unpacked_position_;
        unpacked_configuration.position[joint_index] =  target_position;
      }
      leg_poser->setDesiredConfiguration(unpacked_configuration);
    }
    
    // Transition to Unpacked configuration
    progress = leg_poser->transitionConfiguration(time_to_unpack);
  }
  
  executing_transition_ = (progress != 0 && progress != PROGRESS_COMPLETE);
  if (progress == PROGRESS_COMPLETE && pack_step_ != 0)
  {
    executing_transition_ = false;
    pack_step_--;
    progress = 0;
  }
  
  return progress;
}

/*******************************************************************************************************************//**
 * Iterate through legs in robot model and directly move joints to positions defined by desired configuration. This
 * transition occurs simultaneously for all legs in a time period defined by the input argument.
 * @param[in] transition_time The time period in which to execute the transition
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::transitionConfiguration(const double& transition_time) //Simultaneous leg coordination
{
  int min_progress = INT_MAX; //Percentage progress (0%->100%)
  
  // Iterate through message and build individual leg configurations
  map<string, sensor_msgs::JointState> configuration_sorter;
  if (!executing_transition_)
  {
    for (uint i = 0; i < target_configuration_.name.size(); ++i)
    {
      string joint_name = target_configuration_.name[i];
      string leg_name = joint_name.substr(0, joint_name.find("_"));
      int joint_count = model_->getLegByIDName(leg_name)->getJointCount();
      int joint_index = model_->getLegByIDName(leg_name)->getJointByIDName(joint_name)->id_number_ - 1;
      
      // Create empty configuration for this leg
      if (configuration_sorter.find(leg_name) == configuration_sorter.end())
      {
        sensor_msgs::JointState new_leg_configuration;
        new_leg_configuration.name.assign(joint_count, "");
        new_leg_configuration.position.assign(joint_count, UNASSIGNED_VALUE);
        configuration_sorter.insert(map<string, sensor_msgs::JointState>::value_type(leg_name, new_leg_configuration));
      }
      
      // Populate configuration with desired values
      configuration_sorter.at(leg_name).name[joint_index] = target_configuration_.name[i];
      configuration_sorter.at(leg_name).position[joint_index] = target_configuration_.position[i];
    }
  }
  
  

  // Run configuration transition for each leg
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    if (!executing_transition_)
    {
      sensor_msgs::JointState desired_configuration;
      if (configuration_sorter.find(leg->getIDName()) != configuration_sorter.end())
      {
        desired_configuration = configuration_sorter.at(leg->getIDName());
      }
      leg_poser->setDesiredConfiguration(desired_configuration);
    }
    int progress = leg_poser->transitionConfiguration(transition_time);
    min_progress = min(progress, min_progress);
  }

  executing_transition_ = (min_progress != 0 && min_progress != PROGRESS_COMPLETE);
  return min_progress;
}

/*******************************************************************************************************************//**
 * Iterate through legs in robot model and directly move tips to pose defined by target tip pose and target body pose. 
 * This transition occurs simultaneously for all legs in a time period defined by the input argument.
 * @param[in] transition_time The time period in which to execute the transition
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int PoseController::transitionStance(const double& transition_time)
{
  int min_progress = INT_MAX; //Percentage progress (0%->100%)
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    ExternalTarget target = leg_poser->getExternalTarget();
    Pose target_tip_pose = Pose::Undefined();
    double swing_clearance = 0.0;
    
    // Update target if externally set target exists
    if (target.defined_)
    {
      target_tip_pose = target.pose_.removePose(target.transform_);
      swing_clearance = target.swing_clearance_;
    }
    
    // Update target rotation if gravity alignment is set
    if (target_tip_pose.rotation_.isApprox(UNDEFINED_ROTATION) && params_.gravity_aligned_tips.data)
    {
      target_tip_pose.rotation_ = Quaterniond::FromTwoVectors(Vector3d::UnitX(), model_->estimateGravity());
    }
    
    // Step to target pose
    int progress = leg_poser->stepToPosition(target_tip_pose, target_body_pose_, swing_clearance, transition_time, true);
    ROS_INFO("PROGRESS %s: %d", leg->getIDName().c_str(), progress);
    leg->setDesiredTipPose(leg_poser->getCurrentTipPose());
    leg->applyIK();
    min_progress = min(progress, min_progress);
    
    // Reset target if target achieved
    if (target.defined_ && progress == PROGRESS_COMPLETE)
    {
      target.defined_ = false;
      leg_poser->setExternalTarget(target);
    }
  }
  ROS_INFO("*****************");
  return min_progress;
}

/*******************************************************************************************************************//**
 * Depending on parameter flags, calls multiple posing functions and combines individual poses to update the current
 * desired pose of the robot model.
 * @param[in] robot_state The current state of the robot
***********************************************************************************************************************/
void PoseController::updateCurrentPose(const RobotState& robot_state)
{
  Pose new_pose = Pose::Identity();
  
  // Pose body at clearance offset normal to walk plane and rotate to align parallel
  updateWalkPlanePose();
  new_pose = new_pose.addPose(walk_plane_pose_);
  ROS_ASSERT(walk_plane_pose_.isValid());
  model_->setDefaultPose(walk_plane_pose_);

  // Manually set (joystick controlled) body pose
  if (params_.manual_posing.data)
  {
    updateManualPose();
    new_pose = new_pose.addPose(manual_pose_);
  }

  // Pose to align centre of gravity evenly between tip positions on incline
  if (params_.inclination_posing.data)
  {
    updateInclinationPose();
    new_pose = new_pose.addPose(inclination_pose_);
  }

  // Auto body pose using IMU feedback
  if (params_.imu_posing.data && robot_state == RUNNING)
  {
    updateIMUPose();
    new_pose = new_pose.addPose(imu_pose_);
  }
  // Automatic (non-feedback) pre-defined cyclical body posing
  else if (params_.auto_posing.data)
  {
    updateAutoPose();
    new_pose = new_pose.addPose(auto_pose_);
  }
  
  // Automatic (non-feedback) body posing to align tips orthogonal to walk plane during 2nd half of swing
  if (params_.gravity_aligned_tips.data && model_->getLegByIDNumber(0)->getJointCount() <= 3) // TODO EXPERIMENTAL
  {
    updateTipAlignPose();
    new_pose = new_pose.addPose(tip_align_pose_);
    //updateIKErrorPose();
    //new_pose = new_pose.addPose(ik_error_pose_);
  }
  
  ROS_ASSERT(new_pose.isValid());
  model_->setCurrentPose(new_pose);
}

/*******************************************************************************************************************//**
 * Generates a manual pose to be applied to the robot model, based on linear (x/y/z) and angular (roll/pitch/yaw)
 * velocity body posing inputs. Clamps the posing within set limits and resets the pose to zero in specified axes
 * depending on the pose reset mode.
***********************************************************************************************************************/
void PoseController::updateManualPose(void)
{
  double time_delta = params_.time_delta.data;
  Vector3d current_position = manual_pose_.position_;
  Vector3d current_rotation = quaternionToEulerAngles(manual_pose_.rotation_, true);
  Vector3d default_position = default_pose_.position_;
  Vector3d default_rotation = quaternionToEulerAngles(default_pose_.rotation_, true);
  Vector3d max_position(params_.max_translation.data.at("x"),
                        params_.max_translation.data.at("y"),
                        params_.max_translation.data.at("z"));
  Vector3d max_rotation(params_.max_rotation.data.at("roll"),
                        params_.max_rotation.data.at("pitch"),
                        params_.max_rotation.data.at("yaw"));

  Vector3d translation_limit(0, 0, 0);
  Vector3d rotation_limit(0, 0, 0);
  Vector3d translation_velocity(0, 0, 0);
  Vector3d rotation_velocity(0, 0, 0);
  Vector3d desired_position(0, 0, 0);
  Vector3d desired_rotation(0, 0, 0);

  // Populate axis reset values from pose reset mode
  bool reset_translation[3] = { false, false, false };
  bool reset_rotation[3] = { false, false, false };
  switch (pose_reset_mode_)
  {
    case (Z_AND_YAW_RESET):
      reset_translation[2] = true;
      reset_rotation[2] = true;
      break;
    case (X_AND_Y_RESET):
      reset_translation[0] = true;
      reset_translation[1] = true;
      break;
    case (PITCH_AND_ROLL_RESET):
      reset_rotation[0] = true;
      reset_rotation[1] = true;
      break;
    case (ALL_RESET):
      reset_translation[0] = true;
      reset_translation[1] = true;
      reset_translation[2] = true;
      reset_rotation[0] = true;
      reset_rotation[1] = true;
      reset_rotation[2] = true;
      break;
    case (IMMEDIATE_ALL_RESET):
      manual_pose_ = default_pose_;
      return;
    case (NO_RESET):  // Do nothing
    default:
      break;
  }

  // Override posing velocity commands depending on pose reset mode
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    if (reset_translation[i])
    {
      double diff = current_position[i] - default_position[i];
      if (diff < 0)
      {
        translation_velocity_input_[i] = 1.0;
      }
      else if (diff > 0)
      {
        translation_velocity_input_[i] = -1.0;
      }
    }

    if (reset_rotation[i])
    {
      double diff = current_rotation[i] - default_rotation[i];
      if (diff < 0)
      {
        rotation_velocity_input_[i] = 1.0;
      }
      else if (diff > 0)
      {
        rotation_velocity_input_[i] = -1.0;
      }
    }

    translation_velocity[i] = translation_velocity_input_[i] * params_.max_translation_velocity.data;
    rotation_velocity[i] = rotation_velocity_input_[i] * params_.max_rotation_velocity.data;

    desired_position[i] = current_position[i] + translation_velocity[i] * time_delta;
    desired_rotation[i] = current_rotation[i] + rotation_velocity[i] * time_delta;

    // Zero velocity input depending on position limitations
    // TRANSLATION
    // Assign correct translation limit based on velocity direction and reset command
    translation_limit[i] = sign(translation_velocity[i]) * max_position[i];

    if (reset_translation[i] && default_position[i] < max_position[i] && default_position[i] > -max_position[i])
    {
      translation_limit[i] = default_position[i];
    }

    bool positive_translation_velocity = sign(translation_velocity[i]) > 0;
    bool exceeds_positive_translation_limit = positive_translation_velocity &&
                                              desired_position[i] > translation_limit[i];
    bool exceeds_negative_translation_limit = !positive_translation_velocity &&
                                              desired_position[i] < translation_limit[i];

    // Zero velocity when translation position reaches limit
    if (exceeds_positive_translation_limit || exceeds_negative_translation_limit)
    {
      translation_velocity[i] = (translation_limit[i] - current_position[i]) / time_delta;
    }

    // ROTATION
    // Assign correct rotation limit based on velocity direction and reset command
    rotation_limit[i] = sign(rotation_velocity[i]) * max_rotation[i];

    if (reset_rotation[i] && default_rotation[i] < max_rotation[i] && default_rotation[i] > -max_rotation[i])
    {
      rotation_limit[i] = default_rotation[i];
    }

    bool positive_rotation_velocity = sign(rotation_velocity[i]) > 0;
    bool exceeds_positive_rotation_limit = positive_rotation_velocity &&
                                           desired_rotation[i] > rotation_limit[i];
    bool exceeds_negative_rotation_limit = !positive_rotation_velocity &&
                                           desired_rotation[i] < rotation_limit[i];

    // Zero velocity when rotation position reaches limit
    if (exceeds_positive_rotation_limit || exceeds_negative_rotation_limit)
    {
      rotation_velocity[i] = (rotation_limit[i] - current_rotation[i]) / time_delta;
    }
    
    desired_position[i] = current_position[i] + translation_velocity[i] * time_delta;
    desired_rotation[i] = current_rotation[i] + rotation_velocity[i] * time_delta;
  }

  // Update position according to limitations
  manual_pose_.position_ = desired_position;
  manual_pose_.rotation_ = correctRotation(eulerAnglesToQuaternion(desired_rotation, true), Quaterniond::Identity());
}

/*******************************************************************************************************************//**
 * Poses the body of the robot according to errors in IK for each leg. Ideally, moves legs into configuration
 * to achieve desired tip positions which cannot be achieved otherwise.
 * @todo Improve method for returning ik error pose to zero
***********************************************************************************************************************/
void PoseController::updateIKErrorPose(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    WalkState walk_state = leg->getLegStepper()->getWalkState();
    if (walk_state != STOPPED)
    {
      Vector3d ik_position_error = leg->getCurrentTipPose().position_ - leg->getDesiredTipPose().position_;
      ik_error_pose_.position_ -= ik_position_error;
    }
  }
  ik_error_pose_.position_ *= 0.95; // Returns translation back to zero TODO
}

/*******************************************************************************************************************//**
 * Updates a body pose that, when applied, orients the last joint of a swinging leg inline with the tip along the walk
 * plane normal. This causes the last link of the leg to be oriented orthogonal to the walk plane estimate during the
 * 2nd half of swing. This is used to orient tip sensors to point toward the desired tip landing position at the end of
 * the swing.
***********************************************************************************************************************/
void PoseController::updateTipAlignPose(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    double swing_progress = leg_stepper->getSwingProgress();
    if (swing_progress != -1.0)
    {
      // Calculate vector normal to walk plane and rotation of this vector from vertical.
      Vector3d walk_plane_normal = leg_stepper->getWalkPlaneNormal();
      Quaterniond walk_plane_rotation = Quaterniond::FromTwoVectors(Vector3d::UnitZ(), walk_plane_normal);

      // Calculate vector from tip position to final joint position
      shared_ptr<Tip> tip = leg->getTip();
      shared_ptr<Joint> joint = tip->reference_link_->actuating_joint_;
      Vector3d tip_position = tip->getPoseRobotFrame().position_;
      Vector3d joint_position = joint->getPoseRobotFrame().position_;
      Vector3d tip_to_joint = joint_position - tip_position;
      double link_length = (tip_position - joint_position).norm();

      // Calculate body translation required to align joint position inline with tip position along walk plane normal
      Vector3d a = walk_plane_rotation._transformVector(tip_to_joint);
      Vector3d b = link_length * walk_plane_normal;
      Vector3d rejection = a - (a.dot(b) / b.dot(b))*b; // en.wikipedia.org/wiki/Vector_projection
      Vector3d translation_to_alignment = -rejection;

      // Calculate component of current translation aligned with walk plane
      a = tip_align_pose_.position_;
      b = walk_plane_normal;
      rejection = a - (a.dot(b) / b.dot(b))*b; // en.wikipedia.org/wiki/Vector_projection
      Vector3d current_walk_plane_aligned_translation = rejection;

      // Add current aligned translation with translation required for translation_to_alignment
      Vector3d target_translation = current_walk_plane_aligned_translation + translation_to_alignment;

      // Clamp target translation within limits
      Vector3d limit(params_.max_translation.data.at("x"),
                     params_.max_translation.data.at("y"),
                     params_.max_translation.data.at("z"));
      target_translation = clamped(target_translation, limit);

      // Interpolate between origin tip align pose and calculated target translation
      double c = smoothStep(swing_progress); // Control input (0.0 -> 1.0)
      ROS_ASSERT(c >= 0.0 && c <= 1.0); 
      if (swing_progress < 0.5)
      {
        c = smoothStep(c * 2.0); // 0.0:0.5 -> 0.0:1.0
        tip_align_pose_ = origin_tip_align_pose_.interpolate(c, Pose::Identity());
      }
      else if (swing_progress >= 0.5)
      {
        c = smoothStep((c - 0.5) * 2.0); // 0.5:1.0 -> 0.0:1.0
        tip_align_pose_ = Pose::Identity().interpolate(c, Pose(target_translation, Quaterniond::Identity()));
      }

      // Save pose for origin of interpolation durin next swing period
      if (swing_progress == 1.0)
      {
        origin_tip_align_pose_ = tip_align_pose_;
      }
    }
  }
}

/*******************************************************************************************************************//**
 * Calculates a pose for the robot body such that the robot body is parallel to a calculated walk plane at a normal 
 * offset of the body clearance parameter.
***********************************************************************************************************************/
void PoseController::updateWalkPlanePose(void)
{
  // Generate contol input for transitioning to new walk plane pose using swinging leg as reference.
  Vector3d walk_plane = Vector3d::Zero();
  Vector3d walk_plane_normal = Vector3d::UnitZ();
  double c = 0.0; // Control input ((0.0 -> 1.0)
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
    double swing_progress_scaler = max(1.0, double(params_.swing_phase.data) / params_.phase_offset.data);
    double swing_progress = leg_stepper->getSwingProgress() * swing_progress_scaler; //Handles overlapping swing periods
    
    if (swing_progress >= 0 && swing_progress <= 1.0)
    {
      c = smoothStep(swing_progress); // Use swinging leg progress to smoothly transition to new walk plane pose
      walk_plane = leg_stepper->getWalkPlane(); // Get static walk plane of most up to date leg
      walk_plane_normal = leg_stepper->getWalkPlaneNormal();
    }
  }
  
  // Align robot body with walk plane
  Pose new_walk_plane_pose;
  new_walk_plane_pose.rotation_ = Quaterniond::FromTwoVectors(Vector3d::UnitZ(), walk_plane_normal);
  new_walk_plane_pose.rotation_ = correctRotation(new_walk_plane_pose.rotation_, Quaterniond::Identity());
  
  // Pose robot body along normal of walk plane, offset according to the requested body clearance
  Vector3d body_clearance = Vector3d(0, 0, params_.body_clearance.data);
  new_walk_plane_pose.position_ = new_walk_plane_pose.rotation_._transformVector(body_clearance);
  new_walk_plane_pose.position_[2] += walk_plane[2];
  
  // Interpolate walk plane pose as transitioning from old to new.
  walk_plane_pose_ = origin_walk_plane_pose_.interpolate(c, new_walk_plane_pose);
  ROS_ASSERT(walk_plane_pose_.isValid());
  if (c == 1.0)
  {
    origin_walk_plane_pose_ = walk_plane_pose_;
  }
}

/*******************************************************************************************************************//**
 * Updates the auto pose by feeding each Auto Poser object a phase value and combining the output of each Auto Poser
 * object into a single pose. The input master phase is either an iteration of the pose phase or synced to the step
 * phase from the Walk Controller. This function also iterates through all leg objects in the robot model and updates
 * each Leg Poser's specific Auto Poser pose (this pose is used when the leg needs to ignore the default auto pose)
***********************************************************************************************************************/
void PoseController::updateAutoPose(void)
{
  shared_ptr<LegStepper> leg_stepper = auto_pose_reference_leg_->getLegStepper();
  auto_pose_ = Pose::Identity();

  // Update auto posing state
  bool zero_body_velocity = leg_stepper->getStrideVector().norm() == 0;
  if (leg_stepper->getWalkState() == STARTING || leg_stepper->getWalkState() == MOVING)
  {
    auto_posing_state_ = POSING;
  }
  else if ((zero_body_velocity && leg_stepper->getWalkState() == STOPPING) || leg_stepper->getWalkState() == STOPPED)
  {
    auto_posing_state_ = STOP_POSING;
  }

  // Update master phase
  int master_phase;
  bool sync_with_step_cycle = (pose_frequency_ == -1.0);
  if (sync_with_step_cycle)
  {
    master_phase = leg_stepper->getPhase(); // Correction for calculating auto pose before iterating walk phase
  }
  else
  {
    master_phase = pose_phase_;
    pose_phase_ = (pose_phase_ + 1) % pose_phase_length_; // Iterate pose phase
  }

  // Update auto pose from auto posers
  int auto_posers_complete = 0;
  AutoPoserContainer::iterator auto_poser_it;
  for (auto_poser_it = auto_poser_container_.begin(); auto_poser_it != auto_poser_container_.end(); ++auto_poser_it)
  {
    shared_ptr<AutoPoser> auto_poser = *auto_poser_it;
    Pose updated_pose = auto_poser->updatePose(master_phase);
    auto_posers_complete += int(!auto_poser->isPosing());
    auto_pose_ = auto_pose_.addPose(updated_pose);
  }

  // All auto posers have completed their required posing cycle (Allows walkController to transition to STARTING)
  if (auto_posers_complete == int(auto_poser_container_.size()))
  {
    auto_posing_state_ = POSING_COMPLETE;
  }

  // Update leg specific auto pose using leg posers
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
    leg_poser->updateAutoPose(master_phase);
  }
}

/*******************************************************************************************************************//**
 * Attempts to generate a pose (pitch/roll rotation only) for the robot model to 'correct' any differences between the
 * desired pose rotation and the that estimated by the IMU. A low pass filter is used to smooth out velocity inputs
 * from the IMU and a basic PID controller is used to do control the output pose.
***********************************************************************************************************************/
void PoseController::updateIMUPose(void)
{
  Quaterniond current_rotation = correctRotation(model_->getImuData().orientation, Quaterniond::Identity());
  Quaterniond target_rotation = correctRotation(manual_pose_.rotation_, Quaterniond::Identity());
  Quaterniond rotation_error = (current_rotation * target_rotation.inverse()).normalized();

  // PID gains
  double kp = params_.rotation_pid_gains.data.at("p");
  double ki = params_.rotation_pid_gains.data.at("i");
  double kd = params_.rotation_pid_gains.data.at("d");

  rotation_position_error_ = quaternionToEulerAngles(rotation_error);
  rotation_position_error_[2] = 0.0;
  if (rotation_position_error_.norm() < IMU_POSING_DEADBAND)
  {
    return;
  }

  // Integration of angle position error (absement)
  rotation_absement_error_ += rotation_position_error_ * params_.time_delta.data;

  // Low pass filter of IMU angular velocity data
  double smoothing_factor = 0.15;
  rotation_velocity_error_ = smoothing_factor * -model_->getImuData().angular_velocity +
                             (1 - smoothing_factor) * rotation_velocity_error_;

  Vector3d rotation_correction =  -(kd * rotation_velocity_error_ +
                                    kp * rotation_position_error_ +
                                    ki * rotation_absement_error_);
  
  double max_roll = params_.max_rotation.data.at("roll");
  double max_pitch = params_.max_rotation.data.at("pitch");
  rotation_correction[0] = clamped(rotation_correction[0], -max_roll, max_roll);
  rotation_correction[1] = clamped(rotation_correction[1], -max_pitch, max_pitch);
  rotation_correction[2] = quaternionToEulerAngles(target_rotation)[2];  // No compensation in yaw rotation

  if (rotation_correction.norm() > STABILITY_THRESHOLD)
  {
    ROS_FATAL("IMU rotation compensation became unstable! Adjust PID parameters.\n");
    ros::shutdown();
  }

  imu_pose_.rotation_ = eulerAnglesToQuaternion(rotation_correction);
  imu_pose_.rotation_ = correctRotation(imu_pose_.rotation_, target_rotation);
}

/*******************************************************************************************************************//**
 * Attempts to generate a pose (x/y linear translation only) which shifts the assumed centre of gravity of the body to
 * the vertically projected centre of the support polygon in accordance with the inclination of the terrain.
***********************************************************************************************************************/
void PoseController::updateInclinationPose(void)
{
  Quaterniond compensation_combined = (manual_pose_.rotation_ * auto_pose_.rotation_).normalized();
  Quaterniond compensation_removed = (model_->getImuData().orientation * compensation_combined.inverse()).normalized();
  
  Vector3d euler = quaternionToEulerAngles(compensation_removed);

  double body_height = params_.body_clearance.data;
  double longitudinal_correction = -body_height * tan(euler[1]);
  double lateral_correction = body_height * tan(euler[0]);

  double max_translation_x = params_.max_translation.data.at("x");
  double max_translation_y = params_.max_translation.data.at("y");
  longitudinal_correction = clamped(longitudinal_correction, -max_translation_x, max_translation_x);
  lateral_correction = clamped(lateral_correction, -max_translation_y, max_translation_y);

  inclination_pose_.position_[0] = longitudinal_correction;
  inclination_pose_.position_[1] = lateral_correction;
}

/***********************************************************************************************************************
 * Attempts to generate a pose (x/y linear translation only) to position body such that there is a zero sum of moments
 * from the force acting on the load bearing feet, allowing the robot to shift its centre of mass away from manually
 * manipulated (non-load bearing) legs and remain balanced.
***********************************************************************************************************************/
void PoseController::calculateDefaultPose(void)
{
  int legs_loaded = 0.0;
  int legs_transitioning_states = 0.0;

  // Return early if only one leg in model since pointless
  if (model_->getLegCount() == 1)
  {
    return;
  }

  // Check how many legs are load bearing and how many are transitioning states
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    shared_ptr<Leg> leg = leg_it_->second;
    LegState state = leg->getLegState();

    if (state == WALKING || state == MANUAL_TO_WALKING)
    {
      legs_loaded++;
    }

    if (state == MANUAL_TO_WALKING || state == WALKING_TO_MANUAL)
    {
      legs_transitioning_states++;
    }
  }

  // Only update the sum of moments if specific leg is WALKING and ALL other legs are in WALKING OR MANUAL state.
  if (legs_transitioning_states != 0.0)
  {
    if (recalculate_default_pose_)
    {
      Vector3d zero_moment_offset(0, 0, 0);

      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        shared_ptr<Leg> leg = leg_it_->second;
        shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        LegState state = leg->getLegState();

        if (state == WALKING || state == MANUAL_TO_WALKING)
        {
          zero_moment_offset[0] += leg_stepper->getDefaultTipPose().position_[0];
          zero_moment_offset[1] += leg_stepper->getDefaultTipPose().position_[1];
        }
      }

      double max_translation_x = params_.max_translation.data.at("x");
      double max_translation_y = params_.max_translation.data.at("y");
      zero_moment_offset /= legs_loaded;
      zero_moment_offset[0] = clamped(zero_moment_offset[0], -max_translation_x, max_translation_x);
      zero_moment_offset[1] = clamped(zero_moment_offset[1], -max_translation_y, max_translation_y);

      default_pose_.position_[0] = zero_moment_offset[0];
      default_pose_.position_[1] = zero_moment_offset[1];
      recalculate_default_pose_ = false;
    }
  }
  else
  {
    recalculate_default_pose_ = true;
  }
}

/*******************************************************************************************************************//**
 * Auto poser contructor.
 * @param[in] poser Pointer to the Pose Controller object
 * @param[in] id Int defining the id number of the created Auto Poser object
***********************************************************************************************************************/
AutoPoser::AutoPoser(shared_ptr<PoseController> poser, const int& id)
  : poser_(poser)
  , id_number_(id)
{
}

/*******************************************************************************************************************//**
 * Returns a pose which contributes to the auto pose applied to the robot body. The resultant pose is defined by a 4th
 * order bezier curve for both linear position and angular rotation and iterated along using the phase input argument.
 * The characteristics of each bezier curves are defined by the user parameters in the auto_pose.yaml config file.
 * @param[in] phase The phase is the input value which is used to determine the progression along the bezier curve which
 * defines the output pose.
 * @return The component of auto pose contributed by this Auto Poser object's posing cycle defined by user parameters.
 * @see config/auto_pose.yaml
***********************************************************************************************************************/
Pose AutoPoser::updatePose(int phase)
{
  Pose return_pose = Pose::Identity();
  int start_phase = start_phase_ * poser_->getNormaliser();
  int end_phase = end_phase_ * poser_->getNormaliser();

  //Handles phase overlapping master phase start/end
  if (start_phase > end_phase)
  {
    end_phase += poser_->getPhaseLength();
    if (phase < start_phase)
    {
      phase += poser_->getPhaseLength();
    }
  }

  PosingState state = poser_->getAutoPoseState();
  bool sync_with_step_cycle = (poser_->getPoseFrequency() == -1.0);

  // Coordinates starting/stopping of posing period
  // (posing only ends once a FULL posing cycle completes whilst in STOP_POSING state)
  start_check_ = !sync_with_step_cycle || (!start_check_ && state == POSING && phase == start_phase);
  end_check_.first = (end_check_.first || (state == STOP_POSING && phase == start_phase));
  end_check_.second = (end_check_.second || (state == STOP_POSING && phase == end_phase && end_check_.first));
  if (!allow_posing_ && start_check_) // Start posing
  {
    allow_posing_ = true;
    end_check_ = pair<bool, bool>(false, false);
  }
  else if (allow_posing_ && sync_with_step_cycle && end_check_.first && end_check_.second) // Stop posing
  {
    allow_posing_ = false;
    start_check_ = false;
  }

  // Pose if in correct phase
  if (phase >= start_phase && phase < end_phase && allow_posing_)
  {
    int iteration = phase - start_phase + 1;
    int num_iterations = end_phase - start_phase;

    Vector3d zero(0.0, 0.0, 0.0);
    Vector3d position_control_nodes[5] = {zero, zero, zero, zero, zero};
    Vector3d rotation_control_nodes[5] = {zero, zero, zero, zero, zero};

    bool first_half = iteration <= num_iterations / 2; // Flag for 1st vs 2nd half of posing cycle
    Vector3d gravity_direction = poser_->estimateGravity().normalized();

    if (first_half)
    {
      rotation_control_nodes[3] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      rotation_control_nodes[4] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      if (gravity_amplitude_ != 0.0)
      {
        position_control_nodes[3] = gravity_direction * gravity_amplitude_;
        position_control_nodes[4] = gravity_direction * gravity_amplitude_;
      }
      else
      {
        position_control_nodes[3] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
        position_control_nodes[4] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      }
    }
    else
    {
      rotation_control_nodes[0] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      rotation_control_nodes[1] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      if (gravity_amplitude_ != 0.0)
      {
        position_control_nodes[0] = gravity_direction * gravity_amplitude_;
        position_control_nodes[1] = gravity_direction * gravity_amplitude_;
      }
      else
      {
        position_control_nodes[0] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
        position_control_nodes[1] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      }
    }

    double delta_t = 1.0 / (num_iterations / 2.0);
    int offset = (first_half ? 0 : num_iterations / 2.0); // Offsets iteration count for second half of posing cycle
    double time_input = (iteration - offset) * delta_t;

    Vector3d position = quarticBezier(position_control_nodes, time_input);
    Vector3d rotation = quarticBezier(rotation_control_nodes, time_input);

    return_pose = Pose(position, eulerAnglesToQuaternion(rotation));

    ROS_DEBUG_COND(false,
                   "AUTOPOSE_DEBUG %d - ITERATION: %d\t\t"
                   "TIME: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "POS: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   id_number_, iteration, setPrecision(time_input, 3),
                   position_control_nodes[0][0], position_control_nodes[0][1], position_control_nodes[0][2],
                   position[0], position[1], position[2],
                   position_control_nodes[4][0], position_control_nodes[4][1], position_control_nodes[4][2]);
  }

  return return_pose;
}

/*******************************************************************************************************************//**
 * Leg poser contructor
 * @param[in] poser Pointer to the Pose Controller object
 * @param[in] leg Pointer to the parent leg object associated with the create Leg Poser object
***********************************************************************************************************************/
LegPoser::LegPoser(shared_ptr<PoseController> poser, shared_ptr<Leg> leg)
  : poser_(poser)
  , leg_(leg)
  , auto_pose_(Pose::Identity())
  , current_tip_pose_(Pose::Undefined())
  , target_tip_pose_(Pose::Undefined())
{
}

/*******************************************************************************************************************//**
 * Leg poser copy contructor.
 * @param[in] leg_poser Pointer to the Leg Poser object to be copied from.
***********************************************************************************************************************/
LegPoser::LegPoser(shared_ptr<LegPoser> leg_poser)
  : poser_(leg_poser->poser_)
  , leg_(leg_poser->leg_)
  , auto_pose_(leg_poser->auto_pose_)
  , current_tip_pose_(leg_poser->current_tip_pose_)
{
  pose_negation_phase_start_ = leg_poser->pose_negation_phase_start_;
  pose_negation_phase_end_ = leg_poser->pose_negation_phase_end_;
  negate_auto_pose_ = leg_poser->negate_auto_pose_;
  first_iteration_ = leg_poser->first_iteration_;
  master_iteration_count_ = leg_poser->master_iteration_count_;
  desired_configuration_ = leg_poser->desired_configuration_;
  origin_configuration_ = leg_poser->origin_configuration_;
  origin_tip_pose_ = leg_poser->origin_tip_pose_;
  target_tip_pose_ = leg_poser->target_tip_pose_;
  external_target_ = leg_poser->external_target_;
  transition_poses_ = leg_poser->transition_poses_;
  leg_completed_step_ = leg_poser->leg_completed_step_;
}

/*******************************************************************************************************************//**
 * Uses a bezier curve to smoothly update (over many iterations) the desired joint position of each joint in the leg
 * associated with this Leg Poser object, from the original configuration at the first iteration of this function to
 * the target configuration defined by the pre-set member variable. This transition completes after a time period 
 * defined by the input argument.
 * @param[in] transition_time The time period in which to complete this transition
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int LegPoser::transitionConfiguration(const double& transition_time)
{
  // Return early if desired configuration is undefined
  if (desired_configuration_.name.size() == 0)
  {
    return PROGRESS_COMPLETE;
  }
  
  // Setup origin and target joint positions for bezier curve
  if (first_iteration_)
  {
    origin_configuration_ = sensor_msgs::JointState(); // Clears existing configuration.
    JointContainer::iterator joint_it;
    bool all_joints_at_target = true;
    int i = 0;
    for (joint_it = leg_->getJointContainer()->begin(); joint_it != leg_->getJointContainer()->end(); ++joint_it, ++i)
    {
      shared_ptr<Joint> joint = joint_it->second;
      ROS_ASSERT(desired_configuration_.name[i] == joint->id_name_);
      bool joint_at_target = abs(desired_configuration_.position[i] - joint->desired_position_) < JOINT_TOLERANCE;
      all_joints_at_target = all_joints_at_target && joint_at_target;

      origin_configuration_.name.push_back(joint->id_name_);
      origin_configuration_.position.push_back(joint->desired_position_);
    }

    // Complete early if joint positions are already at target
    if (false)//all_joints_at_target) //TODO
    {
      return PROGRESS_COMPLETE;
    }
    else
    {
      first_iteration_ = false;
      master_iteration_count_ = 0;
    }
  }

  int num_iterations = max(1, int(roundToInt(transition_time / poser_->getParameters().time_delta.data)));
  double delta_t = 1.0 / num_iterations;

  master_iteration_count_++;

  JointContainer::iterator joint_it;
  sensor_msgs::JointState new_configuration;
  int i = 0;
  for (joint_it = leg_->getJointContainer()->begin(); joint_it != leg_->getJointContainer()->end(); ++joint_it, ++i)
  {
    shared_ptr<Joint> joint = joint_it->second;
    double control_nodes[4];
    control_nodes[0] = origin_configuration_.position[i];
    control_nodes[1] = origin_configuration_.position[i];
    control_nodes[2] = desired_configuration_.position[i];
    control_nodes[3] = desired_configuration_.position[i];
    joint->prev_desired_position_ = joint->desired_position_;
    joint->desired_position_ = cubicBezier(control_nodes, master_iteration_count_ * delta_t);
    new_configuration.name.push_back(joint->id_name_);
    new_configuration.position.push_back(joint->desired_position_);
  }

  leg_->applyFK();

  if (poser_->getParameters().debug_moveToJointPosition.data && leg_->getIDNumber() == 0) //reference leg for debugging
  {
    double time = master_iteration_count_ * delta_t;
    string origin_string, current_string, target_string;
    for (uint i = 0; i < new_configuration.name.size(); ++i)
    {
      origin_string += stringFormat("%f\t", origin_configuration_.position[i]);
      current_string += stringFormat("%f\t", new_configuration.position[i]);
      target_string += stringFormat("%f\t", desired_configuration_.position[i]);
    }
    ROS_DEBUG("\nTRANSITION CONFIGURATION DEBUG:\n"
              "\tMASTER ITERATION: %d\n\tTIME: %f\n\tORIGIN: %s\n\tCURRENT: %s\n\tTARGET: %s\n",
              master_iteration_count_, time, origin_string.c_str(), current_string.c_str(), target_string.c_str());
  }

  //Return percentage of progress completion (1%->100%)
  int progress = int((double(master_iteration_count_ - 1) / double(num_iterations)) * PROGRESS_COMPLETE);
  progress = clamped(progress, 1, PROGRESS_COMPLETE); // Ensures 1 percent is minimum return

  // Complete once reached total number of iterations
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }
  else
  {
    return progress;
  }
}

/*******************************************************************************************************************//**
 * Uses bezier curves to smoothly update (over many iterations) the desired tip position of the leg associated with
 * this Leg Poser object, from the original tip position at the first iteration of this function to the target tip
 * position defined by the input argument.
 * @param[in] target_tip_pose The target tip pose in reference to the body centre frame
 * @param[in] target_pose A Pose to be linearly applied to the tip position over the course of the maneuver
 * @param[in] lift_height The height which the stepping leg trajectory should reach at its peak.
 * @param[in] time_to_step The time period to complete this maneuver.
 * @param[in] apply_delta A bool defining if a position offset value (generated by the admittance controller) should
 * be applied to the target tip position.
 * @return Returns an int from 0 to 100 signifying the progress of the sequence (100 meaning 100% complete)
***********************************************************************************************************************/
int LegPoser::stepToPosition(const Pose& target_tip_pose, const Pose& target_pose,
                             const double& lift_height, const double& time_to_step, const bool& apply_delta)
{
  if (first_iteration_)
  {
    origin_tip_pose_ = leg_->getCurrentTipPose();
    current_tip_pose_ = origin_tip_pose_;
    master_iteration_count_ = 0;
    first_iteration_ = false;
  }
  
  Pose desired_tip_pose = target_tip_pose;
  if (desired_tip_pose == Pose::Undefined())
  {
    desired_tip_pose = origin_tip_pose_;
    desired_tip_pose.rotation_ = UNDEFINED_ROTATION; //FIXME
  }
  
  // Check if transition is needed
  Vector3d transition = origin_tip_pose_.position_ - target_pose.inverseTransformVector(desired_tip_pose.position_);
  if (transition.norm() < TIP_TOLERANCE && lift_height == 0.0)
  {
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }

  // Apply delta z to target tip position (used for transitioning to state using admittance control)
  bool manually_manipulated = (leg_->getLegState() == MANUAL || leg_->getLegState()  == WALKING_TO_MANUAL);
  if (apply_delta && !manually_manipulated)
  {
    desired_tip_pose.position_ += leg_->getAdmittanceDelta();
  }

  master_iteration_count_++;

  int num_iterations = max(1, int(roundToInt(time_to_step / poser_->getParameters().time_delta.data)));
  double delta_t = 1.0 / num_iterations;

  double completion_ratio = (double(master_iteration_count_ - 1) / double(num_iterations));

  // Interpolate pose applied to body between identity and target
  Pose desired_pose = Pose::Identity().interpolate(smoothStep(completion_ratio), target_pose);
  
  // Interpolate tip rotation between origin and target (if target is defined)
  Quaterniond new_tip_rotation = UNDEFINED_ROTATION;
  if (!desired_tip_pose.rotation_.isApprox(UNDEFINED_ROTATION))
  {
    Vector3d origin_tip_direction = origin_tip_pose_.rotation_._transformVector(Vector3d::UnitX());
    Vector3d desired_tip_direction = desired_tip_pose.rotation_._transformVector(Vector3d::UnitX());
    Vector3d new_tip_direction = interpolate(origin_tip_direction, desired_tip_direction, smoothStep(completion_ratio));
    new_tip_rotation = Quaterniond::FromTwoVectors(Vector3d::UnitX(), new_tip_direction.normalized());
  }

  double time_input;
  Vector3d new_tip_position = origin_tip_pose_.position_;
  if (desired_tip_pose.position_ != UNDEFINED_POSITION)
  {
    int half_swing_iteration = num_iterations / 2;

    // Update leg tip position
    Vector3d control_nodes_primary[5];
    Vector3d control_nodes_secondary[5];
    Vector3d origin_to_target = origin_tip_pose_.position_ - desired_tip_pose.position_;

    // Control nodes for dual 3d quartic bezier curves
    control_nodes_primary[0] = origin_tip_pose_.position_;
    control_nodes_primary[1] = origin_tip_pose_.position_;
    control_nodes_primary[2] = origin_tip_pose_.position_;
    control_nodes_primary[3] = desired_tip_pose.position_ + 0.75 * origin_to_target;
    control_nodes_primary[4] = desired_tip_pose.position_ + 0.5 * origin_to_target;
    control_nodes_primary[2][2] += lift_height;
    control_nodes_primary[3][2] += lift_height;
    control_nodes_primary[4][2] += lift_height;

    control_nodes_secondary[0] = desired_tip_pose.position_ + 0.5 * origin_to_target;
    control_nodes_secondary[1] = desired_tip_pose.position_ + 0.25 * origin_to_target;
    control_nodes_secondary[2] = desired_tip_pose.position_;
    control_nodes_secondary[3] = desired_tip_pose.position_;
    control_nodes_secondary[4] = desired_tip_pose.position_;
    control_nodes_secondary[0][2] += lift_height;
    control_nodes_secondary[1][2] += lift_height;
    control_nodes_secondary[2][2] += lift_height;

    int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % (num_iterations) + 1;

    // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    if (swing_iteration_count <= half_swing_iteration)
    {
      time_input = swing_iteration_count * delta_t * 2.0;
      new_tip_position = quarticBezier(control_nodes_primary, time_input);
    }
    else
    {
      time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
      new_tip_position = quarticBezier(control_nodes_secondary, time_input);
    }
  }

  if (leg_->getLegState() != MANUAL)
  {
    current_tip_pose_.position_ = desired_pose.inverseTransformVector(new_tip_position);
    current_tip_pose_.rotation_ = new_tip_rotation;
  }

  ROS_DEBUG_COND(poser_->getParameters().debug_stepToPosition.data && leg_->getIDNumber() == 0,
                 "STEP_TO_POSITION DEBUG - LEG: %s\t\t"
                 "MASTER ITERATION: %d\t\t"
                 "TIME INPUT: %f\t\t"
                 "COMPLETION RATIO: %f\t\t"
                 "POSE: %f:%f:%f\t\t"
                 "ORIGIN: %f:%f:%f\t\t"
                 "CURRENT: %f:%f:%f\t\t"
                 "TARGET: %f:%f:%f\n",
                 leg_->getIDName().c_str(), master_iteration_count_, time_input, completion_ratio,
                 target_pose.position_[0], target_pose.position_[0], target_pose.position_[0],
                 origin_tip_pose_.position_[0], origin_tip_pose_.position_[1], origin_tip_pose_.position_[2],
                 current_tip_pose_.position_[0], current_tip_pose_.position_[1], current_tip_pose_.position_[2],
                 desired_tip_pose.position_[0], desired_tip_pose.position_[1], desired_tip_pose.position_[2]);

  //Return ratio of completion (1.0 when fully complete)
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }
  else
  {
    return int(completion_ratio * PROGRESS_COMPLETE);
  }
}

/*******************************************************************************************************************//**
 * Sets the leg specific auto pose from the default auto pose defined by auto pose parameters. The leg specific auto 
 * pose may be negated according to user defined parameters. The negated pose is defined by interpolating from
 * the default auto pose to identity pose (zero pose) and back again over the negation period. The ratio of the
 * period which is used to interpolate to/from is defined by the negation transition ratio parameter.
 * @param[in] phase The phase is the input value which is used to determine the progression along the bezier curves
 * which define the output pose.
***********************************************************************************************************************/
void LegPoser::updateAutoPose(const int& phase)
{
  int start_phase = pose_negation_phase_start_ * poser_->getNormaliser();
  int end_phase = pose_negation_phase_end_ * poser_->getNormaliser();
  int negation_phase = phase;

  // Changes start/end phases from zero to phase length value (which is equivalent)
  if (start_phase == 0)
  {
    start_phase = poser_->getPhaseLength();
  }
  if (end_phase == 0)
  {
    end_phase = poser_->getPhaseLength();
  }

  //Handles phase overlapping master phase start/end
  if (start_phase > end_phase)
  {
    end_phase += poser_->getPhaseLength();
    if (negation_phase < start_phase)
    {
      negation_phase += poser_->getPhaseLength();
    }
  }
  
  // Switch on/off auto pose negation
  StepState step_state = leg_->getLegStepper()->getStepState();
  if (step_state != FORCE_STANCE && step_state != FORCE_STOP && negation_phase == start_phase)
  {
    negate_auto_pose_ = true;
  }
  if (negation_phase < start_phase || negation_phase > end_phase)
  {
    negate_auto_pose_ = false;
  }

  // Assign leg auto pose according to default auto pose
  auto_pose_ = poser_->getAutoPose();

  // Negate auto pose for this leg during negation peroid as defined by parameters
  if (negate_auto_pose_)
  {
    int iteration = negation_phase - start_phase + 1;
    int num_iterations = end_phase - start_phase;
    bool first_half = iteration <= num_iterations / 2;
    double control_input = 1.0;
    if (negation_transition_ratio_ > 0.0)
    {
      if (first_half)
      {
        control_input = min(1.0, iteration / (num_iterations * negation_transition_ratio_));
      }
      else
      {
        control_input = min(1.0, (num_iterations - iteration) / (num_iterations * negation_transition_ratio_));
      }
    }
    control_input = smoothStep(control_input);
    Pose negation = Pose::Identity().interpolate(control_input, auto_pose_);
    auto_pose_ = auto_pose_.removePose(negation);
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
