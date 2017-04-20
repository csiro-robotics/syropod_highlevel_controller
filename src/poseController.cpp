
#include "../include/simple_hexapod_controller/poseController.h"

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
PoseController::PoseController(Model* model, Parameters* params)
  : model_(model)
  , params_(params)
  , manual_pose_(Pose::identity())
  , auto_pose_(Pose::identity())
  , default_pose_(Pose::identity())
{
  rotation_absement_error_ = Vector3d(0, 0, 0);
  rotation_position_error_ = Vector3d(0, 0, 0);
  rotation_velocity_error_ = Vector3d(0, 0, 0);

  translation_absement_error_ = Vector3d(0, 0, 0);
  translation_position_error_ = Vector3d(0, 0, 0);
  translation_velocity_error_ = Vector3d(0, 0, 0);
  translation_acceleration_error_ = Vector3d(0, 0, 0);

  inclination_compensation_offset_ = Vector3d(0, 0, 0);

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    leg->setLegPoser(new LegPoser(this, leg));
  }

  setAutoPoseParams();
}

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
void PoseController::setAutoPoseParams(void)
{
  double raw_phase_length;
  int base_phase_length;
	pose_frequency_ = params_->pose_frequency.data;

  if (pose_frequency_ == -1.0) //Use step cycle parameters
  {
    base_phase_length = params_->stance_phase.data + params_->swing_phase.data;
    double swing_ratio = double(params_->swing_phase.data) / base_phase_length;
    raw_phase_length = ((1.0 / params_->step_frequency.current_value) / params_->time_delta.data) / swing_ratio;
  }
  else
  {
    base_phase_length = params_->pose_phase_length.data;
    raw_phase_length = ((1.0 / pose_frequency_) / params_->time_delta.data);
  }

  pose_phase_length_ = roundToEvenInt(raw_phase_length / base_phase_length) * base_phase_length;
  normaliser_ = pose_phase_length_ / base_phase_length;

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegPoser* leg_poser = leg->getLegPoser();
    leg_poser->setPoseNegationPhaseStart(params_->pose_negation_phase_starts.data[leg->getIDNumber()]);
    leg_poser->setPoseNegationPhaseEnd(params_->pose_negation_phase_ends.data[leg->getIDNumber()]);
  }

  auto_poser_container_.clear();
	for (int i = 0; i < params_->pose_phase_starts.data.size(); ++i)
  {
    auto_poser_container_.push_back(new AutoPoser(this, i));
  }
  
  vector<AutoPoser*>::iterator auto_poser_it;
  for (auto_poser_it = auto_poser_container_.begin(); auto_poser_it != auto_poser_container_.end(); ++auto_poser_it)
  {
    AutoPoser* auto_poser = *auto_poser_it;
    int id = auto_poser->getID();
    auto_poser->setStartPhase(params_->pose_phase_starts.data[id]);
    auto_poser->setEndPhase(params_->pose_phase_ends.data[id]);
    auto_poser->setXAmplitude(params_->x_amplitudes.data[id]);
    auto_poser->setYAmplitude(params_->y_amplitudes.data[id]);
    auto_poser->setZAmplitude(params_->z_amplitudes.data[id]);
    auto_poser->setRollAmplitude(params_->roll_amplitudes.data[id]);
    auto_poser->setPitchAmplitude(params_->pitch_amplitudes.data[id]);
    auto_poser->setYawAmplitude(params_->yaw_amplitudes.data[id]);
    auto_poser->resetChecks();
  }
}

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
void PoseController::updateStance(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    Pose compensation_pose = model_->getCurrentPose();
    LegState leg_state = leg->getLegState();

    if (leg_state == WALKING || leg_state == MANUAL_TO_WALKING)
    {
      // Remove posing compensation from auto_pose under correct conditions
      compensation_pose.position_ -= auto_pose_.position_;
      compensation_pose.rotation_ *= auto_pose_.rotation_.inverse();
      compensation_pose.position_ += leg_poser->getAutoPose().position_;
      compensation_pose.rotation_ *= leg_poser->getAutoPose().rotation_;

      // Apply compensation pose to current walking tip position to calculate new 'posed' tip position
      Vector3d new_tip_position = compensation_pose.inverseTransformVector(leg_stepper->getCurrentTipPosition());
      leg_poser->setCurrentTipPosition(new_tip_position);
    }
    else if (leg_state == MANUAL || leg_state == WALKING_TO_MANUAL)
    {
      leg_poser->setCurrentTipPosition(leg_stepper->getCurrentTipPosition()); //TBD May be redundant, consider cutting
    }
  }
}

/***********************************************************************************************************************
 * Startup sequence
***********************************************************************************************************************/
int PoseController::startUpSequence(void)
{
  int progress = 0;
  
  // Step to raise position OR step to default position if raise has already completed
  if (!step_complete_)
  {
    // Set stepping target
    if (set_target_)
    {
      set_target_ = false;
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        LegStepper* leg_stepper = leg->getLegStepper();
        leg_poser->setLegCompletedStep(false);
        if (!raise_complete_)
        {
          if (leg_poser->getMinLegLength() != UNASSIGNED_VALUE)
          {
            //Set step target as close to default whilst allowing full body raising (i.e. > min leg length)
            Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
            default_tip_position[2] = 0; //Ignore vertical
            double horizontal_target_length = max(leg_poser->getMinLegLength(), default_tip_position.norm());
            double tip_target_bearing = atan2(default_tip_position[1], default_tip_position[0]);
            double new_x_position = horizontal_target_length * cos(tip_target_bearing);
            double new_y_position = horizontal_target_length * sin(tip_target_bearing);
            double new_z_position = !raise_complete_ ? 0.0 : leg->getLocalTipPosition()[2];
            leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, new_z_position));
          }
          else
          {
            // This target used to determine minimum raise/lower leg radius
            leg_poser->setTargetTipPosition(Vector3d(0.0, 0.0, 0.0));
          }
        }
        else
        {
          //Set step target as walking default position
          Vector3d target_tip_position = leg_stepper->getDefaultTipPosition();;
          target_tip_position[2] = leg->getLocalTipPosition()[2];
          leg_poser->setTargetTipPosition(target_tip_position);
        }
      }
    }
    else
    {
      // Step to target
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        LegStepper* leg_stepper = leg->getLegStepper();

        if (leg->getGroup() == current_group_ && !leg_poser->getLegCompletedStep())
        {
          double step_height = !raise_complete_ ? 0.0 : leg_stepper->getSwingHeight();
          double time_to_step = 1.0 / params_->step_frequency.current_value;
          Vector3d target_tip_position = leg_poser->getTargetTipPosition();
          progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, time_to_step);
          leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
          //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
          bool ignore_warnings = leg_poser->getMinLegLength() == UNASSIGNED_VALUE;
          bool within_workspace = leg->applyIK(true, true, ignore_warnings, params_->debug_IK.data);
          // Estimates min raise/lower radius from IK errors
          if (!within_workspace)
          {
            double buffer = 1.05; //5% buffer
            leg_poser->setMinLegLength(setPrecision(leg->getLocalTipPosition().norm()*buffer, 3)); 
            progress = leg_poser->resetStepToPosition();
          }
          leg_poser->setLegCompletedStep(progress == PROGRESS_COMPLETE);
          legs_completed_step_ += int(leg_poser->getLegCompletedStep());
        }
      }
    }

    // Check if next target new stepping/raising target is to be set
    set_target_ = (progress == PROGRESS_COMPLETE);

    // Normalise progress in terms of total start up procedure
    progress = progress / 5 + current_group_ * 20;
    progress += (!raise_complete_ ? 0 : 60); //TBD improve progress calculation

    // Coordinate leg stepping groups
    int num_legs = model_->getLegCount();
    if (legs_completed_step_ == ((num_legs / 2) + (num_legs%2 ? 1:0)) && current_group_ == 0) //Handles odd number of legs
    {
      current_group_ = 1;
      set_target_ = true;
    }
    else if (legs_completed_step_ == num_legs)
    {
      step_complete_ = true;
      legs_completed_step_ = 0;
      current_group_ = 0;
      set_target_ = true;
    }
  }
  // Raise body to default position
  else
  {
    // Set raising target
    if (set_target_)
    {
      set_target_ = false;
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        LegStepper* leg_stepper = leg->getLegStepper();
        Vector3d current_tip_position = leg->getLocalTipPosition();
        double raise_height = leg_stepper->getDefaultTipPosition()[2];
        leg_poser->setTargetTipPosition(Vector3d(current_tip_position[0], current_tip_position[1], raise_height));
      }
    }
    // Raise to target
    else
    {
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        Vector3d target_tip_position = leg_poser->getTargetTipPosition();
        double time_to_step = 1.0 / params_->step_frequency.current_value;
        progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), 0.0, time_to_step);
        leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
        //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
        leg->applyIK(true, true, false, params_->debug_IK.data);
      }
    }

    // Check if raise is complete and order another stepping task
    raise_complete_ = (progress == PROGRESS_COMPLETE);
    step_complete_ = !raise_complete_;

    // Check if next target new stepping/raising target is to be set
    set_target_ = (progress == PROGRESS_COMPLETE);

    // Normalise progress in terms of total start up procedure
    progress = progress / 5 + 40; //TBD improve progress calculation
  }

  // Check if all steps and raises have completed and reset
  if (step_complete_ && raise_complete_)
  {
    step_complete_ = false;
    raise_complete_ = false;
    set_target_ = true;
  }

  return progress;
}

/***********************************************************************************************************************
 * Shutdown sequence
***********************************************************************************************************************/
int PoseController::shutDownSequence(void)
{
  int progress = 0;
  
  // Lower body
  if (!lower_complete_)
  {
    //Set target positions for lowering of body
    if (set_target_)
    {
      set_target_ = false;
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        Vector3d current_tip_position = leg->getLocalTipPosition();
        double horizontal_distance_to_tip = sqrt(sqr(current_tip_position[0]) + sqr(current_tip_position[1]));
        horizontal_distance_to_tip = setPrecision(horizontal_distance_to_tip, 3); //mm precision
        
        // If a leg is not in position to lower fully then skip to stepping
        if (horizontal_distance_to_tip < leg_poser->getMinLegLength())
        {
          lower_complete_ = true;
          set_target_ = true;
          break;
        }
        else
        {
          Vector3d target_tip_position(current_tip_position[0], current_tip_position[1], 0.0);
          leg_poser->setTargetTipPosition(target_tip_position);
        }
      }
    }
    else
    {
      // Lower body to ground
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        double time_to_step = 1.0 / params_->step_frequency.current_value;
        Vector3d target_tip_position = leg_poser->getTargetTipPosition();
        progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), 0.0, time_to_step);
        leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
        //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
        leg->applyIK(true, true, false, params_->debug_IK.data);
        lower_complete_ = (progress == PROGRESS_COMPLETE);
      }

      // Check if next target new stepping/raising target is to be set
      set_target_ = (progress == PROGRESS_COMPLETE);

      // Normalise progress in terms of total start up procedure
      progress = progress / 4;
      progress += (!step_complete_ ? 0 : 75); //TBD improve progress calculation
    }
  }
  // Step to optimal lower position
  else if (!step_complete_)
  {
    // Calculate optimal tip positions for lowering body
    if (set_target_)
    {
      set_target_ = false;
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        LegStepper* leg_stepper = leg->getLegStepper();
        Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
        double min_leg_length = leg_poser->getMinLegLength();
        //Set step target as min leg length distance along walking default tip position bearing
        double tip_target_bearing = atan2(default_tip_position[1], default_tip_position[0]);
        double new_x_position = min_leg_length * cos(tip_target_bearing);
        double new_y_position = min_leg_length * sin(tip_target_bearing);
        leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, leg->getLocalTipPosition()[2]));
      }
    }
    else
    {
      // Step to optimal lowering positions
      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegPoser* leg_poser = leg->getLegPoser();
        LegStepper* leg_stepper = leg->getLegStepper();

        if (leg->getGroup() == current_group_)
        {
          double step_height = leg_stepper->getSwingHeight();
          double time_to_step = 1.0 / params_->step_frequency.current_value;
          Vector3d target_tip_position = leg_poser->getTargetTipPosition();
          progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, time_to_step);
          leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
          //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
          leg->applyIK(true, true, false, params_->debug_IK.data);
          legs_completed_step_ += int(progress == PROGRESS_COMPLETE);
        }
      }
    }

    // Check if next target new stepping/raising target is to be set
    set_target_ = (progress == PROGRESS_COMPLETE);

    // Normalise progress in terms of total start up procedure
    progress = progress / 4 + current_group_ * 25; //TBD improve progress calculation

    // Coordinate leg stepping groups
    int num_legs = model_->getLegCount();
    if (legs_completed_step_ == ((num_legs / 2) + (num_legs%2 ? 1:0))) //Handles odd number of legs
    {
      current_group_ = 1;
    }
    else if (legs_completed_step_ == num_legs)
    {
      step_complete_ = true;
      lower_complete_ = false;
      legs_completed_step_ = 0;
      current_group_ = 0;
    }
  }

  // Check if all steps and lowers have completed and reset
  if (step_complete_ && lower_complete_)
  {
    step_complete_ = false;
    lower_complete_ = false;
    set_target_ = true;
  }

  return progress;
}

/***********************************************************************************************************************
 * Moves tip positions along direct straight path from origin position to target walking stance position
***********************************************************************************************************************/
int PoseController::directStartup(void) //Simultaneous leg coordination
{
  int progress = 0; //Percentage progress (0%->100%)

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegPoser* leg_poser = leg->getLegPoser();
    LegStepper* leg_stepper = leg->getLegStepper();
    double time_to_start = params_->time_to_start.data;
    progress = leg_poser->stepToPosition(leg_stepper->getDefaultTipPosition(), model_->getCurrentPose(), 0.0, time_to_start);
    leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
    //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
    leg->applyIK(true, true, false, params_->debug_IK.data);
  }

  return progress;
}

/***********************************************************************************************************************
 *
***********************************************************************************************************************/
int PoseController::stepToNewStance(void) //Tripod leg coordination
{
  int progress = 0;
  int num_legs = model_->getLegCount();

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;

    if (leg->getGroup() == current_group_)
    {
      LegStepper* leg_stepper = leg->getLegStepper();
      LegPoser* leg_poser = leg->getLegPoser();
      double step_height = leg_stepper->getSwingHeight();
      double step_time = 1.0 / params_->step_frequency.current_value;
      Vector3d target_tip_position = leg_stepper->getDefaultTipPosition();
      progress = leg_poser->stepToPosition(target_tip_position, model_->getCurrentPose(), step_height, step_time);
      leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
      //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
      leg->applyIK(true, true, false, params_->debug_IK.data);
      legs_completed_step_ += int(progress == PROGRESS_COMPLETE);
    }
  }

  // Normalise progress in terms of total procedure
  progress = progress / 2 + current_group_ * 50;

  current_group_ = legs_completed_step_ / (num_legs / 2);

  if (legs_completed_step_ == num_legs)
  {
    legs_completed_step_ = 0;
    current_group_ = 0;
  }

  return progress;
}

/***********************************************************************************************************************
 * Steps tip positions into correct pose for leg manipulation
***********************************************************************************************************************/
int PoseController::poseForLegManipulation(void) //Simultaneous leg coordination
{
  Pose targetPose;
  int progress = 0; //Percentage progress (0%->100%)

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    double step_height = leg_stepper->getSwingHeight();
    double step_time = 1.0 / params_->step_frequency.current_value;

    if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      targetPose = Pose::identity();
      targetPose.position_ += inclination_compensation_offset_;  // Apply inclination control to lifted leg
      targetPose.position_[2] -= step_height;
    }
    // Get target tip positions for legs in WALKING state using default pose
    else
    {
      targetPose = model_->getCurrentPose();
      targetPose.position_ -= manual_pose_.position_;
      targetPose.position_ += default_pose_.position_;
    }

    Vector3d target_tip_position = targetPose.inverseTransformVector(leg_stepper->getDefaultTipPosition());

    if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      leg_stepper->setCurrentTipPosition(target_tip_position);
    }
    else if (leg->getLegState() == MANUAL_TO_WALKING)
    {
      Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
      leg_stepper->setCurrentTipPosition(default_tip_position);
    }

    progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, step_time);
    leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
    //leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
    leg->applyIK(true, true, false, params_->debug_IK.data);
  }

  return progress;
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
int PoseController::packLegs(double time_to_pack) //Simultaneous leg coordination
{
  int progress = 0; //Percentage progress (0%->100%)

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegPoser* leg_poser = leg->getLegPoser();
    vector<double> target_joint_positions;
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      target_joint_positions.push_back(joint_it->second->packed_position);
    }

    progress = leg_poser->moveToJointPosition(target_joint_positions, time_to_pack);
  }

  return progress;
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
int PoseController::unpackLegs(double time_to_unpack) //Simultaneous leg coordination
{
  int progress = 0; //Percentage progress (0%->100%)

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegPoser* leg_poser = leg->getLegPoser();
    vector<double> target_joint_positions;
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      target_joint_positions.push_back(joint_it->second->unpacked_position);
    }

    progress = leg_poser->moveToJointPosition(target_joint_positions, time_to_unpack);
  }

  return progress;
}

/***********************************************************************************************************************
 * Compensation - updates currentPose for body compensation
***********************************************************************************************************************/
void PoseController::updateCurrentPose(double body_height, WalkState walk_state)
{
  Pose new_pose = Pose::identity();

  // Manually set (joystick controlled) body compensation
  if (params_->manual_compensation.data)
  {
    new_pose = manualCompensation();
  }

  // Compensation to align centre of gravity evenly between tip positions on incline
  if (params_->inclination_compensation.data)
  {
    new_pose.position_ += inclinationCompensation(body_height);
  }

  // Compensation to offset average deltaZ from impedance controller and keep body at specificied height
  if (params_->impedance_control.data)
  {
    new_pose.position_[2] += impedanceControllerCompensation();
  }

  // Auto body compensation using IMU feedback
  if (params_->imu_compensation.data)
  {
    new_pose.rotation_ = imuCompensation();
  }
  // Automatic (non-feedback) body compensation
  else if (params_->auto_compensation.data)
  {
    Pose auto_pose = autoCompensation();
    new_pose.position_ += auto_pose.position_;
    new_pose.rotation_ *= auto_pose.rotation_;  //(Quaternion)
  }

  model_->setCurrentPose(new_pose);
}

/***********************************************************************************************************************
 * Calculates pitch/roll/yaw/x,y,z for smooth transition to target pose for manual body compensation
***********************************************************************************************************************/
Pose PoseController::manualCompensation(void)
{
  Vector3d translation_position = manual_pose_.position_;
  Quat rotation_position = manual_pose_.rotation_;

  Vector3d default_translation = default_pose_.position_;
  Vector3d default_rotation = default_pose_.rotation_.toEulerAngles();

  Vector3d max_translation;
  max_translation[0] = params_->max_translation.data["x"];
  max_translation[1] = params_->max_translation.data["y"];
  max_translation[2] = params_->max_translation.data["z"];
  Vector3d max_rotation;
  max_rotation[0] = params_->max_rotation.data["roll"];
  max_rotation[1] = params_->max_rotation.data["pitch"];
  max_rotation[2] = params_->max_rotation.data["yaw"];

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
      return manual_pose_;

    case (NO_RESET):  // Do nothing
    default:  // Do nothing
      break;
  }

  // Override posing velocity commands depending on pose reset mode
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    if (reset_translation[i])
    {
      if (translation_position[i] < default_translation[i])
      {
        translation_velocity_input_[i] = 1.0;
      }
      else if (translation_position[i] > default_translation[i])
      {
        translation_velocity_input_[i] = -1.0;
      }
    }

    if (reset_rotation[i])
    {
      if (rotation_position.toEulerAngles()[i] < default_rotation[i])
      {
        rotation_velocity_input_[i] = 1.0;
      }
      else if (rotation_position.toEulerAngles()[i] > default_rotation[i])
      {
        rotation_velocity_input_[i] = -1.0;
      }
    }
  }

  Vector3d translation_velocity = clamped(translation_velocity_input_, 1.0) * params_->max_translation_velocity.data;
  Vector3d rotation_velocity = clamped(rotation_velocity_input_, 1.0) * params_->max_rotation_velocity.data;

  Vector3d new_translation_position = translation_position + translation_velocity * params_->time_delta.data;
  Quat new_rotation_position = rotation_position * Quat(Vector3d(rotation_velocity * params_->time_delta.data));

  Vector3d translation_limit = Vector3d(0, 0, 0);
  Vector3d rotation_limit = Vector3d(0, 0, 0);

  // Zero velocity input depending on position limitations
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    // TRANSLATION
    // Assign correct translation limit based on velocity direction and reset command
    translation_limit[i] = sign(translation_velocity[i]) * max_translation[i];

    if (reset_translation[i] &&
        default_translation[i] < max_translation[i] && default_translation[i] > -max_translation[i])
    {
      translation_limit[i] = default_translation[i];
    }

    bool positive_translation_velocity = sign(translation_velocity[i]) > 0;
    bool exceeds_positive_translation_limit = positive_translation_velocity && (new_translation_position[i] > translation_limit[i]);
    bool exceeds_negative_translation_limit = !positive_translation_velocity && (new_translation_position[i] < translation_limit[i]);

    // Zero velocity when translation position reaches limit
    if (exceeds_positive_translation_limit || exceeds_negative_translation_limit)
    {
      translation_velocity[i] = (translation_limit[i] - translation_position[i]) / params_->time_delta.data;
    }

    // ROTATION
    // Assign correct rotation limit based on velocity direction and reset command
    rotation_limit[i] = sign(rotation_velocity[i]) * max_rotation[i];

    if (reset_rotation[i] && default_rotation[i] < max_rotation[i] && default_rotation[i] > -max_rotation[i])
    {
      rotation_limit[i] = default_rotation[i];
    }

    bool positive_rotation_velocity = sign(rotation_velocity[i]) > 0;
    bool exceeds_positive_rotation_limit = positive_rotation_velocity && (new_rotation_position.toEulerAngles()[i] > rotation_limit[i]);
    bool exceeds_negative_rotation_limit = !positive_rotation_velocity && (new_rotation_position.toEulerAngles()[i] < rotation_limit[i]);

    // Zero velocity when rotation position reaches limit
    if (exceeds_positive_rotation_limit || exceeds_negative_rotation_limit)
    {
      rotation_velocity[i] = (rotation_limit[i] - rotation_position.toEulerAngles()[i]) / params_->time_delta.data;
    }
  }

  // Update position according to limitations
  manual_pose_.position_ = translation_position + translation_velocity * params_->time_delta.data;
  manual_pose_.rotation_ = rotation_position * Quat(Vector3d(rotation_velocity * params_->time_delta.data));
  // BUG: ^Adding pitch and roll simultaneously adds unwanted yaw

  return manual_pose_;
}

/***********************************************************************************************************************
 * Calculates pitch/roll for smooth auto body compensation from offset pose
***********************************************************************************************************************/
Pose PoseController::autoCompensation(void)
{
  Leg* leg = model_->getLegByIDNumber(0); //Reference leg
  LegStepper* leg_stepper = leg->getLegStepper();
  auto_pose_ = Pose::identity();

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
    master_phase = leg_stepper->getPhase();
  }
  else
  {
    master_phase = pose_phase_;
    pose_phase_ = (pose_phase_ + 1) % pose_phase_length_; // Iterate pose phase
  }

  // Update auto pose from auto posers
  vector<AutoPoser*>::iterator auto_poser_it;
  for (auto_poser_it = auto_poser_container_.begin(); auto_poser_it != auto_poser_container_.end(); ++auto_poser_it)
  {
    AutoPoser* auto_poser = *auto_poser_it;
    Pose updated_pose = auto_poser->updatePose(master_phase);
    auto_pose_.position_ += updated_pose.position_;
    auto_pose_.rotation_ *= updated_pose.rotation_;
    // BUG: ^Adding pitch and roll simultaneously adds unwanted yaw
  }

  // Update leg specific auto pose using leg posers
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegPoser* leg_poser = leg->getLegPoser();
    leg_poser->updateAutoPose(master_phase);
  }

  return auto_pose_;
}

/***********************************************************************************************************************
 * Returns rotation correction used to attempt to match target rotation (manual rotation) using PID controller
***********************************************************************************************************************/
Quat PoseController::imuCompensation(void)
{
  Quat target_rotation = manual_pose_.rotation_;

  // There are two orientations per quaternion and we want the shorter/smaller difference.
  double dot = target_rotation.dot(~imu_data_.orientation);
  if (dot < 0.0)
  {
    target_rotation = -target_rotation;
  }

  // PID gains
  double kp = params_->rotation_pid_gains.data["p"];
  double ki = params_->rotation_pid_gains.data["i"];
  double kd = params_->rotation_pid_gains.data["d"];

  rotation_position_error_ = imu_data_.orientation.toEulerAngles() - target_rotation.toEulerAngles();
  // Integration of angle position error (absement)
  rotation_absement_error_ += rotation_position_error_ * params_->time_delta.data;

  // Low pass filter of IMU angular velocity data
  double smoothingFactor = 0.15;
  rotation_velocity_error_ = smoothingFactor * imu_data_.angular_velocity + (1 - smoothingFactor) * rotation_velocity_error_;

  Vector3d rotation_correction = -(kd * rotation_velocity_error_ + kp * rotation_position_error_ + ki * rotation_absement_error_);
  rotation_correction[2] = target_rotation.toEulerAngles()[2];  // No compensation in yaw rotation

  double stability_threshold = 100; //TBD Magic number

  if (rotation_correction.norm() > stability_threshold)
  {
    ROS_FATAL("IMU rotation compensation became unstable! Adjust PID parameters.\n");
    ros::shutdown();
  }

  return Quat(rotation_correction);
}

/***********************************************************************************************************************
 * Updates inclination pose with translation correction to move centre of body according to inclination of terrain
***********************************************************************************************************************/
Vector3d PoseController::inclinationCompensation(double body_height)
{
  Quat compensation_combined = manual_pose_.rotation_ * auto_pose_.rotation_;
  Quat compensation_removed = imu_data_.orientation * compensation_combined.inverse();
  Vector3d euler_angles = compensation_removed.toEulerAngles();

  double lateral_correction = body_height * tan(euler_angles[0]);
  double longitudinal_correction = -body_height * tan(euler_angles[1]);

  longitudinal_correction = clamped(longitudinal_correction, -params_->max_translation.data["x"], params_->max_translation.data["x"]);
  lateral_correction = clamped(lateral_correction, -params_->max_translation.data["y"], params_->max_translation.data["y"]);

  inclination_compensation_offset_[0] = longitudinal_correction;
  inclination_compensation_offset_[1] = lateral_correction;

  return inclination_compensation_offset_;
}

/***********************************************************************************************************************
 * Calculates mean delta_z value of all legs and returns an offset used to sustain body at correct height
***********************************************************************************************************************/
double PoseController::impedanceControllerCompensation(void)
{
  int loaded_legs = model_->getLegCount();
  double average_delta_z = 0.0;

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    average_delta_z += leg->getDeltaZ();
  }

  average_delta_z /= loaded_legs;

  return clamped(abs(average_delta_z), -params_->max_translation.data["z"], params_->max_translation.data["z"]);
}

/***********************************************************************************************************************
 * Attempts to develop pose to position body such that there is a zero sum of moments from the force acting on the load
 * bearing feet
***********************************************************************************************************************/
void PoseController::calculateDefaultPose(void)
{
  int legs_loaded = 0.0;
  int legs_transitioning_states = 0.0;

  // Check how many legs are load bearing and how many are transitioning states
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
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
    if (recalculate_offset_)
    {
      Vector3d zero_moment_offset(0, 0, 0);

      for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
        Leg* leg = leg_it_->second;
        LegStepper* leg_stepper = leg->getLegStepper();
        LegState state = leg->getLegState();

        if (state == WALKING || state == MANUAL_TO_WALKING)
        {
          zero_moment_offset[0] += leg_stepper->getDefaultTipPosition()[0];
          zero_moment_offset[1] += leg_stepper->getDefaultTipPosition()[1];
        }
      }

      zero_moment_offset /= legs_loaded;
      zero_moment_offset[0] = clamped(zero_moment_offset[0], -params_->max_translation.data["x"], params_->max_translation.data["x"]);
      zero_moment_offset[1] = clamped(zero_moment_offset[1], -params_->max_translation.data["y"], params_->max_translation.data["y"]);

      default_pose_.position_[0] = zero_moment_offset[0];
      default_pose_.position_[1] = zero_moment_offset[1];
      recalculate_offset_ = false;
    }
  }
  else
  {
    recalculate_offset_ = true;
  }
}

/***********************************************************************************************************************
 * Auto poser contructor
***********************************************************************************************************************/
AutoPoser::AutoPoser(PoseController* poser, int id)
  : poser_(poser)
  , id_(id)
{
}

/***********************************************************************************************************************
 * Auto poser contructor
***********************************************************************************************************************/
Pose AutoPoser::updatePose(int phase)
{
  Pose return_pose = Pose::identity();
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
  if (phase >= start_phase && phase <= end_phase && allow_posing_)
  {
    int iteration = phase - start_phase;
    int num_iterations = end_phase - start_phase;

    Vector3d zero(0.0, 0.0, 0.0);
    Vector3d position_control_nodes[5] = {zero, zero, zero, zero, zero};
    Vector3d rotation_control_nodes[5] = {zero, zero, zero, zero, zero};

    if (iteration <= num_iterations / 2)
    {
      position_control_nodes[3] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      position_control_nodes[4] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      rotation_control_nodes[3] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      rotation_control_nodes[4] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
    }
    else
    {
      position_control_nodes[0] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      position_control_nodes[1] = Vector3d(x_amplitude_, y_amplitude_, z_amplitude_);
      rotation_control_nodes[0] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
      rotation_control_nodes[1] = Vector3d(roll_amplitude_, pitch_amplitude_, yaw_amplitude_);
    }

    double delta_t = 2.0 / num_iterations;
    int extra_iteration = (iteration <= num_iterations / 2) ? 0 : 1;
    double time_input = (iteration + extra_iteration) % (1 + num_iterations / 2) * delta_t;

    Vector3d position = quarticBezier(position_control_nodes, time_input);
    Vector3d rotation = quarticBezier(rotation_control_nodes, time_input);

    return_pose = Pose(position, Quat(rotation));
  }

  return return_pose;
}

/***********************************************************************************************************************
 * Leg poser contructor
***********************************************************************************************************************/
LegPoser::LegPoser(PoseController* poser, Leg* leg)
  : poser_(poser)
  , leg_(leg)
  , origin_pose_(Pose::identity())
  , auto_pose_(Pose::identity())
  , current_tip_position_(Vector3d(0, 0, 0))
{
}

/***********************************************************************************************************************
 * Move joints of leg to target positions
***********************************************************************************************************************/
int LegPoser::moveToJointPosition(vector<double> target_joint_positions, double time_to_move)
{
  // Setup origin and target joint positions for bezier curve
  if (first_iteration_)
  {
    first_iteration_ = false;
    master_iteration_count_ = 0;
    origin_joint_positions_.clear();
    std::map<int, Joint*>::iterator joint_it;

    for (joint_it = leg_->getJointContainer()->begin(); joint_it != leg_->getJointContainer()->end(); ++joint_it)
    {
      origin_joint_positions_.push_back(joint_it->second->current_position);
    }
  }

  int num_iterations = max(1, int(roundToInt(time_to_move / poser_->getParameters()->time_delta.data)));
  double delta_t = 1.0 / num_iterations;

  master_iteration_count_++;

  std::map<int, Joint*>::iterator joint_it;
  int i = 0;
  vector<double> new_joint_positions;

  for (joint_it = leg_->getJointContainer()->begin(); joint_it != leg_->getJointContainer()->end(); ++joint_it, ++i)
  {
    Joint* joint = joint_it->second;
    double control_nodes[4];
    control_nodes[0] = origin_joint_positions_[i];
    control_nodes[1] = origin_joint_positions_[i];
    control_nodes[2] = target_joint_positions[i];
    control_nodes[3] = target_joint_positions[i];
    joint->desired_position = cubicBezier(control_nodes, master_iteration_count_ * delta_t);
    new_joint_positions.push_back(joint->desired_position);
  }

  leg_->applyFK();

  if (leg_->getIDNumber() == 0) //reference leg for debugging
  {
    double time = master_iteration_count_ * delta_t;
    bool debug = poser_->getParameters()->debug_moveToJointPosition.data;
    ROS_DEBUG_COND(debug, "MOVE_TO_JOINT_POSITION DEBUG - MASTER ITERATION: %d\t\t"
                   "TIME: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "CURRENT: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   master_iteration_count_, time,
                   origin_joint_positions_[0], origin_joint_positions_[1], origin_joint_positions_[2],
                   new_joint_positions[0], new_joint_positions[1], new_joint_positions[2],
                   target_joint_positions[0], target_joint_positions[1], target_joint_positions[2]);
  }

  //Return percentage of progress completion (1.0 when fully complete)
  int progress = int((double(master_iteration_count_ - 1) / double(num_iterations)) * 100);

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

/***********************************************************************************************************************
 * Step leg tip position to target tip position
***********************************************************************************************************************/
int LegPoser::stepToPosition(Vector3d target_tip_position, Pose target_pose, double lift_height, double time_to_step)
{
  if (first_iteration_)
  {
    origin_tip_position_ = leg_->getLocalTipPosition();
    //origin_tip_position_[2] += delta_z;  // Remove deltaZ offset temporarily //TBD
    double tolerance = 0.001; // 1mm
    if (abs(origin_tip_position_[0] - target_tip_position[0]) < tolerance &&
      abs(origin_tip_position_[1] - target_tip_position[1]) < tolerance &&
      abs(origin_tip_position_[2] - target_tip_position[2]) < tolerance)
    {
      return PROGRESS_COMPLETE; //Target and origin are approximately equal so no need to proceed with step
    }
    current_tip_position_ = origin_tip_position_;
    master_iteration_count_ = 0;
    first_iteration_ = false;
  }

  master_iteration_count_++;

  int num_iterations = max(1, int(roundToInt(time_to_step / poser_->getParameters()->time_delta.data)));
  double delta_t = 1.0 / num_iterations;

  double completion_ratio = (double(master_iteration_count_ - 1) / double(num_iterations));

//   // Applies required posing slowly over course of transition
  // Scales position vector by 0->1.0
  target_pose.position_ *= completion_ratio;
  // Scales rotation quat by 0.0->1.0 (https://en.wikipedia.org/wiki/Slerp)
  target_pose.rotation_ = Pose::identity().rotation_.slerpTo(target_pose.rotation_, completion_ratio);

  int half_swing_iteration = num_iterations / 2;

  // Update leg tip position
  Vector3d control_nodes_primary[4];
  Vector3d control_nodes_secondary[4];

  // Control nodes for dual 3d cubic bezier curves
  control_nodes_primary[0] = origin_tip_position_;
  control_nodes_primary[1] = control_nodes_primary[0];
  control_nodes_primary[3] = 0.5 * (target_tip_position + origin_tip_position_);
  control_nodes_primary[3][2] += lift_height;
  control_nodes_primary[2] = control_nodes_primary[0];
  control_nodes_primary[2][2] += lift_height;

  control_nodes_secondary[0] = control_nodes_primary[3];
  control_nodes_secondary[1] = 2 * control_nodes_secondary[0] - control_nodes_primary[2];
  control_nodes_secondary[3] = target_tip_position;
  control_nodes_secondary[2] = control_nodes_secondary[3];

  Vector3d new_tip_position;
  int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % (num_iterations) + 1;
  double time_input;

  // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
  if (swing_iteration_count <= half_swing_iteration)
  {
    time_input = swing_iteration_count * delta_t * 2.0;
    new_tip_position = cubicBezier(control_nodes_primary, time_input);
  }
  else
  {
    time_input = (swing_iteration_count - half_swing_iteration) * delta_t * 2.0;
    new_tip_position = cubicBezier(control_nodes_secondary, time_input);
  }

  if (leg_->getIDNumber() == 1) //Reference leg for debugging (AR)
  {
    ROS_DEBUG_COND(poser_->getParameters()->debug_stepToPosition.data,
                   "STEP_TO_POSITION DEBUG - LEG: %s\t\t"
                   "MASTER ITERATION: %d\t\t"
                   "TIME INPUT: %f\t\t"
                   "ORIGIN: %f:%f:%f\t\t"
                   "CURRENT: %f:%f:%f\t\t"
                   "TARGET: %f:%f:%f\n",
                   leg_->getIDName().c_str(),
                   master_iteration_count_,
                   time_input,
                   origin_tip_position_[0], origin_tip_position_[1], origin_tip_position_[2],
                   new_tip_position[0], new_tip_position[1], new_tip_position[2],
                   target_tip_position[0], target_tip_position[1], target_tip_position[2]);
  }

  if (leg_->getLegState() != MANUAL)
  {
    current_tip_position_ = target_pose.inverseTransformVector(new_tip_position);
  }

  //Return ratio of completion (1.0 when fully complete)
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return PROGRESS_COMPLETE;
  }
  else
  {
    return int(completion_ratio * 100);
  }
}

/***********************************************************************************************************************
 * Updates leg specific auto pose
***********************************************************************************************************************/
void LegPoser::updateAutoPose(int phase)
{
  int phase_start = pose_negation_phase_start_ * poser_->getNormaliser();
  int phase_end = pose_negation_phase_end_ * poser_->getNormaliser();
  int negation_phase = phase;

  //Handles phase overlapping master phase start/end
  if (phase_start > phase_end)
  {
    phase_end += poser_->getPhaseLength();
    if (negation_phase < phase_start)
    {
      negation_phase += poser_->getPhaseLength();
    }
  }

  if (negation_phase >= phase_start && negation_phase < phase_end && !stop_negation_)
  {
    int iteration = negation_phase - phase_start + 1;
    int num_iterations = phase_end - phase_start;

    Vector3d zero(0.0, 0.0, 0.0);
    Vector3d position_amplitude = poser_->getAutoPose().position_;
    Vector3d rotation_amplitude = poser_->getAutoPose().rotation_.toEulerAngles();
    Vector3d position_control_nodes[5] = {zero, zero, zero, zero, zero};
    Vector3d rotation_control_nodes[5] = {zero, zero, zero, zero, zero};

    if (iteration <= num_iterations / 2)
    {
      position_control_nodes[2] = position_amplitude;
      position_control_nodes[3] = position_amplitude;
      position_control_nodes[4] = position_amplitude;
      rotation_control_nodes[2] = rotation_amplitude;
      rotation_control_nodes[3] = rotation_amplitude;
      rotation_control_nodes[4] = rotation_amplitude;
    }
    else
    {
      position_control_nodes[0] = position_amplitude;
      position_control_nodes[1] = position_amplitude;
      position_control_nodes[2] = position_amplitude;
      rotation_control_nodes[0] = rotation_amplitude;
      rotation_control_nodes[1] = rotation_amplitude;
      rotation_control_nodes[2] = rotation_amplitude;
    }

		double delta_t = 2.0 / num_iterations;
    int extra_iteration = (iteration <= num_iterations / 2) ? 0 : 1;
    double time_input = (iteration + extra_iteration) % (1 + num_iterations / 2) * delta_t;

    Vector3d position = quarticBezier(position_control_nodes, time_input);
    Vector3d rotation = quarticBezier(rotation_control_nodes, time_input);

    auto_pose_ = poser_->getAutoPose();
    auto_pose_.position_ -= position;
    auto_pose_.rotation_ *= Quat(rotation).inverse();
  }
  else
  {
    bool sync_with_step_cycle = (poser_->getPoseFrequency() == -1.0);
    stop_negation_ = (sync_with_step_cycle && poser_->getAutoPoseState() == STOP_POSING);
    auto_pose_ = poser_->getAutoPose();
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
