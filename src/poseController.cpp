
#include "../include/simple_hexapod_controller/poseController.h"

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
PoseController::PoseController(Model *model, Parameters* params)
  : model_(model)
  , params_(params)
  , manual_pose_(Pose::identity())
  , default_pose_(Pose::identity())
  , auto_pose_(Pose::identity())
{
  rotation_absement_error = Vector3d(0, 0, 0);
  rotation_position_error = Vector3d(0, 0, 0);
  rotation_velocity_error = Vector3d(0, 0, 0);

  translationAbsementError = Vector3d(0, 0, 0);
  translationPositionError = Vector3d(0, 0, 0);
  translationVelocityError = Vector3d(0, 0, 0);
  translationAccelerationError = Vector3d(0, 0, 0);
  
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    int index = leg->getIDNumber();
    leg->setLegPoser(this, params_->packed_joint_positions[index], params_->unpacked_joint_positions[index]);
  }  
}

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
void PoseController::updateStance(void)
{
  bool exclude_swinging_legs = params_->auto_compensation && !params_->imu_compensation; 
  
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    Pose compensation_pose = model_->getCurrentPose();
    LegState leg_state = leg->getLegState();
    StepState step_state = leg_stepper->getStepState();
    
    if (leg_state == WALKING || leg_state == MANUAL_TO_WALKING)
    {
      // Remove posing compensation from auto_pose for swinging legs under correct conditions
      if (exclude_swinging_legs && step_state == SWING)
      {
	compensation_pose -= auto_pose_.position_;
	compensation_pose *= auto_pose_.rotation_.inverse();	
      }
      
      // Apply compensation pose to default walking tip position to calculate new 'posed' tip position
      Vector3d new_tip_position = compensation_pose.inverseTransformVector(leg_stepper->getDefaultTipPosition());
      leg_poser->setCurrentTipPosition(new_tip_position);
    }
    else if (leg_state == MANUAL || leg_state == WALKING_TO_MANUAL)
    {
      leg_poser->setCurrentTipPosition(leg_stepper->getDefaultTipPosition()); //TBD May be redundant, consider cutting
    }
  }
}

/***********************************************************************************************************************
 * Startup sequence
***********************************************************************************************************************/
bool PoseController::startUpSequence(void)
{
  double res;  
  if (!step_complete_)
  {
    // Step to raise position OR step to default position if raise has already completed
    // Tripod step syncronisation
    for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg3DOF* leg = leg_it_->second();
      LegPoser* leg_poser = leg->getLegPoser();
      LegStepper* leg_stepper = leg->getLegStepper();
      if (set_target_)
      {
	Vector3d default_tip_position = leg_stepper.getDefaultTipPosition();
	if (raise_complete_)
	{
	  //Set step target as walking default position (on ground)
	  Vector3d target_tip_position = default_tip_position;
	  target_tip_position[2] = leg->getLocalTipPosition()[2];
	  leg_poser->setTargetTipPosition(target_tip_position);
	}
	else
	{
	  //Set step target as min leg length distance along walking default position vector (on ground)
	  double gradient = default_tip_position[1] / default_tip_position[0];
	  double new_x_position = sqrt(sqr(leg->getMinLegLength()) / (1 + sqr(gradient)));
	  double new_y_position = sqrt(sqr(leg->getMinLegLength()) / (1 + sqr(1 / gradient)));
	  leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, leg->getLocalTipPosition()[2]));
	}
      }
      
      if (leg->getTripodGroup() == current_group_)
      {
	double step_height = leg_stepper->getSwingHeight();
	double time_to_step = 1.0 / params_->step_frequency;
	Vector3d target_tip_position = leg_poser->getTargetTipPosition();
	res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, time_to_step);
	legs_completed_task_ += int(res == 1.0);
      }
    }
    if (legs_completed_task_ == 3)
    {
      current_group_ = 1;
    }   
    else if (legs_completed_task_ == model_->getLegCount())
    {
      legs_completed_task_ = 0;
      step_complete_ = true;
    }
  }
  else
  {
    // Raise body to default position
    for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg3DOF* leg = leg_it_->second();
      LegPoser* leg_poser = leg->getLegPoser();
      LegStepper* leg_stepper = leg->getLegStepper();
      if (set_target_)
      {
	Vector3d current_tip_position = leg->getLocalTipPosition();
	double horizontal_distance_to_tip = sqrt(sqr(current_tip_position[0]) + sqr(current_tip_position[1]));
	double raise_height = leg_stepper->getDefaultTipPosition()[2];
	leg_poser->setTargetTipPosition(Vector3d(current_tip_position[0], current_tip_position[1], raise_height));
      }
      Vector3d target_tip_position = leg_poser->getTargetTipPosition();
      double time_to_step = 1.0 / params_->step_frequency;
      res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), 0.0, time_to_step);
    }
    raise_complete_ = (res == 1.0);
    step_complete_ = !raise_complete_;
  }
  set_target_ = (res == 1.0);  
  if (step_complete_ && raise_complete_)
  {
    step_complete_ = false;
    raise_complete_ = false;
    set_target_ = true;
    return true;
  }
  else
  {
    return false;
  }
}

/***********************************************************************************************************************
 * Shutdown sequence
***********************************************************************************************************************/
bool PoseController::shutDownSequence(void)
{ 
  double res;
  if (!lower_complete_)
  {
    //Set target positions for lowering of body
    if (set_target_)
    {      
      //Find the smallest body height all legs can achieve
      double target_body_height = 0; // On ground
      for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->second();
	LegPoser* leg_poser = leg->getLegPoser();	
	
	//Calculate the minimum body height that this leg can lower the body to, along a vertical trajectory
	double min_body_height = 0; // Assume leg can lower body to ground
	Vector3d current_tip_position = leg->getLocalTipPosition();
	double horizontal_distance_to_tip = sqrt(sqr(current_tip_position[0]), sqr(current_tip_position[1]));
	double min_leg_length = leg->getMinLegLength();
	if (min_leg_length > horizontal_distance_to_tip)
	{
	  min_body_height = sqrt(sqr(min_leg_length) - sqr(horizontal_distance_to_tip));
	}
	target_body_height = max(target_body_height, min_body_height);
      }
      // Set leg target position for calculated body height
      for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->second();
	LegPoser* leg_poser = leg->getLegPoser();
	Vector3d current_tip_position = leg->getLocalTipPosition();
	leg_poser->setTargetTipPosition(current_tip_position[0], current_tip_position[1], -target_body_height);
      }
    }
    
    // Lower body as much as possible from current position
    for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg3DOF* leg = leg_it_->second();
      LegPoser* leg_poser = leg->getLegPoser();
      double time_to_step = 1.0 / params_->step_frequency;
      Vector3d target_tip_position = leg_poser->getTargetTipPosition();
      res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), 0.0, time_to_step);
      lower_complete_ = (res == 1.0);
    }
  }
  else
  {
    // Step to optmial lowering position
    for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg3DOF* leg = leg_it_->second();
      LegPoser* leg_poser = leg->getLegPoser();
      LegStepper* leg_stepper = leg->getLegStepper();
      double min_leg_length = leg->getMinLegLength();
      if (set_target_)
      {
	//Find new tip position if stepping a distance of min_leg_length, along same direction out from body centre
	Vector3d current_tip_position = leg->getLocalTipPosition();
	double gradient = current_tip_position[1] / current_tip_position[0];
	double new_x_position = sqrt(sqr(min_leg_length) / (1 + sqr(gradient)));
	double new_y_position = sqrt(sqr(min_leg_length) / (1 + sqr(1 / gradient)));
	leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, current_tip_position[2]));
      }
      
      if (leg->getTripodGroup() == current_group_)
      {
	double step_height = leg_stepper->getSwingHeight();
	double time_to_step = 1.0 / params_->step_frequency;
	Vector3d target_tip_position = leg_poser->getTargetTipPosition();
	res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, time_to_step);
	legs_completed_task_ += int(res == 1.0);
      }
    }
    if (legs_completed_task_ == 3)
    {
      current_group_ = 1;
    }   
    else if (legs_completed_task_ == model_->getLegCount())
    {
      legs_completed_task_ = 0;
      step_complete_ = true;
      lower_complete_ = false;
    }
  }
  set_target_ = (res == 1.0);  
  if (step_complete_ && lower_complete_)
  {
    step_complete_ = false;
    lower_complete_ = false;
    set_target_ = true;
    return true;
  }
  else
  {
    return false;
  }
}

/***********************************************************************************************************************
 * Moves tip positions along direct straight path from origin position to target walking stance position
***********************************************************************************************************************/ 
double PoseController::directStartup(void) //Simultaneous leg coordination
{
  double res;
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();      
    LegPoser* leg_poser = leg->getLegPoser();
    LegStepper* leg_stepper = leg->getLegStepper();
    Vector3d target_tip_position = leg_stepper->getDefaultTipPosition();
    res = leg_poser->stepToPosition(target_tip_position, model_->getCurrentPose(), 0.0,  params_->time_to_start);
  }
  return res;
}

/***********************************************************************************************************************
 * 
***********************************************************************************************************************/ 
double PoseController::stepToNewStance(void) //Tripod leg coordination
{    
  int num_legs = model_->getLegCount();
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    if (leg->getTripodGroup() == current_group_)
    {
      LegStepper* leg_stepper = leg->getLegStepper();
      LegPoser* leg_poser = leg->getLegPoser();
      double step_height = leg_stepper->getSwingHeight();  
      Vector3d target_tip_position = leg_stepper->getDefaultTipPosition();
      double result = leg_poser->stepToPosition(target_tip_position, model_->getCurrentPose(), step_height, 1.0 / params_->step_frequency);
      if (result == 1.0)
      {
	legs_completed_task_ ++;
      }
    }
  }
  if (legs_completed_task_ == 3)
  {
    current_group_++;
  }   
  else if (legs_completed_task_ == num_legs)
  {
    legs_completed_task_ = 0;
  }
  return (legs_completed_task_ == num_legs);
}

/***********************************************************************************************************************
 * Steps tip positions into correct pose for leg manipulation
***********************************************************************************************************************/
double PoseController::poseForLegManipulation(void) //Simultaneous leg coordination
{
  Pose targetPose;
  double res = 0;
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    double step_height = leg_stepper->getSwingHeight();
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
    res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, 1.0 / params_->step_frequency);
  }
  return res;
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
bool PoseController::packLegs(double time_to_pack) //Simultaneous leg coordination
{
  double progress = 0.0;
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegPoser* leg_poser = leg->getLegPoser();	
    Vector3d target_joint_position = leg_poser->getPackedJointPositions();
    progress = leg_poser->moveToJointPosition(target_joint_position, time_to_pack);
  }
  return progress == 1.0;
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
bool PoseController::unpackLegs(double time_to_unpack) //Simultaneous leg coordination
{
  double progress = 0.0;
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegPoser* leg_poser = leg->getLegPoser();	
    Vector3d target_joint_position = leg_poser->getUnpackedJointPositions();
    progress = leg_poser->moveToJointPosition(target_joint_position, time_to_unpack);
  }
  return progress == 1.0;
}

/***********************************************************************************************************************
 * Calculates pitch/roll for smooth auto body compensation from offset pose
***********************************************************************************************************************/
Pose PoseController::autoCompensation(void)
{
  double swing_height_progress = 1.0;
  vector<double> roll_values;
  vector<double> pitch_values;
  vector<double> z_trans_values;
  
  // Turn on/off the calculation of roll/pitch/ztrans based on curent gait
  int calculate_roll = int(params_->gait_type != "amble_gait");
  int calculate_pitch = int(params_->gait_type != "tripod_gait" && params_->gait_type != "amble_gait");
  int calculate_z_trans = int(params_->gait_type == "tripod_gait");

  //For each leg calculate compensation values for roll, pitch & z_translation according to progress of swing
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    if (leg_stepper->getStepState() == SWING)
    {
      //Progress of currrent vertical tip position towards max swing height (0.0 @ initial -> 1.0 @ max swing height -> 0.0 @ final)
      swing_height_progress = abs(leg_stepper->getCurrentTipPosition()[2] - leg_stepper->getDefaultTipPosition()[2]) / (leg_stepper->getSwingHeight());

      int invert_roll = -leg->getMirrorDir(); //Inverting for right side
      int invert_pitch = (leg->getIDNumber() / 2) - 1; //Inverting for front legs, zeroing for middle legs
      roll_values.push_back(swing_height_progress * params_->roll_amplitude * calculate_roll * invert_roll);
      pitch_values.push_back(swing_height_progress * params_->pitch_amplitude * calculate_pitch * invert_pitch);
      z_trans_values.push_back(swing_height_progress * params_->z_translation_amplitude * calculate_z_trans);
    }
    else
    {
      roll_values.push_back(0.0);
      pitch_values.push_back(0.0);
      z_trans_values.push_back(0.0);
    }
  }

  // Calculates how many legs are in phase.
  int legs_in_phase = model_->getLegCount() / (1 + *max_element(params_->offset_multiplier.begin(), params_->offset_multiplier.end()));

  // Only adds pitch/roll/zTrans values from 'lead' legs (this ensures value from 'in phase' legs is only added once).
  auto_pose_ = Pose::identity();  
  for (int i = 0; i < (6 / legs_in_phase); i++)
  {
    auto_pose_.rotation_[1] += roll_values[i];
    auto_pose_.rotation_[2] += pitch_values[i];
    auto_pose_.position_[2] += z_trans_values[i];
  }
  
  return auto_pose_;
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

  Vector3d max_translation = params_->max_translation;
  Vector3d max_rotation = params_->max_rotation;

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

  Vector3d translation_velocity = clamped(translation_velocity_input_, 1.0) * params_->max_translation_velocity;
  Vector3d rotation_velocity = clamped(rotation_velocity_input_, 1.0) * params_->max_rotation_velocity;

  Vector3d new_translation_position = translation_position + translation_velocity * params_->time_delta;
  Quat new_rotation_position = rotation_position * Quat(Vector3d(rotation_velocity * params_->time_delta));

  Vector3d translation_limit = Vector3d(0, 0, 0);
  Vector3d rotation_limit = Vector3d(0, 0, 0);

  // Zero velocity input depending on position limitations
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    // TRANSLATION
    // Assign correct translation limit based on velocity direction and reset command
    translation_limit[i] = sign(translation_velocity[i]) * max_translation[i];
    if (reset_translation[i] && default_translation[i] < max_translation[i] && default_translation[i] > -max_translation[i])
    {
      translation_limit[i] = default_translation[i];
    }

    bool positive_translation_velocity = sign(translation_velocity[i]) > 0;
    bool exceeds_positive_translation_limit = positive_translation_velocity && (new_translation_position[i] > translation_limit[i]);
    bool exceeds_negative_translation_limit = !positive_translation_velocity && (new_translation_position[i] < translation_limit[i]);

    // Zero velocity when translation position reaches limit
    if (exceeds_positive_translation_limit || exceeds_negative_translation_limit)
    {
      translation_velocity[i] = (translation_limit[i] - translation_position[i]) / params_->time_delta;
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
      rotation_velocity[i] = (rotation_limit[i] - rotation_position.toEulerAngles()[i]) / params_->time_delta;
    }
  }

  // Update position according to limitations
  manual_pose_.position_ = translation_position + translation_velocity * params_->time_delta;
  manual_pose_.rotation_ = rotation_position * Quat(Vector3d(rotation_velocity * params_->time_delta)); 
  // BUG: ^Adding pitch and roll simultaneously adds unwanted yaw
  
  return manual_pose_;
}

/***********************************************************************************************************************
 * Returns rotation correction used to attempt to match target rotation (manual rotation) using PID controller
***********************************************************************************************************************/
Quat PoseController::imuCompensation(ImuData imu_data)
{
  Quat target_rotation = manual_pose_.rotation_;
  
  // There are two orientations per quaternion and we want the shorter/smaller difference. 
  double dot = target_rotation.dot(~imu_data.orientation);
  if (dot < 0.0)
  {
    target_rotation = -target_rotation;
  }

  // PID gains
  double gain_d = params_->rotation_gain_d;
  double gain_p = params_->rotation_gain_p;
  double gain_i = params_->rotation_gain_i;

  rotation_position_error = imu_data.orientation.toEulerAngles() - target_rotation.toEulerAngles();
  rotation_absement_error += rotation_position_error * params_->time_delta;  // Integration of angle position error (absement)

  // Low pass filter of IMU angular velocity data
  double smoothingFactor = 0.15;
  rotation_velocity_error = smoothingFactor * imu_data.angular_velocity + (1 - smoothingFactor) * rotation_velocity_error;

  Vector3d rotation_correction = -(gain_d * rotation_velocity_error + gain_p * rotation_position_error + gain_i * rotation_absement_error);
  rotation_correction[2] = target_rotation.toEulerAngles()[2];  // No compensation in yaw rotation

  double stability_threshold = 100;

  if (rotation_correction.norm() > stability_threshold)
  {
    ROS_FATAL("IMU rotation compensation became unstable! Adjust PID parameters.\n");
    ros::shutdown();
  }
  else
  {
    return Quat(rotation_correction);
  }  
}

/***********************************************************************************************************************
 * Updates inclination pose with translation correction to move centre of body according to inclination of terrain
***********************************************************************************************************************/
Vector3d PoseController::inclinationCompensation(WalkController* walker, ImuData imu_data)
{
  Quat compensation_combined = manual_pose_.rotation_ * auto_pose_.rotation_;
  Quat compensation_removed = imu_data.orientation * compensation_combined.inverse();
  Vector3d euler_angles = compensation_removed.toEulerAngles();
  
  double lateral_correction = walker->getBodyHeight() * tan(euler_angles[0]);
  double longitudinal_correction = -walker->getBodyHeight() * tan(euler_angles[1]);
  
  longitudinal_correction = clamped(longitudinal_correction, -params_->max_translation[0], params_->max_translation[0]);
  lateral_correction = clamped(lateral_correction, -params_->max_translation[1], params_->max_translation[1]);

  inclination_compensation_offset_[0] = longitudinal_correction;
  inclination_compensation_offset_[1] = lateral_correction;
  
  return inclination_compensation_offset_;
}

/***********************************************************************************************************************
 * Calculates mean delta_z value of all legs and returns an offset used to sustain body at correct height
***********************************************************************************************************************/
double PoseController::impedanceControllerCompensation(void)
{
  int loaded_legs = 6;
  double average_delta_z = 0.0;
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    average_delta_z += leg.getDeltaZ();
  }
  average_delta_z /= loaded_legs;

  return clamped(abs(average_delta_z), -params_->max_translation[2], params_->max_translation[2]);
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
  for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
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
      for (leg_it_ = model_->getLegContainer().begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->second();
	LegStepper leg_stepper = leg->getLegStepper();
	LegState state = leg->getLegState();
	if (state == WALKING || state == MANUAL_TO_WALKING)
	{
	  zero_moment_offset[0] += leg_stepper.getDefaultTipPosition()[0];
	  zero_moment_offset[1] += leg_stepper.getDefaultTipPosition()[1];
	}
      }
      
      zero_moment_offset /= legs_loaded;
      zero_moment_offset[0] = clamped(zero_moment_offset[0], -params_->max_translation[0], params_->max_translation[0]);
      zero_moment_offset[1] = clamped(zero_moment_offset[1], -params_->max_translation[1], params_->max_translation[1]);
      
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
 * Leg poser contructor
***********************************************************************************************************************/
LegPoser::LegPoser(PoseController* poser, Leg3DOF* leg, 
		   Vector3d packed_joint_positions, Vector3d unpacked_joint_positions) 
  : poser_(poser)
  , leg_(leg)
  , packed_joint_positions_(packed_joint_positions)
  , unpacked_joint_positions_(unpacked_joint_positions)
{
  
}

/***********************************************************************************************************************
 * Move joints of leg to target positions
***********************************************************************************************************************/
bool LegPoser::moveToJointPosition(Vector3d target_joint_positions, double time_to_move)
{
  // Setup origin and target joint positions for bezier curve
  if (first_iteration_)
  {
    first_iteration_ = false;
    master_iteration_count_ = 0;
    double coxa_joint_angle = leg_->getCoxaJointPosition();
    double femur_joint_angle = leg_->getFemurJointPosition();
    double tibia_joint_angle = leg_->getTibiaJointPosition();
    origin_joint_positions_ = Vector3d(coxa_joint_angle, femur_joint_angle, tibia_joint_angle);
  }

  int num_iterations = max(1, int(roundToInt(time_to_step / params_->time_delta));
  double delta_t = 1.0 / num_iterations;
 
  master_iteration_count_++;  

  Vector3d new_joint_positions;
  Vector3d control_nodes[4];

  // Control nodes for linear cubic bezier curve
  control_nodes[0] = origin_joint_positions_;
  control_nodes[1] = origin_joint_positions_;
  control_nodes[2] = target_joint_positions;
  control_nodes[3] = target_joint_positions;

  // Calculate change in position using bezier curve
  new_joint_positions = cubicBezier(control_nodes, master_iteration_count_ * delta_t);

  if (leg_->getIDNumber() == 0) //reference leg for debugging
  {
    double time = master_iteration_count_ * delta_t;
    bool debug = poser_->getParameters().debug_moveToJointPosition;
    ROS_DEBUG_COND(debug, "MOVE_TO_JOINT_POSITION DEBUG - MASTER ITERATION: %d\t\tTIME: %f\t\tORIGIN: %f:%f:%f\t\tCURRENT: %f:%f:%f\t\tTARGET: %f:%f:%f\n", master_iteration_count_, time, origin_joint_positions_[0], origin_joint_positions_[1], origin_joint_positions_[2], pos[0], pos[1], pos[2], target_joint_positions[0], target_joint_positions[1], target_joint_positions[2]);
  }

  //TBD
  leg_->setCoxaJointPosition(new_joint_positions[0]);
  leg_->setFemurJointPosition(new_joint_positions[1]);
  leg_->setTibiaJointPosition(new_joint_positions[2]);
  leg_->applyFK();
  
  //Return ratio of completion (1.0 when fully complete)
  double completion_ratio = (double(master_iteration_count_ - 1) / double(num_iterations));
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return 1.0;
  }
  else
  {
    return completion_ratio;  
  }
}

/***********************************************************************************************************************
 * Step leg tip position to target tip position
***********************************************************************************************************************/
double LegPoser::stepToPosition(Vector3d target_tip_position, Pose target_pose, double lift_height, double time_to_step)
{
  if (first_iteration_)
  {
    first_iteration_ = false;
    master_iteration_count_ = 0;
    origin_tip_position_ = leg_->getLocalTipPosition();
    //origin_tip_position_[2] += delta_z;  // Remove deltaZ offset temporarily //TBD
  }

  master_iteration_count_++;

  int num_iterations = max(1, int(roundToInt(time_to_step / params_->time_delta));
  double delta_t = time_to_step / num_iterations;
  
  double completion_ratio = (double(master_iteration_count_ - 1) / double(num_iterations));
  
  // Applies required posing slowly over course of transition
  // Scales position vector by 0->1.0
  target_pose.position_ *= completion_ratio;  
  // Scales rotation quat by 0.0->1.0 (https://en.wikipedia.org/wiki/Slerp)
  target_pose.rotation_ = Pose::identity().rotation_.slerpTo(target_pose.rotation_, completion_ratio);  

  int half_swing_iteration = num_iterations / 2;

  // Update leg tip position
  Vector3d control_nodes_primary[4];
  Vector3d controlNodesSecondary[4];

  // Control nodes for dual 3d cubic bezier curves
  control_nodes_primary[0] = origin_tip_position_;
  control_nodes_primary[1] = control_nodes_primary[0];
  control_nodes_primary[3] = 0.5 * (target_tip_position + origin_tip_position_);
  control_nodes_primary[3][2] += lift_height;
  control_nodes_primary[2] = control_nodes_primary[0];
  control_nodes_primary[2][2] += lift_height;

  controlNodesSecondary[0] = control_nodes_primary[3];
  controlNodesSecondary[1] = 2 * controlNodesSecondary[0] - control_nodes_primary[2];
  controlNodesSecondary[3] = target_tip_position;
  controlNodesSecondary[2] = controlNodesSecondary[3];

  Vector3d pos;
  int swing_iteration_count = (master_iteration_count_ + (num_iterations - 1)) % (num_iterations) + 1;
  
  // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
  if (swing_iteration_count <= half_swing_iteration)
  {
    pos = cubicBezier(control_nodes_primary, swing_iteration_count * delta_t * 2.0);
  }
  else
  {
    pos = cubicBezier(controlNodesSecondary, (swing_iteration_count - half_swing_iteration) * delta_t * 2.0);
  }

  if (leg_->getIDNumber() == 0) //Reference leg for debugging (AL)
  {
    ROS_DEBUG_COND(poser_->getParameters().debug_stepToPosition, "STEP_TO_POSITION DEBUG - LEG: %s\t\tMASTER ITERATION: %d\t\tORIGIN: %f:%f:%f\t\tCURRENT: %f:%f:%f\t\tTARGET: %f:%f:%f\n", leg_->getIDName(), master_iteration_count_, origin_tip_position_[0], origin_tip_position_[1], origin_tip_position_[2], pos[0], pos[1], pos[2], target_tip_position[0], target_tip_position[1], target_tip_position[2]);
  }

  // Apply inverse kinematics to localTipPositions and stanceTipPositions //TBD How to apply change
  if (leg_->getLegState() != MANUAL)
  {
    current_tip_position_ = target_pose.inverseTransformVector(pos);
    //adjusted_pos[2] -= delta_z;  // Re-apply deltaZ offset
    //leg_->applyLocalIK(adjusted_pos);
  }

  //Return ratio of completion (1.0 when fully complete)
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return 1.0;
  }
  else
  {
    return completion_ratio;
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
