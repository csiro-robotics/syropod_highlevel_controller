
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
  
  inclination_compensation_offset_ = Vector3d(0,0,0);
  
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    leg->setLegPoser(new LegPoser(this, leg));
  }  
}

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
void PoseController::updateStance(void)
{
	bool exclude_swinging_legs = params_->auto_compensation.data && !params_->imu_compensation.data; 

	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		Leg* leg = leg_it_->second;
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
				// Remove auto pose compensation contribution from swinging leg, on swinging leg
				Pose modified_auto_pose = auto_pose_;
				modified_auto_pose.position_ -= leg_poser->getSwingAutoPose().position_;
				modified_auto_pose.rotation_ *= leg_poser->getSwingAutoPose().rotation_.inverse();
				
				compensation_pose.position_ -= auto_pose_.position_;
				compensation_pose.rotation_ *= auto_pose_.rotation_.inverse();
				compensation_pose.position_ += modified_auto_pose.position_;
				compensation_pose.rotation_ *= modified_auto_pose.rotation_;
			}
			
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
			for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
			{
				Leg* leg = leg_it_->second;
				LegPoser* leg_poser = leg->getLegPoser();
				LegStepper* leg_stepper = leg->getLegStepper();
				Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
				if (!raise_complete_)
				{					
					//Set step target as close to default whilst allowing full body raising (i.e. > min leg length)
					Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
					default_tip_position[2] = 0; //Ignore vertical
					double horizontal_target_length = max(leg->getMinLegLength(), default_tip_position.norm());
					double tip_target_bearing = atan2(default_tip_position[1], default_tip_position[0]);
					double new_x_position = horizontal_target_length*cos(tip_target_bearing);
					double new_y_position = horizontal_target_length*sin(tip_target_bearing);
					double new_z_position = !raise_complete_ ? 0.0 : leg->getLocalTipPosition()[2];
					leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, new_z_position));
				}
				else
				{
					//Set step target as walking default position
					Vector3d target_tip_position = default_tip_position;
					target_tip_position[2] = leg->getLocalTipPosition()[2];
					leg_poser->setTargetTipPosition(target_tip_position);
				}
			}
			set_target_ = false;
		}
		
		// Step to target
		for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
		{
			Leg* leg = leg_it_->second;
			LegPoser* leg_poser = leg->getLegPoser();
			LegStepper* leg_stepper = leg->getLegStepper();
			if (leg->getGroup() == current_group_)
			{
				double step_height = !raise_complete_ ? 0.0 : leg_stepper->getSwingHeight();
				double time_to_step = 1.0 / params_->step_frequency.current_value;
				Vector3d target_tip_position = leg_poser->getTargetTipPosition();
				progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, time_to_step);
				leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
				//leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
				leg->applyIK(true, params_->debug_IK.data);
				legs_completed_step_ += int(progress == 100);
			}
		}
		
		// Check if next target new stepping/raising target is to be set
		set_target_ = (progress == 100);
		
		// Normalise progress in terms of total start up procedure
		progress = progress/5 + current_group_*20;
		progress += (!raise_complete_ ? 0 : 60);
		
		// Coordinate leg stepping groups
		int num_legs = model_->getLegCount();
		if (legs_completed_step_ == num_legs/2)
		{
			current_group_ = 1;
		}
		else if (legs_completed_step_ == num_legs)
		{
			step_complete_ = true;
			legs_completed_step_ = 0;
			current_group_ = 0;
		}
	}
	// Raise body to default position
	else
	{
		// Set raising target
		if (set_target_)
		{
			for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
			{
				Leg* leg = leg_it_->second;
				LegPoser* leg_poser = leg->getLegPoser();
				LegStepper* leg_stepper = leg->getLegStepper();				
				Vector3d current_tip_position = leg->getLocalTipPosition();
				double raise_height = leg_stepper->getDefaultTipPosition()[2];
				leg_poser->setTargetTipPosition(Vector3d(current_tip_position[0], current_tip_position[1], raise_height));
			}
			set_target_ = false;
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
				leg->applyIK(true, params_->debug_IK.data);
			}
		}
		
		// Check if raise is complete and order another stepping task
		raise_complete_ = (progress == 100);
		step_complete_ = !raise_complete_;
		
		// Check if next target new stepping/raising target is to be set
		set_target_ = (progress == 100);
		
		// Normalise progress in terms of total start up procedure
		progress = progress/5 + 40;
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
			//Find the smallest body height all legs can achieve
			double target_body_height = 0; // On ground
			for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
			{
				Leg* leg = leg_it_->second;

				//Calculate the minimum body height that this leg can lower the body to, along a vertical trajectory
				double min_body_height = 0; // Assume leg can lower body to ground
				Vector3d current_tip_position = leg->getLocalTipPosition();
				double horizontal_distance_to_tip = sqrt(sqr(current_tip_position[0]) + sqr(current_tip_position[1]));
				double min_leg_length = leg->getMinLegLength();
				if (min_leg_length > horizontal_distance_to_tip)
				{
					min_body_height = sqrt(sqr(min_leg_length) - sqr(horizontal_distance_to_tip));
				}
				target_body_height = max(target_body_height, min_body_height);
			}
			// Set leg target position for calculated body height
			for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
			{
				Leg* leg = leg_it_->second;
				LegPoser* leg_poser = leg->getLegPoser();
				Vector3d current_tip_position = leg->getLocalTipPosition();
				Vector3d target_tip_position(current_tip_position[0], current_tip_position[1], -target_body_height);
				leg_poser->setTargetTipPosition(target_tip_position);
			}
		}
		set_target_ = false;
		
		// Lower body as much as possible from current position
		for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
		{
			Leg* leg = leg_it_->second;
			LegPoser* leg_poser = leg->getLegPoser();
			double time_to_step = 1.0 / params_->step_frequency.current_value;
			Vector3d target_tip_position = leg_poser->getTargetTipPosition();
			progress = leg_poser->stepToPosition(target_tip_position, Pose::identity(), 0.0, time_to_step);
			leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
			//leg->applyDeltaZ(leg_poser->getCurrentTipPosition());
			leg->applyIK(true, params_->debug_IK.data);
			lower_complete_ = (progress == 100);
		}
		
		// Check if next target new stepping/raising target is to be set
		set_target_ = (progress == 100);
		
		// Normalise progress in terms of total start up procedure
		progress = progress/4;
		progress += (!step_complete_ ? 0 : 75);
	}
	// Step to optimal lower position
	else
	{
		if (set_target_)
		{
			for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
			{
				Leg* leg = leg_it_->second;
				LegPoser* leg_poser = leg->getLegPoser();
				LegStepper* leg_stepper = leg->getLegStepper();
				Vector3d default_tip_position = leg_stepper->getDefaultTipPosition();
				//Set step target as min leg length distance along walking default tip position bearing
				double tip_target_bearing = atan2(default_tip_position[1], default_tip_position[0]);
				double new_x_position = leg->getMinLegLength()*cos(tip_target_bearing);
				double new_y_position = leg->getMinLegLength()*sin(tip_target_bearing);
				leg_poser->setTargetTipPosition(Vector3d(new_x_position, new_y_position, leg->getLocalTipPosition()[2]));
			}
			set_target_ = false;
		}
		
		// Step to optmial lowering position
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
				leg->applyIK(true, params_->debug_IK.data);
				legs_completed_step_ += int(progress == 100);
			}
		}
		
		// Check if next target new stepping/raising target is to be set
		set_target_ = (progress == 100);
		
		// Normalise progress in terms of total start up procedure
		progress = progress/4 + current_group_*25;
		
		// Coordinate leg stepping groups
		int num_legs = model_->getLegCount();
		if (legs_completed_step_ == num_legs/2)
		{
			current_group_ = 1;
		}   
		else if (legs_completed_step_ == num_legs)
		{
			step_complete_ = true;
			lower_complete_ = false;
			legs_completed_step_ = 0;
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
    leg->applyIK(true, params_->debug_IK.data);
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
			leg->applyIK(true, params_->debug_IK.data);
			legs_completed_step_ += int(progress == 100);
		}
	}
	
	// Normalise progress in terms of total procedure
	progress = progress/2 + current_group_*50;

	current_group_ = legs_completed_step_ / (num_legs/2);

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
		leg->applyIK(true, params_->debug_IK.data);
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
void PoseController::updateCurrentPose(double body_height)
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
	double swing_height_progress = 1.0;
	vector<double> roll_values;
	vector<double> pitch_values;
	vector<double> z_trans_values;

	// Turn on/off the calculation of roll/pitch/ztrans based on curent gait
	int calculate_roll = int(params_->gait_type.data != "amble_gait");
	int calculate_pitch = int(params_->gait_type.data != "tripod_gait" && params_->gait_type.data != "amble_gait");
	int calculate_z_trans = int(params_->gait_type.data == "tripod_gait");

	//Calculate reference body length/width
	double reference_base_link_length = model_->getLegByIDNumber(0)->getLinkByIDNumber(0)->length;
	double reference_base_link_angle = model_->getLegByIDNumber(0)->getLinkByIDNumber(0)->angle;
	double body_length = reference_base_link_length*cos(reference_base_link_angle);
	double body_width = reference_base_link_length*sin(reference_base_link_angle);

	//For each leg calculate compensation values for roll, pitch & z_translation according to progress of swing
	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		Leg* leg = leg_it_->second;
		LegStepper* leg_stepper = leg->getLegStepper();
		LegPoser* leg_poser = leg->getLegPoser();
		if (leg_stepper->getStepState() == SWING)
		{
			//Progress of currrent vertical tip position towards max swing height (0.0 @ initial -> 1.0 @ max swing height -> 0.0 @ final)
			swing_height_progress = abs(leg_stepper->getCurrentTipPosition()[2] - leg_stepper->getDefaultTipPosition()[2]) / (leg_stepper->getSwingHeight());

			Link* base_link = leg->getLinkByIDNumber(0);
			double roll_scaler = -base_link->length*sin(base_link->angle)/body_width;
			double pitch_scaler = -base_link->length*cos(base_link->angle)/body_length;
			double roll_amplitude = params_->auto_compensation_parameters.data["roll"] * roll_scaler;
			double pitch_amplitude = params_->auto_compensation_parameters.data["pitch"] * pitch_scaler;
			double z_translation_amplitude = params_->auto_compensation_parameters.data["z_trans"];
			
			double roll_value = swing_height_progress * roll_amplitude * calculate_roll;
			double pitch_value = swing_height_progress * pitch_amplitude * calculate_pitch;
			double z_trans_value = swing_height_progress * z_translation_amplitude * calculate_z_trans;
			roll_values.push_back(roll_value);
			pitch_values.push_back(pitch_value);
			z_trans_values.push_back(z_trans_value);
			
			// Store swinging leg contribution of auto_pose
			Pose swing_auto_pose(Vector3d(0,0,z_trans_value), Quat(1.0, roll_value, pitch_value, 0.0));
			leg_poser->setSwingAutoPose(swing_auto_pose);
		}
		else
		{
			roll_values.push_back(0.0);
			pitch_values.push_back(0.0);
			z_trans_values.push_back(0.0);
		}
	}

	// Calculates how many legs are in phase.
	std::vector<int> offset_multiplier = params_->offset_multiplier.data;
	int legs_in_phase = model_->getLegCount() / (1 + *max_element(offset_multiplier.begin(), offset_multiplier.end()));

	// Only adds pitch/roll/zTrans values from 'lead' legs (this ensures value from 'in phase' legs are only added once).
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
  rotation_velocity_error_ = smoothingFactor*imu_data_.angular_velocity + (1 - smoothingFactor)*rotation_velocity_error_;

  Vector3d rotation_correction = -(kd * rotation_velocity_error_ + kp*rotation_position_error_ + ki*rotation_absement_error_);
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
 * Leg poser contructor
***********************************************************************************************************************/
LegPoser::LegPoser(PoseController* poser, Leg* leg) 
  : poser_(poser)
  , leg_(leg)
  , current_tip_position_(Vector3d(0,0,0))
{
	swing_auto_pose_.position_ = Vector3d(0,0,0);
	swing_auto_pose_.rotation_ = Quat(1,0,0,0);
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
  int i=0;
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
  int progress = int((double(master_iteration_count_ - 1) / double(num_iterations))*100);
  if (master_iteration_count_ >= num_iterations)
  {
    first_iteration_ = true;
    return 100;
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
    first_iteration_ = false;
    master_iteration_count_ = 0;
    origin_tip_position_ = leg_->getLocalTipPosition();
    current_tip_position_ = origin_tip_position_;
    //origin_tip_position_[2] += delta_z;  // Remove deltaZ offset temporarily //TBD
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

  if (leg_->getIDNumber() == 0) //Reference leg for debugging (AL)
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
    return 100;
  }
  else
  {
    return int(completion_ratio*100);
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
