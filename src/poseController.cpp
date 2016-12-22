
#include "../include/simple_hexapod_controller/poseController.h"

/***********************************************************************************************************************
 * Pose controller contructor
***********************************************************************************************************************/
PoseController::PoseController(Model *model, Parameters p)
  : params_(p)
  , targetPose(Pose::identity())
  , originPose(Pose::identity())
  , manualPose(Pose::identity())
  , imuPose(Pose::identity())
  , inclinationPose(Pose::identity())
  , deltaZPose(Pose::identity())
  , defaultPose(Pose::identity())
  , autoPoseDefault(Pose::identity())
{
  rotationAbsementError = Vector3d(0, 0, 0);
  rotationPositionError = Vector3d(0, 0, 0);
  rotationVelocityError = Vector3d(0, 0, 0);

  translationAbsementError = Vector3d(0, 0, 0);
  translationPositionError = Vector3d(0, 0, 0);
  translationVelocityError = Vector3d(0, 0, 0);
  translationAccelerationError = Vector3d(0, 0, 0);
  
  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    int index = leg->getIDNumber();
    leg->setLegPoser(this, p.packed_joint_positions[index], p.unpacked_joint_positions[index]);
  }  
}

/***********************************************************************************************************************
 * Updates default stance tip positions according to desired pose
 * This is then later used in walk controller where inverse kinematics are applied
***********************************************************************************************************************/
void PoseController::updateStance(Vector3d targetTipPositions[3][2], bool excludeSwingingLegs)
{
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      LegState state = model->legs_[l][s].leg_state_;
      if (state == WALKING || state == MANUAL_TO_WALKING)
      {
        if (!excludeSwingingLegs || walker->legSteppers[l][s].step_state_ != SWING)
        {
          tipPositions[l][s] = current_pose_.inverseTransformVector(targetTipPositions[l][s]);
        }
        else
        {
          // Remove default autoCompensation and apply leg specific auto compensation during swing
          Pose swingAutoComp = current_pose_;
          swingAutoComp.position_ = (swingAutoComp.position_ - autoPoseDefault.position_) + autoPose[l][s].position_;
          swingAutoComp.rotation_ =
              (swingAutoComp.rotation_ * autoPoseDefault.rotation_.inverse()) * autoPose[l][s].rotation_;
          tipPositions[l][s] = swingAutoComp.inverseTransformVector(targetTipPositions[l][s]);
        }
      }
      else if (state == MANUAL || WALKING_TO_MANUAL)  // Apply zero posing while in MANUAL state
      {
        tipPositions[l][s] = targetTipPositions[l][s];
      }
    }
  }
}

/***********************************************************************************************************************
 * Moves tip positions along direct straight path from origin position to target walking stance position
***********************************************************************************************************************/ 
double PoseController::directStartup(Model* model)
{
  double res;
  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();      
    LegPoser* leg_poser = leg->getLegPoser();
    LegStepper* leg_stepper = leg->getLegStepper();
    Vector3d target_tip_position = leg_stepper->getDefaultTipPosition();
    res = leg_poser->stepToPosition(target_tip_position, model->getCurrentPose(), 0.0,  params_.time_to_start);
  }
  return res;
}

/***********************************************************************************************************************
 * Steps tip positions into correct pose for leg manipulation
***********************************************************************************************************************/
double PoseController::poseForLegManipulation(Model* model, Leg* leg)
{
  Pose targetPose;
  double res = 0;
  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    double step_height = leg_stepper->getSwingHeight();
    if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      targetPose = Pose::identity();
      targetPose.position_ += inclinationPose.position_;  // Apply inclination control to lifted leg
      targetPose.position_[2] -= step_height; 
    }
    // Get target tip positions for legs in WALKING state using default pose
    else
    {
      targetPose = current_pose_;
      targetPose.position_ -= manualPose.position_;
      targetPose.position_ += defaultPose.position_;      
    }
    Vector3d target_tip_position = targetPose.inverseTransformVector(leg_stepper->getDefaultTipPosition());
    res = leg_poser->stepToPosition(target_tip_position, Pose::identity(), step_height, 1.0 / params_.step_frequency);
  }
  return res;
}

/***********************************************************************************************************************
 * Step legs by moving tip position (according to LegCoordinationMode) to the target tip positions
***********************************************************************************************************************/
bool PoseController::stepLegsToPosition(map<Leg3DOF*, Vector3d> leg_target_map, Pose target_pose, 
					LegCoordinationMode mode, double lift_height, double time_to_step)
{
  int num_legs = leg_target_map.size();
  switch (mode)
  {
    case (SIMULTANEOUS_MODE):
    {
      for (leg_it_ = leg_target_map.begin(); leg_it_ != leg_target_map.end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->first();
	LegPoser* leg_poser = leg->getLegPoser();	
	Vector3d target_tip_position = leg_it_->second();
	double result = leg_poser->stepToPosition(target_tip_position, target_pose, lift_height, time_to_step);
	if (result == 1.0)
	{
	  legs_completed_task_ ++;
	}
      }
      break;
    }
    case (SEQUENTIAL_MODE):
    {
      if (legs_completed_task_ == 0)
      {
	leg_it_ = leg_target_map.begin();
      }
      Leg3DOF* leg = leg_it_->first();
      LegPoser* leg_poser = leg->getLegPoser();
      Vector3d target_tip_position = leg_it_->second();
      double result = leg_poser->stepToPosition(target_tip_position, target_pose, lift_height, time_to_step/num_legs);
      if (result == 1.0)
      {
	leg_it_++;
	legs_completed_task_++;
      }
      break;
    }
    case (TRIPOD_MODE):
    {
      for (leg_it_ = leg_target_map.begin(); leg_it_ != leg_target_map.end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->first();
	if (leg->getTripodGroup() == current_group_)
	{
	  LegPoser* leg_poser = leg->getLegPoser();
	  Vector3d target_tip_position = leg_it_->second();
	  double result = leg_poser->stepToPosition(target_tip_position, target_pose, lift_height, time_to_step/2.0);
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
      break;
    }     
  }  
  if (legs_completed_task_ == num_legs)
  {
    legs_completed_task_ = 0;
    return true;
  }
  else 
  {
    return false;
  }
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
bool PoseController::packLegs(Model* model, double time_to_pack)
{
  map<Leg3DOF*, Vector3d> leg_target_map;
  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegPoser* leg_poser = leg->getLegPoser();
    leg_target_map.insert(leg, leg_poser->getPackedJointPositions());
  }  
  return moveLegsToJointPositions(leg_target_map, SIMULTANEOUS_MODE, time_to_pack);
}

/***********************************************************************************************************************
 * Unpack legs by directly moving leg joint positions simultaneously to the packed joint positions defined by parameters
***********************************************************************************************************************/
bool PoseController::unpackLegs(Model* model, double time_to_unpack)
{
  map<Leg3DOF*, Vector3d> leg_target_map;
  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegPoser* leg_poser = leg->getLegPoser();
    leg_target_map.insert(leg, leg_poser->getUnpackedJointPositions());
  }  
  return moveLegsToJointPositions(leg_target_map, SIMULTANEOUS_MODE, time_to_unpack);
}

/***********************************************************************************************************************
 * Move legs by directly moving leg joint positions (according to LegCoordinationMode) to the target joint positions
***********************************************************************************************************************/
bool PoseController::moveLegsToJointPositions(map<Leg3DOF*, Vector3d> leg_target_map, 
					      LegCoordinationMode mode, double time_to_move)
{
  int num_legs = leg_target_map.size();
  switch (mode)
  {
    case (SIMULTANEOUS_MODE):
    {
      for (leg_it_ = leg_target_map.begin(); leg_it_ != leg_target_map.end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->first();
	LegPoser* leg_poser = leg->getLegPoser();	
	Vector3d target_joint_position = leg_it_->second();
	double result = leg_poser->moveToJointPosition(target_joint_position, time_to_move);
	if (result == 1.0)
	{
	  legs_completed_task_ ++;
	}
      }
      break;
    }
    case (SEQUENTIAL_MODE):
    {
      if (legs_completed_task_ == 0)
      {
	leg_it_ = leg_target_map.begin();
      }
      Leg3DOF* leg = leg_it_->first();
      LegPoser* leg_poser = leg->getLegPoser();
      Vector3d target_joint_position = leg_it_->second();
      double result = leg_poser->moveToJointPosition(target_joint_position, time_to_move/num_legs);
      if (result == 1.0)
      {
	leg_it_++;
	legs_completed_task_++;
      }
      break;
    }
    case (TRIPOD_MODE):
    {
      for (leg_it_ = leg_target_map.begin(); leg_it_ != leg_target_map.end(); ++leg_it_)
      {
	Leg3DOF* leg = leg_it_->first();
	if (leg->getTripodGroup() == current_group_)
	{
	  LegPoser* leg_poser = leg->getLegPoser();
	  Vector3d target_joint_position = leg_it_->second();
	  double result = leg_poser->moveToJointPosition(target_joint_position, time_to_move/2.0);
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
      break;
    }     
  }  
  if (legs_completed_task_ == num_legs)
  {
    legs_completed_task_ = 0;
    return true;
  }
  else 
  {
    return false;
  }
}

/***********************************************************************************************************************
 * Startup sequence
***********************************************************************************************************************/
bool PoseController::startUpSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
                                     bool forceSequentialMode)
{
  if (sequenceStep == 0)
  {
    startHeightRatio = createSequence(targetTipPositions);
    if (startHeightRatio > 0.8)
    {
      sequenceStep = 3;
    }
    else
    {
      sequenceStep = 1;
    }
  }

  StepToPositionModes mode;
  if (forceSequentialMode)
  {
    mode = SEQUENTIAL_MODE;
  }
  else if (sequenceStep == 1)
  {
    mode = startHeightRatio < 0.1 ? NO_STEP_MODE : SEQUENTIAL_MODE;
  }
  else if (sequenceStep == 3)
  {
    mode = startHeightRatio > 0.8 ? TRIPOD_MODE : SEQUENTIAL_MODE;
  }

  double res = 0.0;
  double stepHeight = walker->maximum_body_height_ * walker->step_clearance_;
  double zeroDeltaZ[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  switch (sequenceStep)
  {
    case 1:
      res = stepLegsToPosition(phase1TipPositions, Pose::identity(), zeroDeltaZ, mode, stepHeight);
      break;
    case 2:
      res = stepLegsToPosition(phase2TipPositions, Pose::identity(), zeroDeltaZ, NO_STEP_MODE, 0.0);
      break;
    case 3:
      res = stepLegsToPosition(phase3TipPositions, Pose::identity(), zeroDeltaZ, TRIPOD_MODE, stepHeight);
      break;
    case 4:
      res = stepLegsToPosition(phase4TipPositions, targetPose, deltaZ, NO_STEP_MODE, 0.0);
      break;
    case 5:
      sequenceStep = 0;
      return true;
    default:
      return false;
  }

  if (res == 1.0)
  {
    sequenceStep++;
  }

  return false;
}

/***********************************************************************************************************************
 * Shutdown sequence
***********************************************************************************************************************/
bool PoseController::shutDownSequence(Vector3d targetTipPositions[3][2], Pose targetPose, double deltaZ[3][2],
                                      bool forceSequentialMode)
{
  double res = 0.0;
  double stepHeight = walker->maximum_body_height_ * walker->step_clearance_;
  double zeroDeltaZ[3][2] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
  switch (sequenceStep)
  {
    case 0:
      createSequence(targetTipPositions);
      res = true;
      break;
    case 1:
      res = stepLegsToPosition(phase5TipPositions, Pose::identity(), zeroDeltaZ, NO_STEP_MODE, 0.0);
      break;
    case 2:
      res = stepLegsToPosition(phase6TipPositions, Pose::identity(), zeroDeltaZ, TRIPOD_MODE, stepHeight);
      break;
    case 3:
      res = stepLegsToPosition(phase7TipPositions, Pose::identity(), zeroDeltaZ, NO_STEP_MODE, 0.0);
      break;
    case 4:
      sequenceStep = 0;
      return true;
    default:
      return false;
  }

  if (res == 1.0)
  {
    sequenceStep++;
  }

  return false;
}

/***********************************************************************************************************************
 * Calculates tip positions for startup/shutdown sequences
***********************************************************************************************************************/
// TBD REFACTORING
double PoseController::createSequence(Vector3d targetTipPositions[3][2])
{
  // Get average z position of leg tips
  double averageTipZ = 0.0;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      averageTipZ += model->legs_[l][s].local_tip_position_[2];
    }
  }
  averageTipZ /= 6.0;

  // Ratio of average tip z position to required default tip z position
  double startHeightRatio = averageTipZ / targetTipPositions[0][0][2];
  if (startHeightRatio < 0.0)
  {
    startHeightRatio = 0.0;
  }

  double liftAngle = pi / 3;
  double desKneeAngle =
      liftAngle + asin(model->legs_[0][0].femur_link_length_ * sin(liftAngle) / model->legs_[0][0].tibia_link_length_);
  double offset = atan2(model->legs_[0][0].tip_offset_[2], -model->legs_[0][0].tip_offset_[1]);
  desKneeAngle += offset;
  Vector3d minStartTipPositions = model->legs_[0][0].calculateFK(0.77, liftAngle, desKneeAngle);
  double startStanceRatio = minStartTipPositions.squaredNorm() / targetTipPositions[0][0].squaredNorm();

  double heightScaler = 0.75;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      double positionScaler = (-(startStanceRatio - 1.0) * startHeightRatio + startStanceRatio);
      phase1TipPositions[l][s] = targetTipPositions[l][s] * positionScaler;
      phase1TipPositions[l][s][2] = -0.025;  // parameterise

      phase2TipPositions[l][s] = phase1TipPositions[l][s];
      phase2TipPositions[l][s][2] =
          (heightScaler + startHeightRatio * (1 - heightScaler)) * targetTipPositions[0][0][2];

      phase3TipPositions[l][s] = targetTipPositions[l][s];
      phase3TipPositions[l][s][2] = phase2TipPositions[l][s][2];

      phase4TipPositions[l][s] = targetTipPositions[l][s];

      phase5TipPositions[l][s] = model->legs_[l][s].local_tip_position_;
      phase5TipPositions[l][s][2] *= heightScaler;

      phase6TipPositions[l][s] = targetTipPositions[l][s] * 1.3;
      phase6TipPositions[l][s][2] = phase5TipPositions[l][s][2];

      phase7TipPositions[l][s] = phase6TipPositions[l][s];
      phase7TipPositions[l][s][2] = -0.025;  // parameterise
    }
  }
  return startHeightRatio;
}

/***********************************************************************************************************************
 * Reset sequence step
***********************************************************************************************************************/
void PoseController::resetSequence(void)
{
  sequenceStep = 1;
  firstIteration = true;
}

/***********************************************************************************************************************
 * Calculates pitch/roll for smooth auto body compensation from offset pose
***********************************************************************************************************************/
void PoseController::autoCompensation(Model* model)
{
  double swing_height_percentage = 1.0;
  vector<double> roll_values;
  vector<double> pitch_values;
  vector<double> z_trans_values;
  
  // Turn on/off the calculation of roll/pitch/ztrans based on curent gait
  int calculate_roll = int(params_.gait_type != "amble_gait");
  int calculate_pitch = int(params_.gait_type != "tripod_gait" && params_.gait_type != "amble_gait");
  int calculate_z_trans = int(params_.gait_type == "tripod_gait");

  for (leg_it_ = model->getLegContainer().begin(); leg_it_ != model->getLegContainer()->end(); ++leg_it_)
  {
    Leg3DOF* leg = leg_it_->second();
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    if (leg_stepper->getStepState() == SWING)
    {
      swing_height_percentage = abs(leg_stepper->getCurrentTipPosition()[2] - leg_stepper->getDefaultTipPosition()[2]) / (leg_stepper->getSwingHeight());

      int invert_roll = -leg->getMirrorDir(); //Inverting for right side
      int invert_pitch = (leg->getIDNumber() / 2) - 1; //Inverting for front legs, zeroing for middle legs
      roll_values.push_back(swing_height_percentage * params_.roll_amplitude * calculate_roll * invert_roll);
      pitch_values.push_back(swing_height_percentage * params_.pitch_amplitude * calculate_pitch * invert_pitch);
      z_trans_values.push_back(swing_height_percentage * params_.z_translation_amplitude * calculate_z_trans);
    }
    else
    {
      roll_values.push_back(0.0);
      pitch_values.push_back(0.0);
      z_trans_values.push_back(0.0);
    }
  }

  // Calculates how many legs are perfectly in phase.
  int legs_in_phase = model->getLegCount() / (1 + *max_element(params_.offset_multiplier.begin(), params_.offset_multiplier.end()));

  // Only adds pitch/roll/zTrans values from 'lead' legs (this ensures value from 'in phase' legs is only added once).
  autoPoseDefault = Pose::identity();
  
  for (int i = 0; i < (6 / legs_in_phase); i++)
  {
    int leadLegRef = i / 2;
    int leadSideRef = i % 2;

    autoPoseDefault.rotation_[1] += roll_values[leadLegRef][leadSideRef];
    autoPoseDefault.rotation_[2] += pitch_values[leadLegRef][leadSideRef];
    autoPoseDefault.position_[2] += z_trans_values[leadLegRef][leadSideRef];
  }

  // Reduce pose on each leg to zero during swing by subtracting compensation added by it's lead leg.
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      int index = 2 * l + s;
      int inPhaseLeadIndex =
          distance(walker->params_.offset_multiplier.begin(),
                   find(walker->params_.offset_multiplier.begin(), walker->params_.offset_multiplier.end(),
                        walker->params_.offset_multiplier[index]));
      int inPhaseLeadLegRef = inPhaseLeadIndex / 2;
      int inPhaseLeadSideRef = inPhaseLeadIndex % 2;

      autoPose[l][s] = autoPoseDefault;
      autoPose[l][s].rotation_[1] -= roll_values[inPhaseLeadLegRef][inPhaseLeadSideRef];
      autoPose[l][s].rotation_[2] -= pitch_values[inPhaseLeadLegRef][inPhaseLeadSideRef];
      autoPose[l][s].position_[2] -= z_trans_values[inPhaseLeadLegRef][inPhaseLeadSideRef];
    }
  }
}

/***********************************************************************************************************************
 * Calculates pitch/roll/yaw/x,y,z for smooth transition to target pose for manual body compensation
***********************************************************************************************************************/
void PoseController::manualCompensation(Vector3d translationVelocityInput, Vector3d rotationVelocityInput,
                                        PoseResetMode poseResetMode, Pose defaultPose)
{
  Vector3d translationPosition = manualPose.position_;
  Quat rotationPosition = manualPose.rotation_;

  Vector3d defaultTranslation = defaultPose.position_;
  Vector3d defaultRotation = defaultPose.rotation_.toEulerAngles();

  Vector3d maxTranslation = parameters.max_translation;
  Vector3d maxRotation = parameters.max_rotation;

  bool resetTranslation[3] = { false, false, false };
  bool resetRotation[3] = { false, false, false };

  switch (poseResetMode)
  {
    case (Z_AND_YAW_RESET):
      resetTranslation[2] = true;
      resetRotation[2] = true;
      break;
    case (X_AND_Y_RESET):
      resetTranslation[0] = true;
      resetTranslation[1] = true;
      break;
    case (PITCH_AND_ROLL_RESET):
      resetRotation[0] = true;
      resetRotation[1] = true;
      break;
    case (ALL_RESET):
      resetTranslation[0] = true;
      resetTranslation[1] = true;
      resetTranslation[2] = true;
      resetRotation[0] = true;
      resetRotation[1] = true;
      resetRotation[2] = true;
      break;
    case (IMMEDIATE_ALL_RESET):
      manualPose = defaultPose;
      return;
    case (NO_RESET):  // Do nothing
    default:  // Do nothing
      break;
  }

  // Override posing velocity commands depending on pose reset mode
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    if (resetTranslation[i])
    {
      if (translationPosition[i] < defaultTranslation[i])
      {
        translationVelocityInput[i] = 1.0;
      }
      else if (translationPosition[i] > defaultTranslation[i])
      {
        translationVelocityInput[i] = -1.0;
      }
    }

    if (resetRotation[i])
    {
      if (rotationPosition.toEulerAngles()[i] < defaultRotation[i])
      {
        rotationVelocityInput[i] = 1.0;
      }
      else if (rotationPosition.toEulerAngles()[i] > defaultRotation[i])
      {
        rotationVelocityInput[i] = -1.0;
      }
    }
  }

  Vector3d translationVelocity = clamped(translationVelocityInput, 1.0) * params_.max_translation_velocity;
  Vector3d rotationVelocity = clamped(rotationVelocityInput, 1.0) * params_.max_rotation_velocity;

  Vector3d newTranslationPosition = translationPosition + translationVelocity * params_.time_delta;
  Quat newRotationPosition = rotationPosition * Quat(Vector3d(rotationVelocity * params_.time_delta));

  Vector3d translationLimit = Vector3d(0, 0, 0);
  Vector3d rotationLimit = Vector3d(0, 0, 0);

  // Zero velocity input depending on position limitations
  for (int i = 0; i < 3; i++)  // For each axis (x,y,z)/(roll,pitch,yaw)
  {
    // TRANSLATION
    // Assign correct translation limit based on velocity direction and reset command
    translationLimit[i] = sign(translationVelocity[i]) * maxTranslation[i];
    if (resetTranslation[i] && defaultTranslation[i] < maxTranslation[i] && defaultTranslation[i] > -maxTranslation[i])
    {
      translationLimit[i] = defaultTranslation[i];
    }

    bool positiveTranslationVelocity = sign(translationVelocity[i]) > 0;
    bool exceedsPositiveTranslationLimit =
        positiveTranslationVelocity && (newTranslationPosition[i] > translationLimit[i]);
    bool exceedsNegativeTranslationLimit =
        !positiveTranslationVelocity && (newTranslationPosition[i] < translationLimit[i]);

    // Zero velocity when translation position reaches limit
    if (exceedsPositiveTranslationLimit || exceedsNegativeTranslationLimit)
    {
      translationVelocity[i] = (translationLimit[i] - translationPosition[i]) / params_.time_delta;
    }

    // ROTATION
    // Assign correct rotation limit based on velocity direction and reset command
    rotationLimit[i] = sign(rotationVelocity[i]) * maxRotation[i];
    if (resetRotation[i] && defaultRotation[i] < maxRotation[i] && defaultRotation[i] > -maxRotation[i])
    {
      rotationLimit[i] = defaultRotation[i];
    }

    bool positiveRotationVelocity = sign(rotationVelocity[i]) > 0;
    bool exceedsPositiveRotationLimit =
        positiveRotationVelocity && (newRotationPosition.toEulerAngles()[i] > rotationLimit[i]);
    bool exceedsNegativeRotationLimit =
        !positiveRotationVelocity && (newRotationPosition.toEulerAngles()[i] < rotationLimit[i]);

    // Zero velocity when rotation position reaches limit
    if (exceedsPositiveRotationLimit || exceedsNegativeRotationLimit)
    {
      rotationVelocity[i] = (rotationLimit[i] - rotationPosition.toEulerAngles()[i]) / params_.time_delta;
    }
  }

  // Update position according to limitations
  manualPose.position_ = translationPosition + translationVelocity * params_.time_delta;
  manualPose.rotation_ =
      rotationPosition *
      Quat(Vector3d(rotationVelocity * params_.time_delta));  // BUG: Adding pitch and roll simultaneously adds unwanted yaw
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::imuCompensation(ImuData imuData, Quat targetRotation)
{
  // There are two orientations per quaternion and we want the shorter/smaller difference.
  double dot = targetRotation.dot(~imuData.orientation);
  if (dot < 0.0)
  {
    targetRotation = -targetRotation;
  }

  // PID gains
  double kD = parameters.rotation_compensation_derivative_gain;
  double kP = parameters.rotation_compensation_proportional_gain;
  double kI = parameters.rotation_compensation_integral_gain;

  rotationPositionError = imuData.orientation.toEulerAngles() - targetRotation.toEulerAngles();
  rotationAbsementError += rotationPositionError * params_.time_delta;  // Integration of angle position error (absement)

  // Low pass filter of IMU angular velocity data
  double smoothingFactor = 0.15;
  rotationVelocityError = smoothingFactor * imuData.angular_velocity + (1 - smoothingFactor) * rotationVelocityError;

  Vector3d rotationCorrection = -(kD * rotationVelocityError + kP * rotationPositionError + kI * rotationAbsementError);
  rotationCorrection[2] = targetRotation.toEulerAngles()[2];  // No compensation in yaw rotation

  double stabilityThreshold = 100;

  if (rotationCorrection.norm() > stabilityThreshold)
  {
    ROS_FATAL("IMU rotation compensation became unstable! Adjust PID parameters.\n");
    ASSERT(rotationCorrection.norm() < stabilityThreshold);
  }
  else
  {
    imuPose = Pose::identity();
    imuPose.rotation_ = Quat(rotationCorrection);
  }

  // TRANSLATION COMPENSATION
  // DOES NOT CURRENTLY WORK FULLY
  /*
  //PID gains
  double kD = params.translationCompensationDerivativeGain;
  double kP = params.translationCompensationProportionalGain;
  double kI = params.translationCompensationIntegralGain;

  //Remove gravity
  Vector3d gravity = {0.0,0.0,9.75};
  Vector3d orientedGravity = imuData.orientation.rotateVector(gravity);
  Vector3d dynamicLinearAcceleration = imuData.linearAcceleration - orientedGravity;

  double decayRate = 2.3;

  //Low pass filter of IMU linear acceleration data (after removing acceleration due to gravity)
  double smoothingFactor = 0.15;
  translationAccelerationError = smoothingFactor*dynamicLinearAcceleration +
  (1-smoothingFactor)*translationAccelerationError;

  //Integrate for velocity and position and absement
  translationVelocityError += translationAccelerationError*timeDelta - decayRate*timeDelta*translationVelocityError;
  translationPositionError += translationVelocityError*timeDelta - decayRate*timeDelta*translationPositionError;
  translationAbsementError += translationPositionError*timeDelta;

  Vector3d translationCorrection = kD*translationVelocityError + kP*translationPositionError +
  kI*translationAbsementError;
  translationCorrection[2] = 0; //No compensation in z translation (competes with impedance controller)
  */
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::inclinationCompensation(ImuData imuData)
{
  Quat compensationCombined = manualPose.rotation_ * autoPoseDefault.rotation_;
  Quat compensationRemoved = imuData.orientation * compensationCombined.inverse();
  Vector3d eulerAngles = compensationRemoved.toEulerAngles();

  double lateralCorrection = walker->body_clearance_ * walker->maximum_body_height_ * tan(eulerAngles[0]);
  double longitudinalCorrection = -walker->body_clearance_ * walker->maximum_body_height_ * tan(eulerAngles[1]);

  inclinationPose = Pose::identity();
  inclinationPose.position_[0] = clamped(longitudinalCorrection, -parameters.max_translation[0], parameters.max_translation[0]);
  inclinationPose.position_[1] = clamped(lateralCorrection, -parameters.max_translation[1], parameters.max_translation[1]);
}

/***********************************************************************************************************************
 * Returns roll and pitch rotation values to compensate for roll/pitch of IMU and keep body at target rotation
***********************************************************************************************************************/
void PoseController::impedanceControllerCompensation(double deltaZ[3][2])
{
  int loadedLegs = 6;
  double averageDeltaZ = 0.0;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      averageDeltaZ += deltaZ[l][s];
    }
  }
  averageDeltaZ /= loadedLegs;

  deltaZPose = Pose::identity();
  deltaZPose.position_[2] = clamped(abs(averageDeltaZ), -parameters.max_translation[2], parameters.max_translation[2]);
}

/***********************************************************************************************************************
 * Attempts to develop pose to position body such that there is a zero sum of moments from the force acting on the load
 * bearing feet
***********************************************************************************************************************/
void PoseController::calculateDefaultPose()
{
  int legsLoaded = 0.0;
  int legsTransitioningStates = 0.0;

  // Check how many legs are load bearing and how many are transitioning states
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      LegState state = model->leg_states_[l][s];
      if (state == WALKING || state == MANUAL_TO_WALKING)
      {
        legsLoaded++;
      }

      if (state == MANUAL_TO_WALKING || state == WALKING_TO_MANUAL)
      {
        legsTransitioningStates++;
      }
    }
  }

  // Only update the sum of moments if specific leg is WALKING and ALL other legs are in WALKING OR MANUAL state.
  if (legsTransitioningStates != 0.0)
  {
    if (recalculateOffset)
    {
      Vector3d zeroMomentOffset(0, 0, 0);
      for (int l = 0; l < 3; l++)
      {
        for (int s = 0; s < 2; s++)
        {
          LegState state = model->leg_states_[l][s];
          if (state == WALKING || state == MANUAL_TO_WALKING)
          {
            zeroMomentOffset[0] += walker->identityTipPositions[l][s][0];
            zeroMomentOffset[1] += walker->identityTipPositions[l][s][1];
          }
        }
      }
      zeroMomentOffset /= legsLoaded;
      defaultPose.position_[0] = clamped(zeroMomentOffset[0], -parameters.max_translation[0], parameters.max_translation[0]);
      defaultPose.position_[1] = clamped(zeroMomentOffset[1], -parameters.max_translation[1], parameters.max_translation[1]);
      recalculateOffset = false;
    }
  }
  else
  {
    recalculateOffset = true;
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

  int num_iterations = max(1, int(roundToInt(time_to_step / params_.time_delta));
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

  int num_iterations = max(1, int(roundToInt(time_to_step / params_.time_delta));
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
