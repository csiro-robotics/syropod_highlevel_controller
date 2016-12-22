#include "../include/simple_hexapod_controller/walkController.h"


/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by
 * finding the largest footprint radius that each leg can achieve for the
 * specified level of clearance.
***********************************************************************************************************************/
WalkController::WalkController(Model *model, Parameters p)
{
  init(model, p);
}

/***********************************************************************************************************************
 * Initialisation
***********************************************************************************************************************/
void WalkController::init(Model *model, Parameters p)
{
  params_ = p;
  walk_state_ = STOPPED;

  step_clearance_ = params_.step_clearance;
  step_depth_ = params_.step_depth;
  body_clearance_ = params_.body_clearance;
  time_delta_ = params_.time_delta;

  ASSERT(stepClearance >= 0 && stepClearance < 1.0);  
  
  //TBD Assumes universal physical leg characteristics
  typedef std::map<std::string, Leg*>::iterator it_type;
  it_type first_leg = model->getLegContainer().begin();
  Leg3DOF reference_leg = first_leg->second();
  
  //Calculate max body height
  double tibia_length = reference_leg->getTibiaLinkLength();
  double femur_length = reference_leg->getFemurLinkLength();  
  double min_tibia_angle = max(0.0, reference_leg->getMinTibiaJointLimit());
  double max_tibia_angle = reference_leg->getMaxTibiaJointLimit();
  double min_femur_angle = reference_leg->getMinFemurJointLimit();
  double max_femur_angle = min(-min_femur_angle, pi / 2.0 - atan2(tibia_length * sin(min_tibia_angle), femur_length + tibia_length * cos(min_tibia_angle))); //TBD pi/2.0?

  maximum_body_height_ = femur_length * sin(max_femur_angle) + tibia_length * sin(max_femur_angle + clamped(pi / 2.0 - max_femur_angle, min_tibia_angle, max_tibia_angle));

  ASSERT(stepClearance * maximumBodyHeight <= 2.0 * model->legs[0][0].femurLength);  // impossible to lift this high

  // If undefined - work out a best value to maximise circular footprint for given step clearance
  if (body_clearance_ == -1)
  {
    // in this case we assume legs have equal characteristics
    body_clearance_ = (reference_leg->getMinLegLength() + step_clearance_ * maximum_body_height_) / maximum_body_height_;
  }
  ASSERT(bodyClearance >= 0 && bodyClearance < 1.0);

  min_footprint_radius_ = 1e10;

  typedef std::map<std::string, Leg*>::iterator it_type;
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    double coxa_length = leg->getCoxaLinkLength();
    double tibia_length = leg->getTibiaLinkLength();
    double femur_length = leg->getFemurLinkLength();
    double min_virtual_leg_length = leg->getMinLegLength();
    double max_virtual_leg_length = leg->getMaxLegLength();
    
    // find biggest circle footprint inside the pie segment defined by the body clearance and the yaw limits
    
    // Downward angle of virtual leg from zero position
    double virtual_leg_angle = asin((body_clearance_ * maximum_body_height_) / leg->getMaxLegLength());
    double horizontal_range = 0;
    double rad = 1e10;

    if (virtual_leg_angle > -min_femur_angle)  //Straight leg can't reach reach ground for desired body clearance without exceeding min femur angle
    {
      double vertical_body_to_femur_joint = femur_length * sin(-min_femur_angle);
      double vertical_ground_to_tibia_joint = body_clearance_ * maximum_body_height_ - vertical_body_to_femur_joint;
      ASSERT(extraHeight <= leg->tibiaLength);  // this shouldn't be possible with bodyClearance < 1
      double horizontal_tibia_joint_to_tip = sqrt(sqr(tibia_length) - sqr(vertical_ground_to_tibia_joint));
      horizontal_range = femur_length * cos(-min_femur_angle) + horizontal_tibia_joint_to_tip;
    }
    else
    {
      horizontal_range = sqrt(sqr(max_virtual_leg_length) - sqr(body_clearance_ * maximum_body_height_));
    }
    horizontal_range *= params_.leg_span_scale; //Scale range according to user defined parameter

    double half_coxa_joint_range = leg->getMaxCoxaJointLimitAroundDefault();
    //double cotanTheta = tan(0.5 * pi - theta);
    //ALTERNATIVE ALGORITHM FOR RADIUS OF CIRCLE INSCRIBED BY A SECTOR
    //rad = min(rad, solveQuadratic(sqr(cotanTheta), 2.0 * horizontalRange, -sqr(horizontalRange)));
    rad = horizontal_range*sin(half_coxa_joint_range)/(1+sin(half_coxa_joint_range)); 
    ASSERT(rad > 0.0);  // cannot have negative radius

    // we should also take into account the stepClearance not getting too high for the leg to reach
    double leg_tip_body_clearance = max(0.0, body_clearance_ - step_clearance_) * maximum_body_height_;

    // if footprint radius due to lift is smaller due to yaw limits, reduce this minimum radius
    if (leg_tip_body_clearance < min_virtual_leg_length)
    {
      rad = min(rad, (horizontal_range - sqrt(sqr(min_virtual_leg_length) - sqr(leg_tip_body_clearance))) / 2.0);
    }
    ASSERT(rad > 0.0);  // cannot have negative radius, step height is too high to allow any footprint

    double foot_spread_distances = coxa_length + horizontal_range - rad;

    // FootprintDownscale scales the tip footprint down as the step cycle sometimes exceeds the ground footprint in
    // order to maintain velocity
    min_footprint_radius_ = min(min_footprint_radius_, rad * params_.footprint_downscale);

    Vector3d identity_tip_position = leg->getFemurOffset();
    identity_tip_position[0] += foot_spread_distances*sin(leg->getDefaultCoxaJointPosition());
    identity_tip_position[1] += foot_spread_distances*-cos(leg->getDefaultCoxaJointPosition());
    identity_tip_position[2] += -body_clearance_ * maximum_body_height_;
    identity_tip_position *= leg->getMirrorDir();

    leg->setLegStepper(this, identity_tip_position);
  }
  
  // check for overlapping radii
  double min_gap = 1e10;
  LegStepper* reference_leg_stepper_1 = reference_leg->getLegStepper();
  LegStepper* reference_leg_stepper_2 = ((first_leg+2)->second())->getLegStepper();
  LegStepper* reference_leg_stepper_3 = ((first_leg+4)->second())->getLegStepper();
  
  Vector3d positive_difference = reference_leg_stepper_1->getDefaultTipPosition() - reference_leg_stepper_2->getDefaultTipPosition();
  positive_difference[2] = 0.0;
  min_gap = min(min_gap, positive_difference.norm() - 2.0 * min_footprint_radius_);
  
  positive_difference = reference_leg_stepper_2->getDefaultTipPosition() - reference_leg_stepper_1->getDefaultTipPosition();
  positive_difference[2] = 0.0;
  min_gap = min(min_gap, positive_difference.norm() - 2.0 * min_footprint_radius_);

  if (min_gap < 0.0)
  {
    min_footprint_radius_ += min_gap * 0.5;
  }

  // Stance radius based around front-left leg to ensure positive values (due to coordinate frame system: x=forward,
  // y=left)
  double x_position = reference_leg_stepper_1->getDefaultTipPosition()[0];
  double y_position = reference_leg_stepper_1->getDefaultTipPosition()[1];
  stance_radius_ = Vector2d(x_position, y_position).norm();

  current_linear_velocity_ = Vector2d(0, 0);
  current_angular_velocity_ = 0;

  pose_.rotation_ = Quat(1, 0, 0, 0);
  pose_.position_ = Vector3d(0, 0, body_clearance_ * maximum_body_height_);
  
  setGaitParams(p);
}

/***********************************************************************************************************************
 * Sets parameters associated with gait
***********************************************************************************************************************/
void WalkController::setGaitParams(Model* model, Parameters p)
{
  params_ = p;
  stance_end_ = params_.stance_phase * 0.5;
  swing_start_ = stance_end_;
  swing_end_ = swing_start_ + params_.swing_phase;
  stance_start_ = swing_end_;

  ROS_ASSERT(params_.stance_phase % 2 == 0);
  ROS_ASSERT(params_.swing_phase % 2 == 0);

  // Normalises the step phase length to match the total number of iterations over a full step
  int base_phase_length = params_.stance_phase + params_.swing_phase;
  double swing_ratio = (params_.swing_phase) / base_phase_length;  // Used to modify stepFreqency based on gait
  phase_length_ = (roundToInt((1.0 / (2.0 * params_.step_frequency * time_delta_)) / (base_phase_length * swing_ratio)) *
                 (base_phase_length * swing_ratio)) /
                swing_ratio;
  step_frequency_ = 1 / (phase_length_ * time_delta_);  // adjust stepFrequency to match corrected phaseLength
  ASSERT(phaseLength % basePhaseLength == 0);
  int normaliser = phase_length_ / base_phase_length;
  stance_end_ *= normaliser;
  swing_start_ *= normaliser;
  swing_end_ *= normaliser;
  stance_start_ *= normaliser;

  typedef std::map<std::string, Leg*>::iterator it_type;
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    int index = leg->getIDNumber();
    int multiplier = params_.offset_multiplier[index];
    LegStepper* leg_stepper = leg->getLegStepper();
    leg_stepper->setPhaseOffset((int(params_.phase_offset * normaliser) * multiplier) % phase_length_);
  }
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/
void WalkController::updateWalk(Vector2d linear_velocity_input, double angular_velocity_input, Model *model)
{
  double on_ground_ratio = double(phase_length_ - (swing_end_ - swing_start_)) / double(phase_length_);

  Vector2d new_linear_velocity;
  double new_angular_velocity;

  // Distance: 2.0*minFootprintRadius, Time: onGroundratio*(1/stepFrequency) where stepFrequency is FULL step cycles/s)
  double max_linear_speed = 2.0 * min_footprint_radius_ * step_frequency_ / on_ground_ratio;                                            
  double max_angular_speed = max_linear_speed / stance_radius_;

  // Get new angular/linear velocities according to input mode
  if (walk_state_ != STOPPING)
  {
    if (params_.velocity_input_mode == "throttle")
    {
      new_linear_velocity = clamped(linear_velocity_input, 1.0) * max_linear_speed;  // Forces input between -1.0/1.0
      new_angular_velocity = clamped(angular_velocity_input, -1.0, 1.0) * max_angular_speed;
      
      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (1 - abs(angular_velocity_input));  
    }
    else if (params_.velocity_input_mode == "real")
    {
      new_linear_velocity = clamped(linear_velocity_input, max_linear_speed);
      new_angular_velocity = clamped(angular_velocity_input, -max_angular_speed, max_angular_speed);

      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (max_angular_speed != 0 ? (1 - abs(new_angular_velocity / max_angular_speed)) : 0.0);  

      if (linear_velocity_input.norm() > max_linear_speed)
      {
        ROS_WARN_THROTTLE(10, "Input linear speed (%f) exceeds maximum linear speed (%f) and has been clamped.",
                          linearVelocityInput.norm(), maxLinearSpeed);
      }
      if (abs(angular_velocity_input) > max_angular_speed)
      {
        ROS_WARN_THROTTLE(10, "Input angular velocity (%f) exceeds maximum angular speed (%f) and has been clamped.",
                          abs(angularVelocityInput), maxAngularSpeed);
      }
    }
  }
  else
  {
    new_linear_velocity = Vector2d(0.0, 0.0);
  }

  // Angular Acceleration Control
  double angular_acceleration = new_angular_velocity - current_angular_velocity_;
  
  // Calculate max angular acceleration if specified by parameter
  if (params_.max_angular_acceleration == -1.0)
  {
    // Ensures tip of last leg to make first swing does not move further than 
    // footprint radius before starting first swing (s=0.5*a*(t^2))
    double theta = acos(1 - (sqr(min_footprint_radius_) / (2.0 * sqr(stance_radius_))));  // Max angle within footprint
    params_.max_angular_acceleration = 2.0 * theta / sqr(((phase_length_ - (swing_end_ - swing_start_) * 0.5) * time_delta_));
  }
  // Update angular velocity according to acceleration limits
  if (abs(angular_acceleration) > 0.0)
  {
    current_angular_velocity_ +=
        angular_acceleration * min(1.0, params_.max_angular_acceleration * time_delta_ / abs(angular_acceleration));
  }

  // Linear Acceleration Control
  Vector2d linear_acceleration = new_linear_velocity - current_linear_velocity_;
  // Calculate max acceleration if specified by parameter
  if (params_.max_linear_acceleration == -1.0)
  {
    // Ensures tip of last leg to make first swing does not move further than
    // footprint radius before starting first swing (s=0.5*a*(t^2))
    params_.max_linear_acceleration =
        2.0 * min_footprint_radius_ / sqr(((phase_length_ - (swing_end_ - swing_start_) * 0.5) * time_delta_));
  }
  // Update linear velocity according to acceleration limits
  if (linear_acceleration.norm() > 0.0)
  {
    current_linear_velocity_ +=
        linear_acceleration * min(1.0, params_.max_linear_acceleration * time_delta_ / linear_acceleration.norm());
  }

  bool has_velocity_command = linear_velocity_input.norm() || angular_velocity_input;

  // Check that all legs are in WALKING state
  typedef std::map<std::string, Leg*>::iterator it_type;
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF leg = it->second();
    if (has_velocity_command && leg->getState() != WALKING)
    {
      has_velocity_command = false;
      if (angular_velocity_input == 0)
      {
	ROS_INFO_THROTTLE(2, "Unable to walk whilst manually manipulating legs, make sure all legs are in walking "
			      "state.\n");
      }
    }
  }

  // State transitions for robot state machine.
  // State transition: STOPPED->STARTING
  int num_legs = model->getLegCount();
  if (walk_state_ == STOPPED && has_velocity_command)
  {
    walk_state_ = STARTING;
    for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
    {
      Leg3DOF* leg = it->second();
      LegStepper* leg_stepper = leg->getLegStepper();
      leg_stepper->setPhase(leg_stepper->getPhaseOffset() - 1);
    }
  }
  // State transition: STARTING->MOVING
  else if (walk_state_ == STARTING && legs_at_correct_phase_ == num_legs && legs_completed_first_step_ == num_legs)
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
  else if (walk_state_ == STOPPING && legs_at_correct_phase_ == num_legs)
  {
    legs_at_correct_phase_ = 0;
    walk_state_ = STOPPED;
  }

  // Robot State Machine
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    LegStepper* leg_stepper = leg->getLegStepper(); 
    
    Vector3d local_tip_position = leg->getLocalTipPosition();
    Vector3d stride_vector = on_ground_ratio * (current_linear_velocity_ + current_angular_velocity_ * Vector2d(-local_tip_position[1], local_tip_position[0])) / step_frequency_;
    leg_stepper->setStrideVector(stride_vector);    

    if (walk_state_ == STARTING)
    {
      leg_stepper->iteratePhase(this);

      // Check if all legs have completed one step
      if (legs_at_correct_phase_ == num_legs)
      {
	if (leg_stepper->getPhase() == swing_end_ && !leg_stepper->hasCompletedFirstStep())
	{
	  leg_stepper->setCompletedFirstStep(true);
	  legs_completed_first_step_++;
	}
      }

      // Force any leg state into STANCE if it starts offset in a mid-swing state
      if (!leg_stepper->isAtCorrectPhase())
      {
	if (leg_stepper->getPhaseOffset() > swing_start_ && leg_stepper->getPhaseOffset() < swing_end_)  // SWING STATE
	{
	  if (leg_stepper->getPhase() == swing_end_)
	  {
	    legs_at_correct_phase_++;
	    leg_stepper->setAtCorrectPhase(true);
	  }
	  else
	  {
	    leg_stepper->setStepState(FORCE_STANCE);
	  }
	}
	else
	{
	  legs_at_correct_phase_++;
	  leg_stepper->setAtCorrectPhase(true);
	}
      }
    }
    else if (walk_state_ == STOPPING)
    {
      if (!leg_stepper->isAtCorrectPhase())
      {
	leg_stepper->iteratePhase();

	// Reference leg (AL) only "meets target" after completing extra step AND returning to zero phase
	if (leg->getIDNumber() == 0 && leg_stepper->getStepState() == FORCE_STOP && leg_stepper->getPhase() == 0)
	{
	  leg_stepper->setAtCorrectPhase(true);
	  legs_at_correct_phase_++;
	  leg_stepper->setStepState(STANCE);
	}
      }

      // All legs (except reference leg) must make one extra step after receiving stopping signal
      if (leg_stepper->getStrideVector().norm() == 0 && leg_stepper->getPhase() == swing_end_)
      {
	leg_stepper->setStepState(FORCE_STOP);
	if (leg->getIDNumber() != 0)
	{
	  if (!leg_stepper->isAtCorrectPhase())
	  {
	    leg_stepper->setAtCorrectPhase(true);
	    legs_at_correct_phase_++;
	  }
	}
      }
    }
    else if (walk_state_ == MOVING)
    {
      leg_stepper->iteratePhase();
      leg_stepper->setAtCorrectPhase(false);
    }
    else if (walk_state_ == STOPPED)
    {
      leg_stepper->setAtCorrectPhase(false);
      leg_stepper->setCompletedFirstStep(false);
      leg_stepper->setPhase(0);
      leg_stepper->setStepState(STANCE);
    }
  }

  // Step State Machine
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    LegStepper* leg_stepper = leg->getLegStepper(); 
    int phase = leg_stepper->getPhase();
    StepState step_state = leg_stepper.getStepState();

    // Force leg state as STANCE for STARTING robot state
    if (step_state == FORCE_STANCE)
    {
      leg_stepper->setStepState(STANCE);
    }
    // Force leg state as FORCE_STOP for STOPPING robot state
    else if (step_state == FORCE_STOP)
    {
      leg_stepper->setStepState(FORCE_STOP);
    }
    else if (phase >= swing_start_ && phase < swing_end_)
    {
      leg_stepper->setStepState(SWING);
    }
    else if (phase < stance_end_ || phase >= stance_start_)
    {
      leg_stepper->setStepState(STANCE);
    }
  }

  // Update tip positions and apply inverse kinematics
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    LegStepper* leg_stepper = leg->getLegStepper(); 

    if (leg->getLegState() == WALKING)
    {
      if (walk_state_ != STOPPED)
      {
	leg_stepper.updatePosition();  // updates current tip position through step cycle
      }
    }
  }

  // RVIZ
  if (walk_state_ != STOPPED)
  {
    Vector2d push = current_linear_velocity_ * time_delta_;
    pose_.position_ += pose_.rotation_.rotateVector(Vector3d(push[0], push[1], 0));
    pose_.rotation_ *= Quat(Vector3d(0.0, 0.0, current_angular_velocity_ * time_delta_));
  }
  // RVIZ
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/

void WalkController::updateManual(int primary_leg_selection_ID, Vector3d primary_tip_velocity_input,
                                  int secondary_leg_selection_ID, Vector3d secondary_tip_velocity_input,
				  Model* model)
{
  typedef std::map<std::string, Leg*>::iterator it_type;
  for (it_type it = model->getLegContainer().begin(); it != model->getLegContainer()->end(); ++it)
  {
    Leg3DOF* leg = it->second();
    LegStepper* leg_stepper = leg->getLegStepper();  
    if (leg->getLegState() == MANUAL)
    {
      Vector3d tipVelocityInput;
      int selected_leg_ID = leg->getIDNumber();
      if (selected_leg_ID == primary_leg_selection_ID)
      {
	tipVelocityInput = primary_tip_velocity_input;
      }
      else if (selected_leg_ID == secondary_leg_selection_ID)
      {
	tipVelocityInput = secondary_tip_velocity_input;
      }

      if (params_.leg_manipulation_mode == "joint_control")  // HACK Below
      {
	double coxa_joint_velocity = tipVelocityInput[0] * params_.max_rotation_velocity * time_delta_;
	double femur_joint_velocity = tipVelocityInput[1] * params_.max_rotation_velocity * time_delta_;
	double tibia_joint_velocity = tipVelocityInput[2] * params_.max_rotation_velocity * time_delta_;
	double new_coxa_angle = leg.getCoxaJointPosition() += coxa_joint_velocity;
	double new_femur_angle = leg.getFemurJointPosition() += femur_joint_velocity;
	double new_tibia_angle = leg.getTibiaJointPosition() += tibia_joint_velocity;
	Vector3d new_tip_position = leg.calculateFK(new_coxa_angle, new_femur_angle, new_tibia_angle);
	leg_stepper->setCurrentTipPosition(new_tip_position);
      }
      else if (params_.leg_manipulation_mode == "tip_control")
      {
	Vector3d tip_position_change = tipVelocityInput * params_.max_translation_velocity * time_delta_;
	Vector3d new_tip_position = leg_stepper->getCurrentTipPosition() + tip_position_change;
	leg_stepper->setCurrentTipPosition(new_tip_position);
      }
    }
  }
}

/***********************************************************************************************************************
 * Leg stepper object constructor
***********************************************************************************************************************/
LegStepper::LegStepper(WalkController* walker, Leg3DOF* leg, Vector3d identity_tip_position) 
  : walker_(walker)
  , leg_(leg)
  , default_tip_position_(identity_tip_position)
  , current_tip_position_(identity_tip_position)
{
  stride_vector_ = Vector2d(0.0,0.0);
  swing_height_ = walker->getStepClearance() * walker->getMaxBodyHeight();
  stance_depth_ = walker->getStepDepth() * walker->getMaxBodyHeight();
};

/***********************************************************************************************************************
 * Iterates the step phase and updates the progress variables
***********************************************************************************************************************/
void LegStepper::iteratePhase()
{
  int phase_length = walker_->getPhaseLength();
  int swing_start = walker_->getSwingStart();
  int swing_end = walker_->getSwingEnd();
  int stance_start = walker_->getStanceStart();
  int stance_end = walker_->getStanceEnd();
  
  phase_ = (phase_ + 1) % (phase_length);

  if (step_state_ == SWING)
  {
    swing_progress_ = double(phase_ - swing_start + 1) / double(swing_end - swing_start);
    stance_progress_ = -1.0;
  }
  else if (step_state_ == STANCE)
  {
    stance_progress_ = double(mod(phase_ + (phase_length - stance_start), phase_length) + 1) /
                     double(mod(stance_end - stance_start, phase_length));
    swing_progress_ = -1.0;
  }
}

/***********************************************************************************************************************
 * Updates position of tip using tri-quartic bezier curve tip trajectory engine. Calculates change in tip position using
 * the derivatives of three quartic bezier curves, two for swing phase and one for stance phase. Each Bezier curve uses
 * 5 control nodes designed specifically to give a C2 smooth trajectory for the entire step cycle.
***********************************************************************************************************************/
void LegStepper::updatePosition()
{
  int phase_length = walker_->getPhaseLength();
  int swing_start = walker_->getSwingStart();
  int swing_end = walker_->getSwingEnd();
  int stance_start = walker_->getStanceStart();
  int stance_end = walker_->getStanceEnd();
  
  // Swing Phase
  if (step_state_ == SWING)
  {
    int iteration = phase_ - swing_start + 1;
    double swing_length = swing_end - swing_start;
    swing_delta_t_ = calculateDeltaT(swing_length);
    int num_iterations = 2.0 / swing_delta_t_;

    // Save initial tip position at beginning of swing
    if (iteration == 1)
    {
      swing_origin_tip_position_ = current_tip_position_;
    }

    // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    Vector3d delta_pos;
    double t1 = 0;
    double t2 = 0;
    Vector3d stride_vector_3d = Vector3d(stride_vector_[0], stride_vector_[1], 0.0);

    if (iteration <= num_iterations / 2)
    {
      generateSwingControlNodes(stride_vector_3d);
      t1 = iteration * swing_delta_t_;
      delta_pos = swing_delta_t_ * quarticBezierDot(swing_1_nodes_, t1);  //
    }
    else
    {
      // Update values of NEXT stance curve for use in calculation of secondary swing control nodes
      int stance_length = mod(stance_end - stance_start, phase_length);
      stance_delta_t_ = calculateDeltaT(STANCE, stance_length);
      stance_origin_tip_position_ = default_tip_position_ + 0.5 * stride_vector_3d;
      generateStanceControlNodes(stride_vector_3d);

      generateSwingControlNodes(stride_vector_3d);
      t2 = (iteration - num_iterations / 2) * swing_delta_t_;
      delta_pos = swing_delta_t_ * quarticBezierDot(swing_2_nodes_, t2);
    }

    current_tip_position_ += delta_pos;
    current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    if (t1 < swing_delta_t_)
    {
      t1 = 0.0;
    }
    if (t2 < swing_delta_t_)
    {
      t2 = 0.0;
    }

    if (leg_->getIDNumber() == 0)
    {
      ROS_DEBUG_COND(params->debug_swing_trajectory, "SWING TRAJECTORY_DEBUG - ITERATION: %d\t\tTIME: %f:%f\t\tORIGIN: %f:%f:%f\t\tPOS: %f:%f:%f\t\tTARGET: %f:%f:%f\n", iteration, t1, t2, swing_origin_tip_position_[0], swing_origin_tip_position_[1], swing_origin_tip_position_[2], current_tip_position_[0], current_tip_position_[1], current_tip_position_[2], swing_2_control_nodes_[4][0], swing_2_control_nodes_[4][1], swing_2_control_nodes_[4][2]);
    }      
  }
  // Stance phase
  else if (step_state_ == STANCE)
  {
    int phase_length = walker_->getPhaseLength();
    int stance_start = completed_first_step_ ? walker_->getStanceStart() : phase_offset_;
    int stance_end = walker_->getStanceEnd();
    int stance_length = mod(stance_end - stance_start, phase_length);
    stance_delta_t_ = calculateDeltaT(STANCE, stance_length);

    int iteration = mod(phase_ + (phase_length - stance_start), phase_length) + 1;

    // Save initial tip position at beginning of swing
    if (iteration == 1)
    {
      stance_origin_tip_position_ = current_tip_position_;
    }

    // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    Vector3d delta_pos;
    double t;

    // Scales stride vector according to stance length specifically for STARTING state of walker
    Vector3d stride_vector_3d = Vector3d(stride_vector_[0], stride_vector_[1], 0.0);
    stride_vector_3d *= double(stance_length) / (mod(swing_start - swing_end, phase_length));

    generateStanceControlNodes(stride_vector_3d);
    t = iteration * stance_delta_t_;
    delta_pos = stance_delta_t_ * quarticBezierDot(stance_nodes_, t);

    current_tip_position_ += delta_pos;
    current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    if (t < stance_delta_t_)
    {
      t = 0.0;
    }
    
    if (leg_->getIDNumber() == 0)
    {
      ROS_DEBUG_COND(params->debug_stance_trajectory, "STANCE TRAJECTORY_DEBUG - ITERATION: %d\t\tTIME: %f\t\tORIGIN: %f:%f:%f\t\tPOS: %f:%f:%f\t\tTARGET: %f:%f:%f\n", iteration, t, stance_origin_tip_position_[0], stance_origin_tip_position_[1], stance_origin_tip_position_[2], current_tip_position_[0], current_tip_position_[1], current_tip_position_[2], stance_control_nodes_[4][0], stance_control_nodes_[4][1], stance_control_nodes_[4][2]);      
    }
  }
}

/***********************************************************************************************************************
 * Calculates time deltas for use in quartic bezier curve tip trajectory calculations
***********************************************************************************************************************/
double LegStepper::calculateDeltaT(int length)
{
  int phase_length = walker_->getPhaseLength();
  double step_frequency = walker_->getStepFrequency();
  double time_delta = walker_->getTimeDelta();
  
  // Ensure compatible number of iterations
  int numIterations = roundToInt((double(length) / phase_length) / (step_frequency * time_delta) / 2.0) * 2.0;  
  if (step_state_ == SWING)
  {
    return 2.0 / numIterations;
  }
  else
  {
    return 1.0 / numIterations;
  }
}

/***********************************************************************************************************************
 * Generates control nodes for each quartic bezier curve of swing tip trajectory calculation
***********************************************************************************************************************/
void LegStepper::generateSwingControlNodes(Vector3d stride_vector)
{
  // Used to scale the difference between control nodes for the stance curve which has differing delta time values to
  // swing curves
  double bezier_scaler = stance_delta_t_ / swing_delta_t_;

  // Control nodes for swing quartic bezier curves - horizontal plane
  // Set for horizontal position continuity at transition between stance and primary swing curves (C0 Smoothness)
  swing_1_nodes_[0] = swing_origin_tip_position_;  
  // Set for horizontal velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing_1_nodes_[1] = swing_1_nodes_[0] + bezier_scaler * (stance_nodes_[4] - stance_nodes_[3]);
  // Set for horizontal acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing_1_nodes_[2] = swing_1_nodes_[1] + (swing_1_nodes_[1] - swing_1_nodes_[0]);  
  // Set to default tip position so max swing height and transition to 2nd swing curve occurs at default tip position
  swing_1_nodes_[4] = default_tip_position_;  
  // Set for horizontal acceleration continuity at transition between swing curves (C2 Smoothness) (symetrical curves)
  swing_1_nodes_[3] = (swing_1_nodes_[2] + swing_1_nodes_[4]) / 2.0;
  // Set for horizontal position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing_2_nodes_[0] = swing_1_nodes_[4]; 
  // Set for horizontal velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_2_nodes_[1] = swing_2_nodes_[0] + (swing_2_nodes_[0] - swing_1_nodes_[3]);
  // Set for horizontal velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
  swing_2_nodes_[3] = stance_nodes_[0] + bezier_scaler * (stance_nodes_[0] - stance_nodes_[1]);
  // Set for horizontal acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing_2_nodes_[2] = swing_2_nodes_[3] + swing_2_nodes_[3] - stance_nodes_[0];  
  // Set for horizontal position continuity at transition between secondary swing and stance curves (C0 Smoothness)
  swing_2_nodes_[4] = stance_nodes_[0];  

  // Control nodes for swing quartic bezier curves - vertical plane
  // Set for vertical position continuity at transition between stance and primary swing curves (C0 Smoothness)
  swing_1_nodes_[0][2] = swing_origin_tip_position_[2];
  // Set for vertical velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing_1_nodes_[1][2] = swing_1_nodes_[0][2] + bezier_scaler * (stance_nodes_[4][2] - stance_nodes_[3][2]);  
  // Set to default tip position plus swing height so max swing height and transition to 2nd swing curve occurs here
  swing_1_nodes_[4][2] = swing_1_nodes_[0][2] + swing_height_;  
  // Set for vertical acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing_1_nodes_[2][2] = swing_1_nodes_[0][2] + 2.0 * bezier_scaler * (stance_nodes_[4][2] - stance_nodes_[3][2]);
  // Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_1_nodes_[3][2] = swing_1_nodes_[4][2];  
  // Set for vertical position continuity at transition between primary and secondary swing curves (C0 Smoothness)
  swing_2_nodes_[0][2] = swing_1_nodes_[4][2];  
  // Set for vertical velocity continuity at transition between primary and secondary swing curves (C1 Smoothness)
  swing_2_nodes_[1][2] = swing_2_nodes_[0][2]; 
  // Set for vertical acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)
  swing_2_nodes_[2][2] = stance_nodes_[0][2] + 2.0 * bezier_scaler * (stance_nodes_[0][2] - stance_nodes_[1][2]);  
  // Set for vertical velocity continuity at transition between secondary swing and stance curves (C1 Smoothness)
  swing_2_nodes_[3][2] = stance_nodes_[0][2] + bezier_scaler * (stance_nodes_[0][2] - stance_nodes_[1][2]); 
  // Set for vertical position continuity at transition between secondary swing and stance curves (C0 Smoothness)
  swing_2_nodes_[4][2] = stance_nodes_[0][2];  
}

/***********************************************************************************************************************
 * Generates control nodes for quartic bezier curve of stance tip trajectory calculation
***********************************************************************************************************************/
void LegStepper::generateStanceControlNodes(Vector3d stride_vector)
{
  // Control nodes for stance quartic bezier curve - horizontal plane
  // Set as initial horizontal tip position
  stance_nodes_[0] = stance_origin_tip_position_;  
  // Set as target horizontal tip position
  stance_nodes_[4] = stance_origin_tip_position_ - stride_vector;  
  // Set for constant horizontal velocity in stance phase
  stance_nodes_[1] = stance_nodes_[4] + 0.75 * (stance_nodes_[0] - stance_nodes_[4]);  
  // Set for constant horizontal velocity in stance phase
  stance_nodes_[2] = stance_nodes_[4] + 0.5 * (stance_nodes_[0] - stance_nodes_[4]); 
  // Set for constant horizontal velocity in stance phase;
  stance_nodes_[3] = stance_nodes_[4] + 0.25 * (stance_nodes_[0] - stance_nodes_[4]);  

  // Control nodes for stance quartic bezier curve - vertical plane
  // Set as initial vertical tip position
  stance_nodes_[0][2] = stance_origin_tip_position_[2];  
  // Set as target vertical tip position
  stance_nodes_[4][2] = default_tip_position_[2];  
  // Set to control depth below ground level of stance trajectory, defined by stanceDepth
  stance_nodes_[2][2] = stance_nodes_[0][2] - stance_depth_;  
  // Set for vertical acceleration continuity at transition between secondary swing and stance curves (C2 Smoothness)                                                                    
  stance_nodes_[1][2] = (stance_nodes_[0][2] + stance_nodes_[2][2]) / 2.0;  
  // Set for vertical acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  stance_nodes_[3][2] = (stance_nodes_[4][2] + stance_nodes_[2][2]) / 2.0;  
}

/***********************************************************************************************************************
***********************************************************************************************************************/
