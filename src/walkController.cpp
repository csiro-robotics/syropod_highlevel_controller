#include "../include/simple_hexapod_controller/walkController.h"

/***********************************************************************************************************************
 * Determines the basic stance pose which the hexapod will try to maintain, by
 * finding the largest footprint radius that each leg can achieve for the
 * specified level of clearance.
***********************************************************************************************************************/
WalkController::WalkController(Model* model, Parameters* params)
  : model_(model)
  , params_(params)
  , walk_state_(STOPPED)
{
  init();
}

/***********************************************************************************************************************
 * Initialisation
***********************************************************************************************************************/
void WalkController::init(void)
{
	step_clearance_ = params_->step_clearance.current_value;
	step_depth_ = params_->step_depth.data;
	body_clearance_ = params_->body_clearance.current_value;
	time_delta_ = params_->time_delta.data;  

	//Find the maximum body height by finding the min possible vertical tip position for each leg 
	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		resetJointOrientationTracking();
		Leg* leg = leg_it_->second;
		Matrix4d transform = Matrix4d::Identity();
		for (link_it_ = leg->getLinkContainer()->begin(); link_it_ != leg->getLinkContainer()->end(); ++link_it_)
		{
			// If joint is twisted such that actuation increases vertical tip displacement
			// set joint angle toward ground bearing (within limits) and update transform
			Link* link = link_it_->second;
			double angle = 0.0;
			if (link->id_number != 0)
			{
				if (joint_twist_ < 0)
				{
					angle = clamped(ground_bearing_, link->actuating_joint->min_position, link->actuating_joint->max_position);
          
				}
				else if (joint_twist_ > 0)
				{
					angle = clamped(-ground_bearing_, link->actuating_joint->min_position, link->actuating_joint->max_position);	
				}
				ground_bearing_ -= abs(angle);
			}
			else
			{
				angle = link->angle;
			}
			transform = transform*createDHMatrix(link->offset, angle, link->length, link->twist);
			joint_twist_ += link->twist;
		}
		Vector4d result = transform*Vector4d(0,0,0,1);
		Vector3d min_vertical_tip_position = Vector3d(result[0], result[1], result[2]);
		maximum_body_height_ = min(maximum_body_height_, -min_vertical_tip_position[2]);
	}

	//Check that required body height is possible
	body_clearance_ = min(params_->body_clearance.current_value, maximum_body_height_);

	//Find workspace radius of tip on ground by finding maximum horizontal distance of each tip 
	double min_horizontal_range = UNASSIGNED_VALUE;
	double min_half_stance_yaw_range = UNASSIGNED_VALUE;
	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		resetJointOrientationTracking();
		Leg* leg = leg_it_->second;
		double half_stance_yaw_range = UNASSIGNED_VALUE;
		double distance_to_ground = body_clearance_;
		double horizontal_range = 0.0;
		for (link_it_ = ++leg->getLinkContainer()->begin(); link_it_ != leg->getLinkContainer()->end(); ++link_it_)
		{
			Link* link = link_it_->second;
			const Joint* joint = link->actuating_joint;
			joint_twist_ += joint->reference_link->twist;
			distance_to_ground += link->offset*cos(joint_twist_);
			
			// Find first joint able to move in x/y plane and assign a half yaw range value for the leg
			if (abs(joint_twist_) != pi/2 && half_stance_yaw_range == UNASSIGNED_VALUE)
			{
				half_stance_yaw_range = min(abs(leg->getStanceLegYaw() - (joint->reference_link->angle + joint->min_position)),
				abs(leg->getStanceLegYaw() - (joint->reference_link->angle + joint->max_position)));
			}
			
			if (joint_twist_ != 0.0)
			{
				Tip* tip = leg->getTip();
				Vector3d tip_position = tip->getPositionWorldFrame(true);
				double distance_to_tip = joint->getPositionJointFrame(true, tip_position).norm();
				double joint_angle;
				double virtual_link_length;
				if (abs(distance_to_tip*sin(joint_twist_)) > distance_to_ground)
				{	  
					double required_joint_angle = asin(-distance_to_ground/(abs(distance_to_tip*sin(joint_twist_))));
					joint_angle = clamped(required_joint_angle, joint->min_position, joint->max_position);
					virtual_link_length = (joint_angle == required_joint_angle) ? distance_to_tip : link->length;
				}
				else if (joint_twist_ > 0.0)
				{
					joint_angle = joint->min_position;
					virtual_link_length = link->length;
				}
				else if (joint_twist_ < 0.0)
				{
					joint_angle = joint->max_position;
					virtual_link_length = link->length;
				}
				horizontal_range += virtual_link_length*cos(joint_angle);
				distance_to_ground -= abs(virtual_link_length*sin(joint_angle)*sin(joint_twist_));
			}
			else
			{
				horizontal_range += link->length;
			}
			
			if (setPrecision(distance_to_ground, 3) <= 0.0)
			{
				break;
			}
		}
		
		min_horizontal_range = min(min_horizontal_range, horizontal_range);
		min_half_stance_yaw_range = min(min_half_stance_yaw_range, half_stance_yaw_range);
	}

	min_horizontal_range *= params_->leg_span.current_value;

	// Fitting largest circle within sector defined by yaw ranges and horizontal range
	workspace_radius_ = min_horizontal_range*sin(min_half_stance_yaw_range)/(1+sin(min_half_stance_yaw_range));

	// Calculate default stance tip positions as centre of workspace circle on the ground at required body height
	Vector3d previous_identity_position(0,0,0);
	leg_it_ = model_->getLegContainer()->begin();
	while (leg_it_ != model_->getLegContainer()->end())
	{
		Leg* leg = leg_it_->second;
		Link* base_link = leg->getLinkByIDName(leg->getIDName() + "_base_link");
		double x_position = base_link->length*cos(base_link->angle); // First joint world position (x)
		double y_position = base_link->length*sin(base_link->angle); // First joint world position (y)
		x_position += (min_horizontal_range - workspace_radius_)*cos(leg->getStanceLegYaw()); 
		y_position += (min_horizontal_range - workspace_radius_)*sin(leg->getStanceLegYaw());
		Vector3d identity_tip_position(x_position, y_position, -body_clearance_);
		
		double distance_to_previous_tip_position = (identity_tip_position - previous_identity_position).norm();
		previous_identity_position = identity_tip_position;
		if (2*workspace_radius_ > distance_to_previous_tip_position)
		{
			workspace_radius_ = distance_to_previous_tip_position/2.0; //Workspace radius ammended to prevent overlapping
			leg_it_ = model_->getLegContainer()->begin(); //Restart leg identity tip position calculation with new radius
			previous_identity_position = Vector3d(0,0,0);
		}
		else
		{
			leg->setLegStepper(new LegStepper(this, leg, identity_tip_position));
			leg_it_++;
		}
	}

	// Stance radius based around front right leg to ensure positive values
	Leg* reference_leg = model_->getLegByIDNumber(0);
	LegStepper* reference_leg_stepper = reference_leg->getLegStepper();
	double x_position = reference_leg_stepper->getDefaultTipPosition()[0];
	double y_position = reference_leg_stepper->getDefaultTipPosition()[1];
	stance_radius_ = Vector2d(x_position, y_position).norm();

	desired_linear_velocity_ = Vector2d(0, 0);
	desired_angular_velocity_ = 0;

	setGaitParams(params_);
}

/***********************************************************************************************************************
 * Sets parameters associated with gait
***********************************************************************************************************************/
void WalkController::setGaitParams(Parameters* p)
{
  params_ = p;
  stance_end_ = params_->stance_phase.data * 0.5;
  swing_start_ = stance_end_;
  swing_end_ = swing_start_ + params_->swing_phase.data;
  stance_start_ = swing_end_;

  // Normalises the step phase length to match the total number of iterations over a full step
  int base_phase_length = params_->stance_phase.data + params_->swing_phase.data;
  double swing_ratio = double(params_->swing_phase.data) / double(base_phase_length);  // Used to modify stepFreqency based on gait
	
	// Ensure phase length is even and divisible by base phase length and therefore gives whole even normaliser value
	double raw_phase_length = ((1.0 / params_->step_frequency.current_value) / time_delta_) / swing_ratio;
	phase_length_ = roundToEvenInt(raw_phase_length/base_phase_length)*base_phase_length;
	ROS_DEBUG_COND(false, "For requested step frequency (%f), phase length calculated as %f and rounded to %d\n",
	               params_->step_frequency.current_value, raw_phase_length, phase_length_);

  step_frequency_ = 1 / (phase_length_ * time_delta_);  // adjust stepFrequency to match corrected phaseLength
  int normaliser = phase_length_ / base_phase_length;
  stance_end_ *= normaliser;
  swing_start_ *= normaliser;
  swing_end_ *= normaliser;
  stance_start_ *= normaliser;
  int base_phase_offset = int(params_->phase_offset.data * normaliser);
	
  stance_length_ = mod(stance_end_ - stance_start_, phase_length_);
  swing_length_ = swing_end_ - swing_start_;	
	
  ROS_ASSERT(stance_length_ % 2 == 0);
  ROS_ASSERT(swing_length_ % 2 == 0);
	
  max_forced_stance_length_ = 0;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    int index = leg->getIDNumber();
    int multiplier = params_->offset_multiplier.data[index];
    LegStepper* leg_stepper = leg->getLegStepper();
    int phase_offset = (base_phase_offset * multiplier) % phase_length_;
    leg_stepper->setPhaseOffset(phase_offset);
    // Check if leg starts in swing phase (i.e. forced to stance for the 1st step cycle) 
    // and finds this max 'forced stance' phase length used in acceleration calculations
    if (phase_offset > swing_start_ && phase_offset < swing_end_)  // SWING STATE
    {
      max_forced_stance_length_ = max(max_forced_stance_length_, swing_end_ - phase_offset);
    }     
  }
  
  double on_ground_ratio = double(stance_length_) / double(phase_length_);
  double time_to_max_stride = (max_forced_stance_length_ + stance_length_ + swing_length_)*time_delta_; 
  double max_acceleration = (workspace_radius_*2.0) / ((on_ground_ratio/step_frequency_)*time_to_max_stride);
  double max_speed = (workspace_radius_*2.0) * step_frequency_ / on_ground_ratio;   
  
  // Calculates max overshoot of tip (in stance phase) outside workspace
  double overshoot = 0;
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper();
    // All referenced swings are the LAST swing phase BEFORE the max velocity (stride length) is reached
    double phase_offset = leg_stepper->getPhaseOffset();    
    double time_to_swing_end = time_to_max_stride - phase_offset*time_delta_;
    double stride_length = max_acceleration * ((on_ground_ratio/step_frequency_)*time_to_swing_end);
    double d0 = -stride_length/2.0; // Distance from default tip position at time of swing end
    double v0 = max_acceleration * time_to_swing_end; // Tip velocity at time of swing end
    double t = phase_offset*time_delta_; // Time between swing end and max velocity being reached
    double d1 = d0 + v0*t + 0.5*max_acceleration*sqr(t); // Distance from default tip position when max velocity is reached
    double d2 = max_speed*(stance_length_*time_delta_ - t); // Distance from default tip position at end of current stance phase
    overshoot = max(overshoot, d1 + d2 - workspace_radius_); // Max overshoot distance past workspace limitations
  }
  
  // Scale workspace to accomodate max stance overshoot and calculate max stride length to accomodate known swing overshoot
  double scaled_workspace = (workspace_radius_/(workspace_radius_ + overshoot))*workspace_radius_*2.0;
  double bezier_scaler = swing_length_ / (2.0*stance_length_);
  double max_stride_length = scaled_workspace * (1.0 / (1.0 + 0.5*bezier_scaler)); // Based on trajectory engine
  
  // Distance: max_stride_length_, Time: on_ground_ratio*(1/step_frequency_) where step frequency is FULL step cycles/s)
  max_linear_speed_ = max_stride_length / (on_ground_ratio/step_frequency_);
  max_linear_acceleration_ = max_stride_length / ((on_ground_ratio/step_frequency_)*time_to_max_stride);
  max_angular_speed_ = max_linear_speed_ / stance_radius_;  
  max_angular_acceleration_ = max_acceleration/stance_radius_;
  
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/
void WalkController::updateWalk(Vector2d linear_velocity_input, double angular_velocity_input)
{
  std::map<int, Leg*>::iterator leg_it;
  
  double on_ground_ratio = double(stance_length_) / double(phase_length_);

  Vector2d new_linear_velocity;
  double new_angular_velocity;

  // Get new angular/linear velocities according to input mode
  if (walk_state_ != STOPPING)
  {
    if (params_->velocity_input_mode.data == "throttle")
    {
      new_linear_velocity = clamped(linear_velocity_input, 1.0) * max_linear_speed_;  // Forces input between -1.0/1.0
      new_angular_velocity = clamped(angular_velocity_input, -1.0, 1.0) * max_angular_speed_;

      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (1 - abs(angular_velocity_input)); 
    }
    else if (params_->velocity_input_mode.data == "real")
    {
      new_linear_velocity = clamped(linear_velocity_input, max_linear_speed_);
      new_angular_velocity = clamped(angular_velocity_input, -max_angular_speed_, max_angular_speed_);

      // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
      new_linear_velocity *= (max_angular_speed_ != 0 ? (1 - abs(new_angular_velocity / max_angular_speed_)) : 0.0);

      if (linear_velocity_input.norm() > max_linear_speed_)
      {
        ROS_WARN_THROTTLE(10, "Input linear speed (%f) exceeds maximum linear speed (%f) and has been clamped.",
                          linear_velocity_input.norm(), max_linear_speed_);
      }
      if (abs(angular_velocity_input) > max_angular_speed_)
      {
        ROS_WARN_THROTTLE(10, "Input angular velocity (%f) exceeds maximum angular speed (%f) and has been clamped.",
                          abs(angular_velocity_input), max_angular_speed_);
      }
    }
  }
  else
  {
    new_linear_velocity = Vector2d(0.0, 0.0);
  }
  
  // Linear Acceleration Control
  Vector2d linear_acceleration = new_linear_velocity - desired_linear_velocity_;
  // Calculate max acceleration if specified by parameter

  // Update linear velocity according to acceleration limits
  if (linear_acceleration.norm() > 0.0)
  {
    desired_linear_velocity_ +=
        linear_acceleration * min(1.0, max_linear_acceleration_  * time_delta_ / linear_acceleration.norm());
  }

  // Angular Acceleration Control
  double angular_acceleration = new_angular_velocity - desired_angular_velocity_;

  // Update angular velocity according to acceleration limits
  if (abs(angular_acceleration) > 0.0)
  {
    desired_angular_velocity_ +=
        angular_acceleration * min(1.0, max_angular_acceleration_ * time_delta_ / abs(angular_acceleration));
  }  

  bool has_velocity_command = linear_velocity_input.norm() || angular_velocity_input;

  // Check that all legs are in WALKING state
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    if (has_velocity_command && leg->getLegState() != WALKING)
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
  int num_legs = model_->getLegCount();
  if (walk_state_ == STOPPED && has_velocity_command && pose_state_ == POSING_COMPLETE)
  {
    walk_state_ = STARTING;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg* leg = leg_it_->second;
      LegStepper* leg_stepper = leg->getLegStepper();
      leg_stepper->setPhase(leg_stepper->getPhaseOffset() - 1);
			leg_stepper->setAtCorrectPhase(false);
			leg_stepper->setCompletedFirstStep(false);
			leg_stepper->setStepState(STANCE);
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
	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		Leg* leg = leg_it_->second;
		LegStepper* leg_stepper = leg->getLegStepper();    

		if (walk_state_ == STARTING)
		{
			leg_stepper->iteratePhase();

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
			leg_stepper->iteratePhase();
			
			// All legs (except reference leg) must make one extra step after receiving stopping signal
			bool zero_body_velocity = leg_stepper->getStrideVector().norm() == 0;
			if (zero_body_velocity && !leg_stepper->isAtCorrectPhase() && leg_stepper->getPhase() == swing_end_)
			{
				leg_stepper->setStepState(FORCE_STOP);
				if (!leg_stepper->isAtCorrectPhase())
				{
					leg_stepper->setAtCorrectPhase(true);
					legs_at_correct_phase_++;
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
			leg_stepper->iteratePhase();
			leg_stepper->setStepState(FORCE_STOP);
		}
	}
  
  // Step State Machine
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper(); 
    int phase = leg_stepper->getPhase();
    StepState step_state = leg_stepper->getStepState();

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
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper(); 
        
    Vector3d tip_position = leg->getLocalTipPosition(); //TBD Should be walker tip positions?
    Vector2d rotation_normal = Vector2d(-tip_position[1], tip_position[0]);
    Vector2d stride_vector = desired_linear_velocity_ + desired_angular_velocity_ * rotation_normal;
    stride_vector *= (on_ground_ratio / step_frequency_);
    stride_length_ = stride_vector.norm();
    leg_stepper->updateStride(stride_vector); 

    if (leg->getLegState() == WALKING)
    {
      if (walk_state_ != STOPPED)
      {
	leg_stepper->updatePosition();  // updates current tip position through step cycle
      }
    }
  }
}

/***********************************************************************************************************************
 * Calculates body and stride velocities and uses velocities in body and leg state machines
 * to update tip positions and apply inverse kinematics
***********************************************************************************************************************/

void WalkController::updateManual(int primary_leg_selection_ID, Vector3d primary_tip_velocity_input,
                                  int secondary_leg_selection_ID, Vector3d secondary_tip_velocity_input)
{
	for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
	{
		Leg* leg = leg_it_->second;
		LegStepper* leg_stepper = leg->getLegStepper();  
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

			if (params_->leg_manipulation_mode.data == "joint_control")  // HACK Below
			{
				double coxa_joint_velocity = tip_velocity_input[0] * params_->max_rotation_velocity.data * time_delta_;
				double femur_joint_velocity = tip_velocity_input[1] * params_->max_rotation_velocity.data * time_delta_;
				double tibia_joint_velocity = tip_velocity_input[2] * params_->max_rotation_velocity.data * time_delta_;
				leg->getJointByIDName(leg->getIDName() + "_coxa_joint")->desired_position += coxa_joint_velocity;
				leg->getJointByIDName(leg->getIDName() + "_femur_joint")->desired_position += femur_joint_velocity;
				leg->getJointByIDName(leg->getIDName() + "_tibia_joint")->desired_position += tibia_joint_velocity;	
				Vector3d new_tip_position = leg->applyFK(false);
				leg_stepper->setCurrentTipPosition(new_tip_position);
			}
			else if (params_->leg_manipulation_mode.data == "tip_control")
			{
				Vector3d tip_position_change = tip_velocity_input * params_->max_translation_velocity.data * time_delta_;
				Vector3d new_tip_position = leg_stepper->getCurrentTipPosition() + tip_position_change;
				leg_stepper->setCurrentTipPosition(new_tip_position);
			}
		}
	}
}

/***********************************************************************************************************************
 * Leg stepper object constructor
***********************************************************************************************************************/
LegStepper::LegStepper(WalkController* walker, Leg* leg, Vector3d identity_tip_position) 
  : walker_(walker)
  , leg_(leg)
  , default_tip_position_(identity_tip_position)
  , current_tip_position_(default_tip_position_)
{
  stride_vector_ = Vector3d(0.0,0.0,0.0);
  swing_height_ = walker->getStepClearance();
  stance_depth_ = walker->getStepDepth();
  
  for (int i = 0; i < 5; ++i)
  {
    swing_1_nodes_[i] = Vector3d(0,0,0);
    swing_2_nodes_[i] = Vector3d(0,0,0);
    stance_nodes_[i] = Vector3d(0,0,0);
  }
};

/***********************************************************************************************************************
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
  double step_frequency = walker_->getStepFrequency();
  double time_delta = walker_->getTimeDelta(); 
  
  // Swing Phase
  if (step_state_ == SWING)
  {
    int phase_length = walker_->getPhaseLength();
    int swing_start = walker_->getSwingStart();
    int swing_end = walker_->getSwingEnd();		
    int swing_length = swing_end - swing_start;
    int stance_start = walker_->getStanceStart();
    int stance_end = walker_->getStanceEnd();  
    int stance_length = mod(stance_end - stance_start, phase_length);

    swing_delta_t_ = 2.0 / (roundToInt((double(swing_length) / phase_length) / (step_frequency * time_delta) / 2.0) * 2.0);

    // Used to scale the distance between control nodes for the stance/swing curves which have differing delta time values
    double bezier_scaler = swing_length / (2.0*stance_length);
    
    int iteration = phase_ - swing_start + 1;
    int num_iterations = 2.0 / swing_delta_t_;

    // Save initial tip position at beginning of swing
    if (iteration == 1)
    {
      swing_origin_tip_position_ = current_tip_position_;
      generateSwingControlNodes(bezier_scaler, stride_vector_);
    }

    // Calculate change in position using 1st/2nd bezier curve (depending on 1st/2nd half of swing)
    Vector3d delta_pos;
    double t1 = 0;
    double t2 = 0;

    if (iteration <= num_iterations / 2)
    {
      t1 = iteration * swing_delta_t_;
      //delta_pos = swing_delta_t_ * quarticBezierDot(swing_1_nodes_, t1);
      generateSwingControlNodes(bezier_scaler, stride_vector_);
      current_tip_position_ = quarticBezier(swing_1_nodes_, t1);
    }
    else
    {
      stance_origin_tip_position_ = default_tip_position_ + 0.5 * stride_vector_;
      generateStanceControlNodes(stride_vector_);
      generateSwingControlNodes(bezier_scaler, stride_vector_);
      t2 = (iteration - num_iterations / 2) * swing_delta_t_;
      //delta_pos = swing_delta_t_ * quarticBezierDot(swing_2_nodes_, t2);
      current_tip_position_ = quarticBezier(swing_2_nodes_, t2);
    }

    //current_tip_position_ += delta_pos;
    //current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

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
      ROS_DEBUG_COND(walker_->getParameters()->debug_swing_trajectory.data, 
		     "SWING TRAJECTORY_DEBUG - ITERATION: %d\t\t"
		     "TIME: %f:%f\t\t"
		     "ORIGIN: %f:%f:%f\t\t"
		     "POS: %f:%f:%f\t\t"
		     "TARGET: %f:%f:%f\n", 
		     iteration, t1, t2, 
		     swing_origin_tip_position_[0], swing_origin_tip_position_[1], swing_origin_tip_position_[2], 
		     current_tip_position_[0], current_tip_position_[1], current_tip_position_[2], 
		     swing_2_nodes_[4][0], swing_2_nodes_[4][1], swing_2_nodes_[4][2]);
    }      
  }
  // Stance phase
  else if (step_state_ == STANCE)
  {
    int phase_length = walker_->getPhaseLength();
    int stance_start = completed_first_step_ ? walker_->getStanceStart() : phase_offset_;
    int stance_end = walker_->getStanceEnd();  
    int stance_length = mod(stance_end - stance_start, phase_length);
    
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
    double stride_scaler = double(stance_length) / (mod(stance_end - walker_->getStanceStart(), phase_length));
    Vector3d scaled_strid_vector = stride_vector_ * stride_scaler;
    generateStanceControlNodes(scaled_strid_vector);
    stance_delta_t_ = 1.0 / (roundToInt((double(stance_length) / phase_length) / (step_frequency * time_delta) / 2.0) * 2.0);
    
    t = iteration * stance_delta_t_;
    //current_tip_position_ = quarticBezier(stance_nodes_, t);
    delta_pos = stance_delta_t_ * quarticBezierDot(stance_nodes_, t);

	current_tip_position_ += delta_pos;
	current_tip_velocity_ = delta_pos / walker_->getTimeDelta();

    if (t < stance_delta_t_)
    {
      t = 0.0;
    }
    
    if (leg_->getIDNumber() == 1)
    {
      ROS_DEBUG_COND(walker_->getParameters()->debug_stance_trajectory.data,
		     "STANCE TRAJECTORY_DEBUG - ITERATION: %d\t\t"
		     "TIME: %f\t\t"
		     "ORIGIN: %f:%f:%f\t\t"
		     "POS: %f:%f:%f\t\t"
		     "TARGET: %f:%f:%f\n", 
		     iteration, t, 
		     stance_origin_tip_position_[0], stance_origin_tip_position_[1], stance_origin_tip_position_[2], 
		     current_tip_position_[0], current_tip_position_[1], current_tip_position_[2], 
		     stance_nodes_[4][0], stance_nodes_[4][1], stance_nodes_[4][2]);      
    }
  }
}

/***********************************************************************************************************************
 * Generates control nodes for each quartic bezier curve of swing tip trajectory calculation
***********************************************************************************************************************/
void LegStepper::generateSwingControlNodes(double bezier_scaler, Vector3d stride_vector)
{
  double scaler = leg_->getIDNumber() < 3 ? 0.0:-0.0;
  Vector3d curver(scaler*stride_vector[1], -scaler*stride_vector[0], 0); //TBD Parameterise
  
  
  // Control nodes for swing quartic bezier curves - horizontal plane
  // Set for horizontal position continuity at transition between stance and primary swing curves (C0 Smoothness)
  swing_1_nodes_[0] = swing_origin_tip_position_;  
  // Set for horizontal velocity continuity at transition between stance and primary swing curves (C1 Smoothness)
  swing_1_nodes_[1] = swing_1_nodes_[0] + bezier_scaler * (stance_nodes_[4] - stance_nodes_[3]);
  // Set for horizontal acceleration continuity at transition between stance and primary swing curves (C2 Smoothness)
  swing_1_nodes_[2] = swing_1_nodes_[1] + (swing_1_nodes_[1] - swing_1_nodes_[0]);  
  // Set to default tip position so max swing height and transition to 2nd swing curve occurs at default tip position
  swing_1_nodes_[4] = default_tip_position_ + curver;
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
