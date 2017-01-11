#include "simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * State controller contructor
***********************************************************************************************************************/
StateController::StateController(ros::NodeHandle n) : n_(n)
{
  // Get parameters from parameter server and initialises parameter map
  getParameters();

  // Initiate model and imu objects
  model_ = new Model(params_);
  
  // Hexapod Remote topic subscriptions
  n.subscribe("hexapod_remote/desired_velocity", 1, &bodyVelocityInputCallback);
  n.subscribe("hexapod_remote/primary_tip_velocity", 1, &primaryTipVelocityInputCallback);
  n.subscribe("hexapod_remote/secondary_tip_velocity", 1, &secondaryTipVelocityInputCallback);
  n.subscribe("hexapod_remote/desired_pose", 1, &bodyPoseInputCallback);
  n.subscribe("hexapod_remote/system_state", 1, &systemStateCallback);
  n.subscribe("hexapod_remote/gait_selection", 1, &gaitSelectionCallback);
  n.subscribe("hexapod_remote/posing_mode", 1, &posingModeCallback);
  n.subscribe("hexapod_remote/cruise_control_mode", 1, &cruiseControlCallback);
  n.subscribe("hexapod_remote/auto_navigation_mode", 1, &autoNavigationCallback);
  n.subscribe("hexapod_remote/parameter_selection", 1, &parameterSelectionCallback);
  n.subscribe("hexapod_remote/parameter_adjustment", 1, &parameterAdjustCallback);
  n.subscribe("hexapod_remote/primary_leg_selection", 1, &primaryLegSelectionCallback);
  n.subscribe("hexapod_remote/primary_leg_state", 1, &primaryLegStateCallback);
  n.subscribe("hexapod_remote/secondary_leg_selection", 1, &secondaryLegSelectionCallback);
  n.subscribe("hexapod_remote/secondary_leg_state", 1, &secondaryLegStateCallback);
  n.subscribe("hexapod_remote/pose_reset_mode", 1, &poseResetCallback);

  // Motor and other sensor topic subscriptions
  imu_data_subscriber_ = n.subscribe("ig/imu/data_ned", 1, &imuCallback);
  tip_force_subscriber_ = n.subscribe("/motor_encoders", 1, &tipForceCallback);
  joint_state_subscriber_1_ = n.subscribe("/hexapod_joint_state", 1000, &jointStatesCallback);
  joint_state_subscriber_2_ = n.subscribe("/hexapod/joint_states", 1000, &jointStatesCallback);
  
  //Set up debugging publishers  
  pose_publisher_ = n.advertise<geometry_msgs::Twist>("/hexapod/pose", 1000);
  imu_data_publisher_ = n.advertise<std_msgs::Float32MultiArray>("/hexapod/imu_data", 1000);
  body_velocity_publisher_ = n.advertise<std_msgs::Float32MultiArray>("/hexapod/body_velocity", 1000);
  rotation_pose_error_publisher_ = n.advertise<std_msgs::Float32MultiArray>("/hexapod/rotation_pose_error", 1000);
  translation_pose_error_publisher_ = n.advertise<std_msgs::Float32MultiArray>("/hexapod/translation_pose_error", 1000);
  z_tip_error_publisher_ = n.advertise<std_msgs::Float32MultiArray>("/hexapod/z_tip_error", 1000);
  
  // Set up leg state publishers within leg objects
  std::map<std::string, Leg*>::iterator leg_it;
  for (leg_it = model_->getLegContainer().begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
  {
    Leg3DOF* leg = leg_it->second();
    std::string leg_name = leg->getIDName();
    leg->setStatePublisher(n.advertise<simple_hexapod_controller::legState>("/hexapod/"+leg_name+"/state", 1000));
    leg->setASCStatePublisher(n.advertise<std_msgs::Bool>("/leg_state_"+leg_name+"_bool", 1));
  }
  
  // Populate joint position arrays
  for (int i = 0; i < 18; i++)
  {
    joint_positions_[i] = 1e10;  // Excessive to make obvious when real values are read from subscribed topic
    jointVelocities[i] = 0.0;
    jointEfforts[i] = 0.0;
    if (i < 6)
    {
      tipForces[i] = 0;
    }
  }
  
  use_tip_force_ = (params_.impedance_input == "tip_force");
  use_joint_effort_ = (params_.impedance_input == "joint_effort");
}

/***********************************************************************************************************************
 * State controller destructor
***********************************************************************************************************************/
StateController::~StateController()
{
  delete model_;
  delete walker_;
  delete poser_;
  delete impedance_;
}

/***********************************************************************************************************************
 * Init state controller
***********************************************************************************************************************/
void StateController::init()
{
  // Setup motor interface
  if (params_.use_dynamixel_pro_interface)
  {
    interface_ = new DynamixelProMotorInterface();
  }
  else
  {
    interface_ = new DynamixelMotorInterface();
  }

  interface_->setupSpeed(params_.interface_setup_speed);

  // Set initial gait selection number for gait toggling
  if (params_.gait_type == "tripod_gait")
  {
    gait_selection_ = TRIPOD_GAIT;
  }
  else if (params_.gait_type == "ripple_gait")
  {
    gait_selection_ = RIPPLE_GAIT;
  }
  else if (params_.gait_type == "wave_gait")
  {
    gait_selection_ = WAVE_GAIT;
  }
  else if (params_.gait_type == "amble_gait")
  {
    gait_selection_ = AMBLE_GAIT;
  }

  // Create controller objects
  walker_ = new WalkController(model_, &params_);
  poser_ = new PoseController(model_, &params_);
  impedance_ = new ImpedanceController(&params_);

  linear_velocity_input_ = Vector2d(0.0, 0.0);
}

/***********************************************************************************************************************
 * Sets joint values in model according to aquired joint positions (set to default values if required)
***********************************************************************************************************************/
void StateController::setJointPositions(bool use_defaults)
{
  if (use_defaults)
  {
    params_.start_up_sequence = false;
  }
  else
  {
    // Set initial leg angles
    int index = 0;
    typedef std::map<std::string, Leg*>::iterator it_type;
    for (it_type it = model_->getLegContainer().begin(); it != model_->getLegContainer()->end(); ++it)
    {
      Leg3DOF* leg = it->second();
      double dir = leg->getMirrorDir();
      leg->init(joint_positions_[index + 0] + dir * params_.physical_coxa_angle_offset[leg],
		-joint_positions_[index + 1], joint_positions_[index + 2] + dir * params_.physical_tibia_angle_offset);
      }
      index += num_joints;
    }    
  }  
}

/***********************************************************************************************************************
 * State machine loop
***********************************************************************************************************************/
void StateController::loop()
{
  // Compensation - updates currentPose for body compensation
  updatePose();

  // Impedance control - updates deltaZ values
  if (params_.impedance_control)
  {
    impedanceControl();
  }

  // Hexapod state machine
  if (transitionSystemStateFlag)
  {
    transitionSystemState();
  }
  else if (system_state_ == RUNNING)
  {
    runningState();
  }
  model_->clampToLimits();
}

/***********************************************************************************************************************
 * State transition handler
***********************************************************************************************************************/
void StateController::transitionSystemState()
{
  // UNKNOWN -> OFF/PACKED/READY/RUNNING  if (systemState == UNKNOWN)
  if (system_state_ == UNKNOWN)
  {
    int checkPacked = 0;
    typedef std::map<std::string, Leg*>::iterator it_type;
    for (it_type it = model_->getLegContainer().begin(); it != model_->getLegContainer()->end(); ++it)
    {
      Leg3DOF leg = it->second();
      
      // Check all current joint positions are 'close' to packed joint positions
      double jointTolerance = 0.2;
      int index = leg.getIDNumber();
      checkPacked += int(abs(leg.coxa_joint_position_ - packedJointPositions[index][0]) < jointTolerance &&
			 abs(leg.femur_joint_position_ - packedJointPositions[index][1]) < jointTolerance &&
			 abs(leg.tibia_joint_position_ - packedJointPositions[index][2]) < jointTolerance);
    }
    if (checkPacked == 6)  // All joints in each leg are approximately in the packed position
    {
      if (!params_.start_up_sequence)
      {
        ROS_FATAL("Hexapod currently in packed state and cannot run direct startup sequence.\nEither manually unpack "
                  "hexapod or set start_up_sequence to true in config file\n");
        ros::shutdown();
      }
      else
      {
        system_state_ = PACKED;
        ROS_INFO("Hexapod currently packed.\n");
      }
    }
    else if (!params_.start_up_sequence)
    {
      ROS_WARN("start_up_sequence parameter is set to false, ensure hexapod is off the ground before transitioning "
               "system state.\n");
      system_state_ = OFF;
    }
    else
    {
      // double timeOut = 30;
      system_state_ = PACKED;
      ROS_WARN("Hexapod state is unknown. Future state transitions may be undesireable, recommend ensuring hexapod is "
               "off the ground before proceeding.\n");
      // ROS_WARN("Suspending controller for %f seconds. Any desired state transitions will begin after this period.\n",
      // timeOut);
      // sleep(timeOut);
      // ROS_WARN("Controller resuming.\n");
    }
  }
  // OFF -> !OFF (Start controller or directly transition to walking stance)
  else if (system_state_ == OFF && new_system_state_ != OFF)
  {
    // OFF -> RUNNING (Direct startup)
    if (new_system_state_ == RUNNING && !params_.start_up_sequence)
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning directly to RUNNING state . . .\n");
      //walker->init(hexapod, params); TBD
      double res = poser_->directStartup(model_);
      if (res == 1.0)
      {
        system_state_ = RUNNING;
        ROS_INFO("Direct startup sequence complete. Ready to walk.\n");
      }
    }
    // OFF -> PACKED/READY/RUNNING (Start controller)
    else
    {
      system_state_ = PACKED;
      ROS_INFO("Controller running.\n");
    }
  }
  // PACKED -> OFF (Suspend controller)
  else if (system_state_ == PACKED && new_system_state_ == OFF)
  {
    system_state_ = OFF;
    ROS_INFO("Controller suspended.\n");
  }
  // PACKED -> READY/RUNNING (Unpack Hexapod)
  else if (system_state_ == PACKED && (new_system_state_ == READY || new_system_state_ == RUNNING))
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to READY state . . .\n");
    int completed_count = 0;    
    if (poser_->unpackLegs(2.0 / params_.step_frequency))
    {
      system_state_ = READY;
      ROS_INFO("State transition complete. Hexapod is in READY state.\n");
    }
  }
  // READY -> PACKED/OFF (Pack Hexapod)
  else if (system_state_ == READY && (new_system_state_ == PACKED || new_system_state_ == OFF))
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to PACKED state . . .\n");
    if (poser_->moveToJointPosition(packedJointPositions, 2.0 / params_.step_frequency))
    {
      system_state_ = PACKED;
      ROS_INFO("State transition complete. Hexapod is in PACKED state.\n");
    }
  }
  // READY -> RUNNING (Initate start up sequence to step to walking stance)
  else if (system_state_ == READY && new_system_state_ == RUNNING)
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to RUNNING state . . .\n");
    if (poser_->startUpSequence(walker_->identityTipPositions, poser_->getCurrentPose(), deltaZ, params_.move_legs_sequentially))
    {
      system_state_ = RUNNING;
      ROS_INFO("State transition complete. Hexapod is in RUNNING state. Ready to walk.\n");
    }
  }
  // RUNNING -> !RUNNING (Initiate shut down sequence to step from walking stance to ready stance or suspend controller)
  else if (system_state_ == RUNNING && new_system_state_ != RUNNING)
  {
    // RUNNING -> OFF (Suspend controller)
    if (new_system_state_ == OFF && !params_.start_up_sequence)
    {
      system_state_ = OFF;
      ROS_INFO("Controller suspended.\n");
    }
    else
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to READY state . . .\n");
      if (poser_->shutDownSequence(poser_->getTipPositions(), poser_->getCurrentPose(), deltaZ, params_.move_legs_sequentially))
      {
        system_state_ = READY;
        ROS_INFO("State transition complete. Hexapod is in READY state.\n");
      }
    }
  }
  // Undefined system transition
  else
  {
    ROS_FATAL("Undefined system state transition was requested! Shutting down controller!\n");
    ros::shutdown();
  }

  // Transition complete
  if (system_state_ == new_system_state_)
  {
    transitionSystemStateFlag = false;
  }
}

/***********************************************************************************************************************
 * Running state
***********************************************************************************************************************/
void StateController::runningState()
{
  // Cruise control (constant velocity input)
  if (cruise_control_mode_ == CRUISE_CONTROL_ON)
  {
    if (params_.force_cruise_velocity)
    {
      linear_velocity_input_ = params_.linear_cruise_velocity;
      angularVelocityInput = params_.angular_cruise_velocity;
    }
    else
    {
      linear_velocity_input_ = linearCruiseVelocity;
      angularVelocityInput = angularCruiseVelocity;
    }
  }

  // Switch gait and update walker parameters
  if (gait_change_flag_)
  {
    gaitChange();
  }
  // Dynamically adjust parameters and change stance if required
  else if (parameter_adjust_flag_)
  {
    parameterAdjust();
  }
  // Toggle state of leg and transition between states
  else if (toggle_primary_leg_state_ || toggle_secondary_leg_state_)
  {
    legStateToggle();
  }

  // Update tip positions unless hexapod is undergoing gait switch, parameter adjustment or leg state transition
  //(which all only occur once the hexapod has stopped walking)
  if (!((gait_change_flag_ || parameter_adjust_flag_ || toggle_primary_leg_state_ || toggle_secondary_leg_state_) &&
        walker_->walk_state_ == STOPPED))
  {
    
    // Update tip positions for walking legs
    walker_->updateWalk(linear_velocity_input_, angularVelocityInput, model_);

    // Update tip positions for manually controlled legs
    walker_->updateManual(primary_leg_selection_, primaryManualTipVelocity, secondary_leg_selection_,
                         secondaryManualTipVelocity, model_);

    // Pose controller takes current tip positions from walker and applies pose compensation
    poser_->updateStance(model_);

    // Model uses posed tip positions and adds deltaZ from impedance controller and applies inverse kinematics on each
    // leg
    model_->updateLocal();
  }
}

/***********************************************************************************************************************
 * Compensation
***********************************************************************************************************************/
void StateController::updatePose()
{
  Pose new_pose;
  
  // Manually set (joystick controlled) body compensation
  if (params_.manual_compensation)
  {
    new_pose = poser_->manualCompensation();
  }
  else
  {
    new_pose = Pose::identity();
  }

  // Compensation to align centre of gravity evenly between tip positions on incline
  if (params_.inclination_compensation)
  {
    new_pose.position_ += poser_->inclinationCompensation(walker_, imu_data_);
  }

  // Compensation to offset average deltaZ from impedance controller and keep body at specificied height
  if (params_.impedance_control)
  {
    new_pose.position_[2] += poser_->impedanceControllerCompensation(model_);
  }

  // Auto body compensation using IMU feedback
  if (params_.imu_compensation)
  {
    new_pose.rotation_ = poser_->imuCompensation(imu_data_);
  }
  // Automatic (non-feedback) body compensation
  else if (params_.auto_compensation)
  {
    Pose auto_pose = poser_->autoCompensation();
    new_pose.position_ += auto_pose.position_;
    new_pose.rotation_ *= auto_pose.rotation_;  //(Quaternion)
  }
  
  model_->setCurrentPose(new_pose);
}

/***********************************************************************************************************************
 * Impedance Control
***********************************************************************************************************************/
void StateController::impedanceControl()
{
  // Check how many legs are in stance phase
  int legsInStance = 0;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      if (walker_->legSteppers[l][s].step_state_ != SWING)
      {
        legsInStance++;
      }
    }
  }

  // Calculate new stiffness based on imu orientation or walking cycle
  bool useIMUForStiffness = false;  // Not fully tested
  if (params_.dynamic_stiffness && walker_->walk_state_ != STOPPED)
  {
    if (params_.imu_compensation && useIMUForStiffness)
    {
      impedance_->updateStiffness(poser_->current_pose_, walker_->identityTipPositions);  // TBD CHECK
    }
    else
    {
      impedance_->updateStiffness(walker_);
    }
  }

  // Get current force value on leg and run impedance calculations to get a vertical tip offset (deltaZ)
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      if (params_.dynamic_stiffness || legsInStance == 6)
      {
        double maxForce = 0;
        double minForce = 0;
        if (use_tip_force_)
        {
          double forceOffset = 1255.0;
          tipForce[l][s] = tipForces[2 * l + s] - forceOffset;
          maxForce = 1000.0;
          minForce = 0.0;
        }
        else if (use_joint_effort_)
        {
          int index = 6 * l + 3 * s + 1;
          int dir = (s == 0) ? -1 : 1;
          tipForce[l][s] = dir * (jointEfforts[index]);
          maxForce = 1e9;
          minForce = 0.0;
        }
        // Ensure force is within limits
        tipForce[l][s] = min(tipForce[l][s], maxForce);
        tipForce[l][s] = max(tipForce[l][s], minForce);
      }

      if (model_->legs_[l][s].leg_state_ == WALKING)
      {
        impedance_->updateImpedance(l, s, tipForce, deltaZ);
      }
    }
  }
}

/***********************************************************************************************************************
 * Dynamic Parameter adjustment
***********************************************************************************************************************/
void StateController::parameterAdjust()
{
  if (walker_->walk_state_ == STOPPED)
  {
    if (!new_parameter_set_)
    {
      switch (parameter_selection_)
      {
        case (NO_PARAMETER_SELECTION):
        {
          break;
        }
        case (STEP_FREQUENCY):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.step_frequency / default_params_.step_frequency;
          }
          params_.step_frequency = minMax(default_params_.step_frequency * param_scaler_, 0.1, 3.0);
          walker_->setGaitParams(params_);
          poser_->parameters = params_;
          param_value_ = params_.step_frequency;
          break;
        }
        case (STEP_CLEARANCE):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.step_clearance / default_params_.step_clearance;
          }
          params_.step_clearance = minMax(default_params_.step_clearance * param_scaler_, 0.01, 0.4);
          walker_->init(model_, params_);
          param_value_ = params_.step_clearance;
          break;
        }
        case (BODY_CLEARANCE):
        {
          if (default_params_.body_clearance == -1)
          {
            params_.body_clearance = walker_->body_clearance_;
            default_params_.body_clearance = params_.body_clearance;
          }
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.body_clearance / default_params_.body_clearance;
          }
          params_.body_clearance = minMax(default_params_.body_clearance * param_scaler_, 0.1, 0.99);
          walker_->init(model_, params_);
          param_value_ = params_.body_clearance;
          break;
        }
        case (LEG_SPAN_SCALE):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.leg_span_scale / default_params_.leg_span_scale;
          }
          params_.leg_span_scale = minMax(default_params_.leg_span_scale * param_scaler_, 0.1, 1.5);
          walker_->init(model_, params_);
          param_value_ = params_.leg_span_scale;
          break;
        }
        case (VIRTUAL_MASS):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.virtual_mass / default_params_.virtual_mass;
          }
          params_.virtual_mass = minMax(default_params_.virtual_mass * param_scaler_, 0, 500);
          impedance_->init(params_);
          param_value_ = params_.virtual_mass;
          break;
        }
        case (VIRTUAL_STIFFNESS):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.virtual_stiffness / default_params_.virtual_stiffness;
          }
          params_.virtual_stiffness = minMax(default_params_.virtual_stiffness * param_scaler_, 0, 500);
          impedance_->init(params_);
          param_value_ = params_.virtual_stiffness;
          break;
        }
        case (VIRTUAL_DAMPING):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.virtual_damping_ratio / default_params_.virtual_damping_ratio;
          }
          params_.virtual_damping_ratio = minMax(default_params_.virtual_damping_ratio * param_scaler_, 0, 2.0);
          impedance_->init(params_);
          param_value_ = params_.virtual_damping_ratio;
          break;
        }
        case (FORCE_GAIN):
        {
          if (param_scaler_ == -1)
          {
            param_scaler_ = params_.force_gain / default_params_.force_gain;
          }
          params_.force_gain = minMax(default_params_.force_gain * param_scaler_, 0, 2.0);
          impedance_->init(params_);
          param_value_ = params_.force_gain;
          break;
        }
        default:
        {
          param_value_ = -1;
          param_scaler_ = -1;
          ROS_WARN("Attempting to adjust unknown parameter.\n");
          break;
        }
      }
      new_parameter_set_ = true;
      ROS_INFO("Attempting to adjust '%s' parameter to %d%% of default (%f) . . .\n", paramString.c_str(),
               roundToInt(paramScaler * 100), paramVal);
    }
    else
    {
      // Update tip Positions for new parameter value
      bool complete = (poser_->stepToNewStance(model_) == 1.0);
      if (complete)
      {
	std::string param_string = parameter_name_map_[parameter_selection_];
        ROS_INFO("Parameter '%s' set to %d%% of default (%f).\n", params_string.c_str(), roundToInt(paramScaler * 100),
                 paramVal);
        parameter_adjust_flag_ = false;
        new_parameter_set_ = false;
      }
    }
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to adjust parameters . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angularVelocityInput = 0.0;
  }
}

/***********************************************************************************************************************
 * Gait change
***********************************************************************************************************************/
void StateController::gaitChange()
{
  if (walker_->walk_state_ == STOPPED)
  {
    getGaitParameters(gait_selection_);
    walker_->setGaitParams(params_);
    ROS_INFO("Now using %s mode.\n", params.gait_type.c_str());
    gait_change_flag_ = false;
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to change gait . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angularVelocityInput = 0.0;
  }
}

/***********************************************************************************************************************
 * Leg State Toggle
***********************************************************************************************************************/
void StateController::legStateToggle()
{
  if (walker_->walk_state_ == STOPPED)
  {
    // Choose primary or secondary leg state to transition
    Leg3DOF* transitioning_leg;
    if (toggle_primary_leg_state_)
    {
      transitioning_leg = model_->getLegByID(primary_leg_selection_);
    }
    else if (toggle_secondary_leg_state_)
    {
      transitioning_leg = model_->getLegByID(secondary_leg_selection_);
    }
    std::string leg_name = transitioning_leg->getIDName();

    // Calculate default pose for new loading pattern
    poser_->calculateDefaultPose();

    if (transitioning_leg->getLegState() == WALKING)
    {
      if (manual_leg_count_ < MAX_MANUAL_LEGS)
      {
        ROS_INFO_COND(leg.leg_state_ == WALKING, "%s leg transitioning to MANUAL state . . .\n",
                      leg_name.c_str());
        transitioning_leg->setLegState(WALKING_TO_MANUAL);
      }
      else
      {
        ROS_INFO("Only allowed to have %d legs manually manipulated at one time.\n", MAX_MANUAL_LEGS);
        toggle_primary_leg_state_ = false;
        toggle_secondary_leg_state_ = false;
      }
    }
    else if (transitioning_leg->getLegState() == MANUAL)
    {
      ROS_INFO_COND(leg.leg_state_ == MANUAL, "%s leg transitioning to WALKING state . . .\n",
                    leg_name.c_str());
      transitioning_leg->setLegState(MANUAL_TO_WALKING);
    }
    else if (transitioning_leg->getLegState() == WALKING_TO_MANUAL)
    {
      poser_->setPoseResetMode(IMMEDIATE_ALL_RESET);  // Set to ALL_RESET to force pose to new default pose
      //walker->tipPositions[l][s] = hexapod->local_tip_positions_[l][s];  // Override walker tip positions for manual
                                                                      // control
      //poser->tipPositions[l][s] = hexapod->local_tip_positions_[l][s];
      double res = poser_->poseForLegManipulation(model_);

      if (params_.dynamic_stiffness)
      {
        impedance_->updateStiffness(res, l, s);
      }

      if (res == 1.0)
      {
        transitioning_leg->setLegState(MANUAL);
        ROS_INFO("%s leg set to state: MANUAL.\n", leg_name.c_str());
        toggle_primary_leg_state_ = false;
        toggle_secondary_leg_state_ = false;
        poser_->setPoseResetMode(NO_RESET);
        manual_leg_count_++;
      }
    }
    else if (transitioning_leg->getLegState() == MANUAL_TO_WALKING)
    {
      poser_->setPoseResetMode(IMMEDIATE_ALL_RESET);  // Set to ALL_RESET to force pose to new default pose
      //walker->tipPositions[l][s] = walker->identityTipPositions[l][s];  // Return walker tip positions to default
      double res = poser_->poseForLegManipulation(model_);

      if (params_.dynamic_stiffness)
      {
        impedance_->updateStiffness(1.0 - res, l, s);
      }

      if (res == 1.0)
      {
        //walker->tipPositions[l][s] = walker->identityTipPositions[l][s];
        transitioning_leg->setLegState(WALKING);
        ROS_INFO("%s leg set to state: WALKING.\n", leg->getIDName().c_str());
        toggle_primary_leg_state_ = false;
        toggle_secondary_leg_state_ = false;
        poser_->setPoseResetMode(NO_RESET);
        manual_leg_count_--;
      }
    }
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to transition leg state . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angularVelocityInput = 0.0;
  }
}

/***********************************************************************************************************************
 * Publishes various state variables for each leg
***********************************************************************************************************************/
void StateController::publishLegState()
{
  simple_hexapod_controller::legState msg;
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      msg.header.stamp = ros::Time::now();

      msg.leg_name.data = legNameMap[2 * l + s].c_str();

      // Tip positions
      msg.local_tip_position.x = model_->local_tip_positions_[l][s][0];
      msg.local_tip_position.y = model_->local_tip_positions_[l][s][1];
      msg.local_tip_position.z = model_->local_tip_positions_[l][s][2];
      msg.poser_tip_positions.x = poser_->tipPositions[l][s][0];
      msg.poser_tip_positions.y = poser_->tipPositions[l][s][1];
      msg.poser_tip_positions.z = poser_->tipPositions[l][s][2];
      msg.walker_tip_positions.x = walker_->tipPositions[l][s][0];
      msg.walker_tip_positions.y = walker_->tipPositions[l][s][1];
      msg.walker_tip_positions.z = walker_->tipPositions[l][s][2];

      // Step progress
      msg.swing_progress.data = walker_->legSteppers[l][s].swing_progress_;
      msg.stance_progress.data = walker_->legSteppers[l][s].stance_progress_;

      // Impedance controller
      msg.tip_force.data = tipForce[l][s];
      msg.delta_z.data = deltaZ[l][s];
      msg.virtual_stiffness.data = impedance_->virtualStiffness[l][s];

      leg_state_publishers_[l][s].publish(msg);

      // Publish leg state (ASC)
      std_msgs::Bool msg;
      if (walker_->legSteppers[l][s].step_state_ == SWING ||
          (model_->legs_[l][s].leg_state_ != WALKING && model_->legs_[l][s].leg_state_ != MANUAL))
      {
        msg.data = true;
      }
      else
      {
        msg.data = false;
      }
      ascLegStatePublishers[l][s].publish(msg);
    }
  }
}

/***********************************************************************************************************************
 * Publishes body velocity for debugging
***********************************************************************************************************************/
void StateController::publishBodyVelocity()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(walker_->current_linear_velocity_[0]);
  msg.data.push_back(walker_->current_linear_velocity_[1]);
  msg.data.push_back(walker_->current_angular_velocity_);
  body_velocity_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current pose (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishPose()
{
  geometry_msgs::Twist msg;
  msg.linear.x = poser_->current_pose_.position_[0];
  msg.linear.y = poser_->current_pose_.position_[1];
  msg.linear.z = poser_->current_pose_.position_[2];
  msg.angular.x = poser_->current_pose_.rotation_.toEulerAngles()[0];
  msg.angular.y = poser_->current_pose_.rotation_.toEulerAngles()[1];
  msg.angular.z = poser_->current_pose_.rotation_.toEulerAngles()[2];
  pose_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current rotation as per the IMU (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishIMUData()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(imu_data_.orientation.toEulerAngles()[0]);
  msg.data.push_back(imu_data_.orientation.toEulerAngles()[1]);
  msg.data.push_back(imu_data_.orientation.toEulerAngles()[2]);
  msg.data.push_back(imu_data_.linear_acceleration[0]);
  msg.data.push_back(imu_data_.linear_acceleration[1]);
  msg.data.push_back(imu_data_.linear_acceleration[2]);
  msg.data.push_back(imu_data_.angular_velocity[0]);
  msg.data.push_back(imu_data_.angular_velocity[1]);
  msg.data.push_back(imu_data_.angular_velocity[2]);
  imu_data_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishRotationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser_->rotation_position_error[0]);
  msg.data.push_back(poser_->rotation_position_error[1]);
  msg.data.push_back(poser_->rotation_position_error[2]);
  msg.data.push_back(poser_->rotationAbsementError[0]);
  msg.data.push_back(poser_->rotationAbsementError[1]);
  msg.data.push_back(poser_->rotationAbsementError[2]);
  msg.data.push_back(poser_->rotation_velocity_error[0]);
  msg.data.push_back(poser_->rotation_velocity_error[1]);
  msg.data.push_back(poser_->rotation_velocity_error[2]);
  rotation_pose_error_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishTranslationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser_->translationPositionError[0]);
  msg.data.push_back(poser_->translationPositionError[1]);
  msg.data.push_back(poser_->translationPositionError[2]);
  msg.data.push_back(poser_->translationAbsementError[0]);
  msg.data.push_back(poser_->translationAbsementError[1]);
  msg.data.push_back(poser_->translationAbsementError[2]);
  msg.data.push_back(poser_->translationVelocityError[0]);
  msg.data.push_back(poser_->translationVelocityError[1]);
  msg.data.push_back(poser_->translationVelocityError[2]);
  translation_pose_error_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishZTipError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      msg.data.push_back(impedance_->zTipPositionError[l][s]);
      msg.data.push_back(impedance_->zTipAbsementError[l][s]);
      msg.data.push_back(impedance_->zTipVelocityError[l][s]);
    }
  }
  z_tip_error_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Draws robot in RVIZ for debugging
***********************************************************************************************************************/
void StateController::RVIZDebugging()
{
  for (int s = 0; s < 2; s++)
  {
    for (int l = 0; l < 3; l++)
    {
      debug_.tip_positions_.insert(debug_.tip_positions_.begin(),
                                walker_->pose_.transformVector(model_->legs_[l][s].local_tip_position_));
      debug_.static_tip_positions_.insert(debug_.static_tip_positions_.begin(), model_->legs_[l][s].local_tip_position_);
    }
  }

  debug_.drawRobot(model_->legs_[0][0].femur_offset_, model_->getJointPositions(walker_->pose_), Vector4d(1, 1, 1, 1));
  debug_.drawPoints(debug_.tip_positions_, Vector4d(1, 0, 0, 1));  // Actual Tip Trajectory Paths
  // debug.drawPoints(debug.staticTipPositions, Vector4d(1,0,0,1)); //Static Single Tip Trajectory command

  if (debug_.tip_positions_.size() > 2000)
  {
    debug_.tip_positions_.erase(debug_.tip_positions_.end() - 6, debug_.tip_positions_.end());
  }
  if (debug_.static_tip_positions_.size() >= 6 * (1 / (params_.time_delta.current_value * walker_->step_frequency_)))
  {
    debug_.static_tip_positions_.clear();
  }
}

/***********************************************************************************************************************
 * Clamps joint velocities and publishes angle and velocity data to motor interface
***********************************************************************************************************************/
void StateController::publishJointValues()
{
  for (int s = 0; s < 2; s++)
  {
    double dir = s == 0 ? -1 : 1;
    for (int l = 0; l < 3; l++)
    {
      double yaw = dir * (model_->legs_[l][s].coxa_angle_ - params_.physical_coxa_angle_offset[l]);
      double lift = dir * model_->legs_[l][s].femur_angle_;
      double knee = dir * model_->legs_[l][s].tibia_angle_ - dir * params_.physical_tibia_angle_offset;

      double yawVel = 0;
      double liftVel = 0;
      double kneeVel = 0;

      if (firstIteration >= 6)  // First Iteration of ALL legs
      {
        yawVel = (yaw - model_->legs_[l][s].old_coxa_angle_) / params_.time_delta.current_value;
        liftVel = (lift - model_->legs_[l][s].old_femur_angle) / params_.time_delta.current_value;
        kneeVel = (knee - model_->legs_[l][s].old_tibia_angle_) / params_.time_delta.current_value;

        if (abs(yawVel) > model_->joint_max_angular_speeds_[0])
        {
          ROS_WARN("%s_body_coxa joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   leg_name.c_str(), yawVel, sign(yawVel) * hexapod->joint_max_angular_speeds_[0]);

          yawVel = sign(yawVel) * model_->joint_max_angular_speeds_[0];
          yaw = model_->legs_[l][s].old_coxa_angle_ + yawVel * params_.time_delta.current_value;
        }
        if (abs(liftVel) > model_->joint_max_angular_speeds_[1])
        {
          ROS_WARN("%s_coxa_femour joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   leg_name.c_str(), liftVel, sign(liftVel) * hexapod->joint_max_angular_speeds_[1]);

          liftVel = sign(liftVel) * model_->joint_max_angular_speeds_[1];
          lift = model_->legs_[l][s].old_femur_angle + liftVel * params_.time_delta.current_value;
        }
        if (abs(kneeVel) > model_->joint_max_angular_speeds_[2])
        {
          ROS_WARN("%s_femour_tibia joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   leg_name.c_str(), kneeVel, sign(kneeVel) * hexapod->joint_max_angular_speeds_[2]);

          kneeVel = sign(kneeVel) * model_->joint_max_angular_speeds_[2];
          knee = model_->legs_[l][s].old_tibia_angle_ + kneeVel * params_.time_delta.current_value;
        }
      }
      else
      {
        firstIteration++;  // First Iteration of ALL legs
      }

      interface_->setTargetAngle(l, s, 0, yaw);
      interface_->setTargetAngle(l, s, 1, -lift);
      interface_->setTargetAngle(l, s, 2, knee);

      if (!params_.use_dynamixel_pro_interface)
      {
        interface_->setVelocity(l, s, 0, yawVel);  // Doesn't cooperate with with Large Hexapod Dynamixel Pro drivers
      }
      interface_->setVelocity(l, s, 1, -liftVel);
      interface_->setVelocity(l, s, 2, kneeVel);

      model_->legs_[l][s].old_coxa_angle_ = yaw;
      model_->legs_[l][s].old_femur_angle = lift;
      model_->legs_[l][s].old_tibia_angle_ = knee;
    }
  }
  interface_->publish();
}

/***********************************************************************************************************************
 * Input Body Velocity Topic Callback
***********************************************************************************************************************/
void StateController::bodyVelocityInputCallback(const geometry_msgs::Twist &input)
{
  linear_velocity_input_ = Vector2d(input.linear.x, input.linear.y);
  angularVelocityInput = input.angular.z;
}

/***********************************************************************************************************************
 * Input Primary Manual Tip Velocity Topic Callback
***********************************************************************************************************************/
void StateController::primaryTipVelocityInputCallback(const geometry_msgs::Point &input)
{
  primaryManualTipVelocity = Vector3d(input.x, input.y, input.z);
}

/***********************************************************************************************************************
 * Input Secondary Manual Tip Velocity Topic Callback
***********************************************************************************************************************/
void StateController::secondaryTipVelocityInputCallback(const geometry_msgs::Point &input)
{
  secondaryManualTipVelocity = Vector3d(input.x, input.y, input.z);
}

/***********************************************************************************************************************
 * Body Pose Topic Callback
***********************************************************************************************************************/
void StateController::bodyPoseInputCallback(const geometry_msgs::Twist &input)
{
  Vector3d rotation_input(input.angular.x, input.angular.y, input.angular.z);
  Vector3d translation_input(input.linear.x, input.linear.y, input.linear.z);
  poser_->setManualPoseInput(translation_input, rotation_input);
}

/***********************************************************************************************************************
 * System state callback handling desired system state from hexapod_remote
***********************************************************************************************************************/
void StateController::systemStateCallback(const std_msgs::Int8 &input)
{
  new_system_state_ = static_cast<SystemState>(int(input.data));
  // 
  if (system_state_ == WAITING_FOR_USER && new_system_state_ != OFF)
  {
    system_state_ = UNKNOWN;
  }  
  // If startUpSequence parameter is false then skip READY and PACKED states
  else if (!params_.start_up_sequence)
  {
    if (new_system_state_ == READY || new_system_state_ == PACKED)
    {
      new_system_state_ = OFF;
    }
  }
}

/***********************************************************************************************************************
 * Gait Selection Callback
***********************************************************************************************************************/
void StateController::gaitSelectionCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    GaitDesignation newGaitSelection = static_cast<GaitDesignation>(int(input.data));
    if (newGaitSelection != gait_selection_ && newGaitSelection != GAIT_UNDESIGNATED)
    {
      gait_selection_ = newGaitSelection;
      gait_change_flag_ = true;
    }
  }
}

/***********************************************************************************************************************
 * Posing Mode Callback
***********************************************************************************************************************/
void StateController::posingModeCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    PosingMode newPosingMode = static_cast<PosingMode>(int(input.data));
    if (newPosingMode != posing_mode_)
    {
      posing_mode_ = newPosingMode;
      switch (posing_mode_)
      {
        case (NO_POSING):
          ROS_INFO("Posing mode set to "
                   "NO_POSING"
                   ". Body will not respond to manual posing input (except for reset commands).\n");
          break;
        case (X_Y_POSING):
          ROS_INFO("Posing mode set to "
                   "X_Y_POSING"
                   ". Body will only respond to x/y translational manual posing input.\n");
          break;
        case (PITCH_ROLL_POSING):
          ROS_INFO("Posing mode set to "
                   "PITCH_ROLL_POSING"
                   ". Body will only respond to pitch/roll rotational manual posing input.\n");
          break;
        case (Z_YAW_POSING):
          ROS_INFO("Posing mode set to "
                   "Z_YAW_POSING"
                   ". Body will only respond to z translational and yaw rotational manual posing input.\n");
          break;
      }
    }
  }
}

/***********************************************************************************************************************
 * Cruise Control Callback
***********************************************************************************************************************/
void StateController::cruiseControlCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    CruiseControlMode newCruiseControlMode = static_cast<CruiseControlMode>(int(input.data));
    if (newCruiseControlMode != cruise_control_mode_)
    {
      cruise_control_mode_ = newCruiseControlMode;
      if (newCruiseControlMode == CRUISE_CONTROL_ON)
      {
        // Save current velocity input as cruise input
        linearCruiseVelocity = linear_velocity_input_;
        angularCruiseVelocity = angularVelocityInput;
        ROS_INFO("Cruise control ON - Input velocity set to constant: Linear(X:Y): %f:%f, Angular(Z): %f\n",
                 linearCruiseVelocity[0], linearCruiseVelocity[1], angularCruiseVelocity);
      }
      else if (newCruiseControlMode == CRUISE_CONTROL_OFF)
      {
        ROS_INFO("Cruise control OFF - Input velocity set by user.\n");
      }
    }
  }
}

/***********************************************************************************************************************
 * Auto Navigation Callback
***********************************************************************************************************************/
void StateController::autoNavigationCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    AutoNavigationMode newAutoNavigationMode = static_cast<AutoNavigationMode>(int(input.data));
    if (newAutoNavigationMode != auto_navigation_mode_)
    {
      auto_navigation_mode_ = newAutoNavigationMode;
      ROS_INFO_COND(autoNavigationMode == AUTO_NAVIGATION_ON, "Auto Navigation mode ON. User input is being "
                                                              "ignored.\n");
      ROS_INFO_COND(autoNavigationMode == AUTO_NAVIGATION_OFF, "Auto Navigation mode OFF. Control returned to user "
                                                               "input.\n");
    }
  }
}

/***********************************************************************************************************************
 * Parameter selection callback
***********************************************************************************************************************/
void StateController::parameterSelectionCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    ParameterSelection newParameterSelection = static_cast<ParameterSelection>(int(input.data));
    if (newParameterSelection != parameter_selection_)
    {
      param_scaler_ = -1;
      ROS_INFO("%s parameter currently selected.\n", getParameterName(new_parameter_selection).c_str());
      parameter_selection_ = newParameterSelection;
    }
  }
}

/***********************************************************************************************************************
 * Parameter adjustment callback
***********************************************************************************************************************/
void StateController::parameterAdjustCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    if (input.data != 0.0 && !parameter_adjust_flag_ && parameter_selection_ != NO_PARAMETER_SELECTION)
    {
      if (param_scaler_ != -1)
      {
        param_scaler_ += input.data / paramAdjustSensitivity;
        param_scaler_ = minMax(param_scaler_, 0.1, 3.0);  // Parameter scaler ranges from 10%->300%
      }
      parameter_adjust_flag_ = true;
    }
  }
}

/***********************************************************************************************************************
 * Pose reset mode callback
***********************************************************************************************************************/
void StateController::poseResetCallback(const std_msgs::Int8 &input)
{
  if (poser_->getPoseResetMode() != IMMEDIATE_ALL_RESET)
  {
    poser_->setPoseResetMode(static_cast<PoseResetMode>(input.data));
  }
}

/***********************************************************************************************************************
 * Actuating Leg Primary Selection Callback
***********************************************************************************************************************/
void StateController::primaryLegSelectionCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    LegDesignation newPrimaryLegSelection = static_cast<LegDesignation>(input.data);
    if (primary_leg_selection_ != newPrimaryLegSelection)
    {
      if (newPrimaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("No leg currently selected for primary control.\n");
      }
      else
      {
        ROS_INFO("%s leg selected for primary control.\n", legNameMap[int(input.data)].c_str());
      }
      primary_leg_selection_ = newPrimaryLegSelection;
    }
  }
}

/***********************************************************************************************************************
 * Actuating Leg Secondary Selection Callback
***********************************************************************************************************************/
void StateController::secondaryLegSelectionCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    LegDesignation newSecondaryLegSelection = static_cast<LegDesignation>(input.data);
    if (secondary_leg_selection_ != newSecondaryLegSelection)
    {
      if (newSecondaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("No leg currently selected for secondary control.\n");
      }
      else
      {
        ROS_INFO("%s leg selected for secondary control.\n", legNameMap[int(input.data)].c_str());
      }
      secondary_leg_selection_ = newSecondaryLegSelection;
    }
  }
}

/***********************************************************************************************************************
 * Toggle Primary Leg State Callback
***********************************************************************************************************************/
void StateController::primaryLegStateCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    LegState newPrimaryLegState = static_cast<LegState>(int(input.data));
    if (newPrimaryLegState != primary_leg_state_)
    {
      if (primary_leg_selection_ == LEG_UNDESIGNATED)
      {
        ROS_INFO("Cannot toggle primary leg state as no leg is currently selected as primary.");
        ROS_INFO("Press left bumper to select a leg and try again.\n");
      }
      else if (toggle_secondary_leg_state_)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Cannot toggle primary leg state as secondary leg is currently "
                                           "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Please wait and try again.\n");
      }
      else
      {
        primary_leg_state_ = newPrimaryLegState;
        toggle_primary_leg_state_ = true;
      }
    }
  }
}

/***********************************************************************************************************************
 * Toggle Secondary Leg State Callback
***********************************************************************************************************************/
void StateController::secondaryLegStateCallback(const std_msgs::Int8 &input)
{
  if (system_state_ == RUNNING)
  {
    LegState newSecondaryLegState = static_cast<LegState>(int(input.data));
    if (newSecondaryLegState != secondary_leg_state_)
    {
      if (secondary_leg_selection_ == LEG_UNDESIGNATED)
      {
        ROS_INFO("Cannot toggle secondary leg state as no leg is currently selected as secondary.");
        ROS_INFO("Press left bumper to select a leg and try again.\n");
      }
      else if (toggle_primary_leg_state_)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Cannot toggle secondary leg state as secondary leg is currently "
                                           "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Please wait and try again.\n");
      }
      else
      {
        secondary_leg_state_ = newSecondaryLegState;
        toggle_secondary_leg_state_ = true;
      }
    }
  }
}

/***********************************************************************************************************************
 * IMU data callback
***********************************************************************************************************************/
void StateController::imuCallback(const sensor_msgs::Imu &data)
{
  Quat imuRotationOffset(params_.imu_rotation_offset);

  Quat rawOrientation;
  rawOrientation.w = data.orientation.w;
  rawOrientation.x = data.orientation.x;
  rawOrientation.y = data.orientation.y;
  rawOrientation.z = data.orientation.z;

  Vector3d rawLinearAcceleration;
  rawLinearAcceleration[0] = data.linear_acceleration.x;
  rawLinearAcceleration[1] = data.linear_acceleration.y;
  rawLinearAcceleration[2] = data.linear_acceleration.z;

  Vector3d rawAngularVelocity;
  rawAngularVelocity[0] = data.angular_velocity.x;
  rawAngularVelocity[1] = data.angular_velocity.y;
  rawAngularVelocity[2] = data.angular_velocity.z;

  // Rotate raw imu data according to physical imu mounting
  imu_data_.orientation = (imuRotationOffset * rawOrientation) * imuRotationOffset.inverse();
  imu_data_.linear_acceleration = imuRotationOffset.toRotationMatrix() * rawLinearAcceleration;
  imu_data_.angular_velocity = imuRotationOffset.toRotationMatrix() * rawAngularVelocity;
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void StateController::jointStatesCallback(const sensor_msgs::JointState &joint_states)
{
  bool get_effort_values = (joint_states.effort.size() != 0);
  bool get_velocity_values = (joint_states.velocity.size() != 0);

  for (uint i = 0; i < joint_states.name.size(); ++i)
  {
    const char *joint_name = joint_states.name[i].c_str();
    typedef std::map<std::string, Leg*>::iterator it_type;
    for (it_type leg_it = model_->getLegContainer().begin(); leg_it != model_->getLegContainer()->end(); ++leg_it)
    {
      Leg3DOF* leg = leg_it->second();
      leg->updateJointPositionActual(joint_name, joint_states.position[i]);
      leg->updateJointVelocityActual(joint_name, joint_states.velocity[i]);
      leg->updateJointEffortActual(joint_name, joint_states.effort[i]);
    }
  }

    
    
    
    if (!strcmp(joint_name, "front_left_body_coxa") || !strcmp(joint_name, "AL_coxa_joint"))
    {
      joint_positions_[0] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[0] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[0] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "front_left_coxa_femour") || !strcmp(joint_name, "AL_femur_joint"))
    {
      joint_positions_[1] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[1] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[1] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "front_left_femour_tibia") || !strcmp(joint_name, "AL_tibia_joint"))
    {
      joint_positions_[2] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[2] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[2] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "front_right_body_coxa") || !strcmp(joint_name, "AR_coxa_joint"))
    {
      joint_positions_[3] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[3] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[3] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "front_right_coxa_femour") || !strcmp(joint_name, "AR_femur_joint"))
    {
      joint_positions_[4] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[4] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[4] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "front_right_femour_tibia") || !strcmp(joint_name, "AR_tibia_joint"))
    {
      joint_positions_[5] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[5] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[5] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_left_body_coxa") || !strcmp(joint_name, "BL_coxa_joint"))
    {
      joint_positions_[6] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[6] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[6] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_left_coxa_femour") || !strcmp(joint_name, "BL_femur_joint"))
    {
      joint_positions_[7] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[7] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[7] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_left_femour_tibia") || !strcmp(joint_name, "BL_tibia_joint"))
    {
      joint_positions_[8] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[8] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[8] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_right_body_coxa") || !strcmp(joint_name, "BR_coxa_joint"))
    {
      joint_positions_[9] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[9] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[9] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_right_coxa_femour") || !strcmp(joint_name, "BR_femur_joint"))
    {
      joint_positions_[10] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[10] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[10] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "middle_right_femour_tibia") || !strcmp(joint_name, "BR_tibia_joint"))
    {
      joint_positions_[11] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[11] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[11] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_left_body_coxa") || !strcmp(joint_name, "CL_coxa_joint"))
    {
      joint_positions_[12] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[12] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[12] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_left_coxa_femour") || !strcmp(joint_name, "CL_femur_joint"))
    {
      joint_positions_[13] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[13] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[13] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_left_femour_tibia") || !strcmp(joint_name, "CL_tibia_joint"))
    {
      joint_positions_[14] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[14] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[14] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_right_body_coxa") || !strcmp(joint_name, "CR_coxa_joint"))
    {
      joint_positions_[15] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[15] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[15] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_right_coxa_femour") || !strcmp(joint_name, "CR_femur_joint"))
    {
      joint_positions_[16] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[16] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[16] = joint_states.effort[i];
      }
    }
    else if (!strcmp(joint_name, "rear_right_femour_tibia") || !strcmp(joint_name, "CR_tibia_joint"))
    {
      joint_positions_[17] = joint_states.position[i];
      if (get_velocity_values)
      {
        jointVelocities[17] = joint_states.velocity[i];
      }
      if (get_effort_values)
      {
        jointEfforts[17] = joint_states.effort[i];
      }
    }

    // Check if all joint positions have been received from topic
    received_all_joint_positions_ = true;
    for (int i = 0; i < 18; i++)
    {
      if (joint_positions_[i] > 1e9)
      {
        received_all_joint_positions_ = false;
      }
    }
  }
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallback(const sensor_msgs::JointState &jointStates)
{
  tipForces[0] = jointStates.effort[0];
  tipForces[1] = jointStates.effort[2];
  tipForces[2] = jointStates.effort[4];
  tipForces[3] = jointStates.effort[6];
  tipForces[4] = jointStates.effort[8];
  tipForces[5] = jointStates.effort[10];
}

/***********************************************************************************************************************
 * Gets hexapod parameters from rosparam server
***********************************************************************************************************************/
void StateController::getParameters()
{
  typedef std::map<std::string, Parameter> p;
  params_map_.clear();
  //Control parameters
  params_map_.insert(p::value_type("time_delta", &params_.time_delta));
  params_map_.insert(p::value_type("imu_compensation", &params_.imu_compensation));
  params_map_.insert(p::value_type("auto_compensation", &params_.auto_compensation));
  params_map_.insert(p::value_type("manual_compensation", &params_.manual_compensation));
  params_map_.insert(p::value_type("inclination_compensation", &params_.inclination_compensation));
  params_map_.insert(p::value_type("impedance_control", &params_.impedance_control));
  params_map_.insert(p::value_type("use_dynamixel_pro_interface", &params_.use_dynamixel_pro_interface));
  params_map_.insert(p::value_type("imu_rotation_offset", &params_.imu_rotation_offset)); //TBD Remove and use tf instead
  params_map_.insert(p::value_type("interface_setup_speed", &params_.interface_setup_speed));
  // Model parameters - provision for max 8 legs with 5 joints each (40DOF)
  params_map_.insert(p::value_type("hexapod_type", &params_.hexapod_type));
  params_map_.insert(p::value_type("num_legs", &params_.num_legs));
  params_map_.insert(p::value_type("leg_DOF", &params_.leg_DOF));
  params_map_.insert(p::value_type("leg_id_names", &params_.leg_id_names));  
  params_map_.insert(p::value_type("AL_0_joint_parameters", &params_.AL_0_joint_parameters));
  params_map_.insert(p::value_type("AL_1_joint_parameters", &params_.AL_1_joint_parameters));
  params_map_.insert(p::value_type("AL_2_joint_parameters", &params_.AL_2_joint_parameters));
  params_map_.insert(p::value_type("AL_3_joint_parameters", &params_.AL_3_joint_parameters));
  params_map_.insert(p::value_type("AL_4_joint_parameters", &params_.AL_4_joint_parameters));
  params_map_.insert(p::value_type("AR_0_joint_parameters", &params_.AR_0_joint_parameters));
  params_map_.insert(p::value_type("AR_1_joint_parameters", &params_.AR_1_joint_parameters));
  params_map_.insert(p::value_type("AR_2_joint_parameters", &params_.AR_2_joint_parameters));
  params_map_.insert(p::value_type("AR_3_joint_parameters", &params_.AR_3_joint_parameters));
  params_map_.insert(p::value_type("AR_4_joint_parameters", &params_.AR_4_joint_parameters));
  params_map_.insert(p::value_type("BL_0_joint_parameters", &params_.BL_0_joint_parameters));
  params_map_.insert(p::value_type("BL_1_joint_parameters", &params_.BL_1_joint_parameters));
  params_map_.insert(p::value_type("BL_2_joint_parameters", &params_.BL_2_joint_parameters));
  params_map_.insert(p::value_type("BL_3_joint_parameters", &params_.BL_3_joint_parameters));
  params_map_.insert(p::value_type("BL_4_joint_parameters", &params_.BL_4_joint_parameters));
  params_map_.insert(p::value_type("BR_0_joint_parameters", &params_.BR_0_joint_parameters));
  params_map_.insert(p::value_type("BR_1_joint_parameters", &params_.BR_1_joint_parameters));
  params_map_.insert(p::value_type("BR_2_joint_parameters", &params_.BR_2_joint_parameters));
  params_map_.insert(p::value_type("BR_3_joint_parameters", &params_.BR_3_joint_parameters));
  params_map_.insert(p::value_type("BR_4_joint_parameters", &params_.BR_4_joint_parameters));
  params_map_.insert(p::value_type("CL_0_joint_parameters", &params_.CL_0_joint_parameters));
  params_map_.insert(p::value_type("CL_1_joint_parameters", &params_.CL_1_joint_parameters));
  params_map_.insert(p::value_type("CL_2_joint_parameters", &params_.CL_2_joint_parameters));
  params_map_.insert(p::value_type("CL_3_joint_parameters", &params_.CL_3_joint_parameters));
  params_map_.insert(p::value_type("CL_4_joint_parameters", &params_.CL_4_joint_parameters));
  params_map_.insert(p::value_type("CR_0_joint_parameters", &params_.CR_0_joint_parameters));
  params_map_.insert(p::value_type("CR_1_joint_parameters", &params_.CR_1_joint_parameters));
  params_map_.insert(p::value_type("CR_2_joint_parameters", &params_.CR_2_joint_parameters));
  params_map_.insert(p::value_type("CR_3_joint_parameters", &params_.CR_3_joint_parameters));
  params_map_.insert(p::value_type("CR_4_joint_parameters", &params_.CR_4_joint_parameters));
  params_map_.insert(p::value_type("DL_0_joint_parameters", &params_.DL_0_joint_parameters));
  params_map_.insert(p::value_type("DL_1_joint_parameters", &params_.DL_1_joint_parameters));
  params_map_.insert(p::value_type("DL_2_joint_parameters", &params_.DL_2_joint_parameters));
  params_map_.insert(p::value_type("DL_3_joint_parameters", &params_.DL_3_joint_parameters));
  params_map_.insert(p::value_type("DL_4_joint_parameters", &params_.DL_4_joint_parameters));
  params_map_.insert(p::value_type("DR_0_joint_parameters", &params_.DR_0_joint_parameters));
  params_map_.insert(p::value_type("DR_1_joint_parameters", &params_.DR_1_joint_parameters));
  params_map_.insert(p::value_type("DR_2_joint_parameters", &params_.DR_2_joint_parameters));
  params_map_.insert(p::value_type("DR_3_joint_parameters", &params_.DR_3_joint_parameters));
  params_map_.insert(p::value_type("DR_4_joint_parameters", &params_.DR_4_joint_parameters));
  // Walker parameters
  params_map_.insert(p::value_type("gait_type", &params_.gait_type));
  params_map_.insert(p::value_type("step_frequency", &params_.step_frequency));
  params_map_.insert(p::value_type("step_clearance", &params_.step_clearance));
  params_map_.insert(p::value_type("step_depth", &params_.step_depth));
  params_map_.insert(p::value_type("body_clearance", &params_.body_clearance));
  params_map_.insert(p::value_type("leg_span_scale", &params_.leg_span_scale));
  params_map_.insert(p::value_type("max_linear_acceleration", &params_.max_linear_acceleration));
  params_map_.insert(p::value_type("max_angular_acceleration", &params_.max_angular_acceleration));
  params_map_.insert(p::value_type("footprint_downscale", &params_.footprint_downscale));
  params_map_.insert(p::value_type("velocity_input_mode", &params_.velocity_input_mode));
  params_map_.insert(p::value_type("force_cruise_velocity", &params_.force_cruise_velocity));
  params_map_.insert(p::value_type("linear_cruise_velocity", &params_.linear_cruise_velocity));
  params_map_.insert(p::value_type("angular_cruise_velocity", &params_.angular_cruise_velocity));
  // Poser parameters
  params_map_.insert(p::value_type("start_up_sequence", &params_.start_up_sequence));
  params_map_.insert(p::value_type("time_to_start", &params_.time_to_start));  
  params_map_.insert(p::value_type("rotation_gain_p", &params_.rotation_gain_p));
  params_map_.insert(p::value_type("rotation_gain_i", &params_.rotation_gain_i));
  params_map_.insert(p::value_type("rotation_gain_d", &params_.rotation_gain_d));
  params_map_.insert(p::value_type("translation_gain_p", &params_.translation_gain_p));  
  params_map_.insert(p::value_type("translation_gain_i", &params_.translation_gain_i));
  params_map_.insert(p::value_type("translation_gain_d", &params_.translation_gain_d));  
  params_map_.insert(p::value_type("pitch_amplitude", &params_.pitch_amplitude));
  params_map_.insert(p::value_type("roll_amplitude", &params_.roll_amplitude));
  params_map_.insert(p::value_type("z_translation_amplitude", &params_.z_translation_amplitude));  
  params_map_.insert(p::value_type("max_translation", &params_.max_translation));
  params_map_.insert(p::value_type("max_translation_velocity", &params_.max_translation_velocity));
  params_map_.insert(p::value_type("max_rotation", &params_.max_rotation));
  params_map_.insert(p::value_type("max_rotation_velocity", &params_.max_rotation_velocity));  
  params_map_.insert(p::value_type("leg_manipulation_mode", &params_.leg_manipulation_mode)); 
  // Impedance controller parameters
  params_map_.insert(p::value_type("impedance_control", &params_.impedance_control));
  params_map_.insert(p::value_type("dynamic_stiffness", &params_.dynamic_stiffness));
  params_map_.insert(p::value_type("integrator_step_time", &params_.integrator_step_time));
  params_map_.insert(p::value_type("virtual_mass", &params_.virtual_mass));
  params_map_.insert(p::value_type("virtual_stiffness", &params_.virtual_stiffness));
  params_map_.insert(p::value_type("load_stiffness_scaler", &params_.load_stiffness_scaler));
  params_map_.insert(p::value_type("swing_stiffness_scaler", &params_.swing_stiffness_scaler));
  params_map_.insert(p::value_type("virtual_damping_ratio", &params_.virtual_damping_ratio));
  params_map_.insert(p::value_type("force_gain", &params_.force_gain));
  params_map_.insert(p::value_type("impedance_input", &params_.impedance_input));
  // Debug parameters
  params_map_.insert(p::value_type("debug_rviz", &params_.debug_rviz));
  params_map_.insert(p::value_type("console_verbosity", &params_.console_verbosity));
  params_map_.insert(p::value_type("debug_moveToJointPosition", &params_.debug_moveToJointPosition));
  params_map_.insert(p::value_type("debug_stepToPosition", &params_.debug_stepToPosition));
  params_map_.insert(p::value_type("debug_swing_trajectory", &params_.debug_swing_trajectory));
  params_map_.insert(p::value_type("debug_stance_trajectory", &params_.debug_stance_trajectory));
  // Gait parameters
  gait_params_map_.insert(p::value_type("stance_phase", &params_.stance_phase));
  gait_params_map_.insert(p::value_type("swing_phase", &params_.swing_phase));
  gait_params_map_.insert(p::value_type("phase_offset", &params_.phase_offset));
  gait_params_map_.insert(p::value_type("offset_multiplier", &params_.offset_multiplier)); 
  
  std::string baseParamString = "/hexapod/parameters/";
  typedef std::map<std::string, Parameter>::iterator it_type;
  for(it_type param_it = params_map_.begin(); param_it != params_map_.end(); ++param_it)
  {
    Parameter* parameter = param_it->second();
    parameter->name = param_it->first();
    bool parameter_found  = n_.getParam(baseParamString + parameter->name, parameter->default_value);
    bool ignore_error = false;
    std::string param_type(parameter->name, 5);
    if (!parameter_found && (param_type == 'joint_parameters' || param_type == 'link_parameters'))
    {      
      std::string leg_id(parameter->name, 0, 2);
      std::string joint_id(parameter->name, 3, 1);
      map<std::string, int> legs = params_.leg_info.current_value;
      ignore_error = (legs.find(leg_id) == legs.end()) || stringToInt(joint_id) > legs[leg_id];
    }
      
    parameter->current_value = parameter->default_value;
    ROS_ERROR_COND(!parameter_found && !ignore_error, 
		   "Error reading parameter/s %s from rosparam. Check config file is loaded and type is correct\n",
		   parameter->name());
  } 
  
  getGaitParameters(GAIT_UNDESIGNATED); //Get gait parameters seperately as it depends on gait designation
}

/***********************************************************************************************************************
 * Gets hexapod gait parameters from rosparam server
***********************************************************************************************************************/
void StateController::getGaitParameters(GaitDesignation gaitSelection)
{  
  switch (gaitSelection)
  {
    case (TRIPOD_GAIT):
      params_.gait_type = "tripod_gait";
      break;
    case (RIPPLE_GAIT):
      params_.gait_type = "ripple_gait";
      break;
    case (WAVE_GAIT):
      params_.gait_type = "wave_gait";
      break;
    case (AMBLE_GAIT):
      params_.gait_type = "amble_gait";
      break;
    case (GAIT_UNDESIGNATED):
      if (!n_.getParam("/hexapod/parameters/default_gait_type", params_.gait_type))
      {
        ROS_ERROR("Error reading parameter/s (default_gait_type) from rosparam. Check config file is loaded and type is "
                  "correct\n");
      }
      break;
  }

  std::string base_param_string = "/hexapod/gait_parameters/" + params_.gait_type + "/";
  typedef std::map<std::string, Parameter>::iterator it_type;
  for(it_type param_it = gait_params_map_.begin(); param_it != gait_params_map_.end(); ++param_it)
  {
    Parameter* parameter = param_it->second();
    parameter->name = param_it->first();
    bool parameter_found  = n_.getParam(base_param_string + parameter->name, parameter->default_value);
    parameter->current_value = parameter->default_value;
    ROS_ERROR_COND(!parameter_found, 
		   "Error reading parameter/s %s from rosparam. Check config file is loaded and type is correct\n",
		   parameter->name());
  } 
}
/***********************************************************************************************************************
***********************************************************************************************************************/
