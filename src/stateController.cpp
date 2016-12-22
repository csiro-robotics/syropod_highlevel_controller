#include "simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * State controller contructor
***********************************************************************************************************************/
StateController::StateController(ros::NodeHandle nodeHandle) : n(nodeHandle)
{
  // Get parameters from parameter server
  getParameters();
  defaultParams = params;  // Store original params as a default

  // Initiate model and imu objects
  hexapod = new Model(params);

  populateParameterNameMap();

  // Populate joint position arrays
  for (int i = 0; i < 18; i++)
  {
    jointPositions[i] = 1e10;  // Excessive to make obvious when real values are read from subscribed topic
    jointVelocities[i] = 0.0;
    jointEfforts[i] = 0.0;
    if (i < 6)
    {
      tipForces[i] = 0;
    }
  }
}

/***********************************************************************************************************************
 * State controller destructor
***********************************************************************************************************************/
StateController::~StateController()
{
  delete hexapod;
  delete walker;
  delete poser;
  delete impedance;
}

/***********************************************************************************************************************
 * Init state controller
***********************************************************************************************************************/
void StateController::init()
{
  // Setup motor interface
  if (params.use_dynamixel_pro_interface)
  {
    interface = new DynamixelProMotorInterface();
  }
  else
  {
    interface = new DynamixelMotorInterface();
  }

  interface->setupSpeed(params.interface_setup_speed);

  // Set initial gait selection number for gait toggling
  if (params.gait_type == "tripod_gait")
  {
    gaitSelection = TRIPOD_GAIT;
  }
  else if (params.gait_type == "ripple_gait")
  {
    gaitSelection = RIPPLE_GAIT;
  }
  else if (params.gait_type == "wave_gait")
  {
    gaitSelection = WAVE_GAIT;
  }
  else if (params.gait_type == "amble_gait")
  {
    gaitSelection = AMBLE_GAIT;
  }

  // Create controller objects
  walker = new WalkController(hexapod, params);
  poser = new PoseController(hexapod, walker, params);
  impedance = new ImpedanceController(params);

  // Get unpacked/packed joint positions from params
  unpackedJointPositions[0][0] = params.unpacked_joint_positions_AL;
  unpackedJointPositions[0][1] = params.unpacked_joint_positions_AR;
  unpackedJointPositions[1][0] = params.unpacked_joint_positions_BL;
  unpackedJointPositions[1][1] = params.unpacked_joint_positions_BR;
  unpackedJointPositions[2][0] = params.unpacked_joint_positions_CL;
  unpackedJointPositions[2][1] = params.unpacked_joint_positions_CR;
  packedJointPositions[0][0] = params.packed_joint_positions_AL;
  packedJointPositions[0][1] = params.packed_joint_positions_AR;
  packedJointPositions[1][0] = params.packed_joint_positions_BL;
  packedJointPositions[1][1] = params.packed_joint_positions_BR;
  packedJointPositions[2][0] = params.packed_joint_positions_CL;
  packedJointPositions[2][1] = params.packed_joint_positions_CR;

  linearVelocityInput = Vector2d(0.0, 0.0);

  isInitialised = true;
}

/***********************************************************************************************************************
 * Generates map of all parameter names
***********************************************************************************************************************/
void StateController::populateParameterNameMap()
{
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(NO_PARAMETER_SELECTION, "No"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(STEP_FREQUENCY, "step_frequency"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(STEP_CLEARANCE, "step_clearance"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(BODY_CLEARANCE, "body_clearance"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(LEG_SPAN_SCALE, "leg_span_scale"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(VIRTUAL_MASS, "virtual_mass"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(VIRTUAL_STIFFNESS, "virtual_"
                                                                                                   "stiffness"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(VIRTUAL_DAMPING, "virtual_damping"));
  parameterNameMap.insert(std::map<ParameterSelection, std::string>::value_type(FORCE_GAIN, "force_gain"));
}

/***********************************************************************************************************************
 * Sets joint values in model according to aquired joint positions (set to default values if required)
***********************************************************************************************************************/
void StateController::setJointPositions(bool useDefaults)
{
  if (useDefaults)
  {
    int index = 0;
    typedef std::map<std::string, Leg*>::iterator it_type;
    for (it_type it = hexapod->getLegContainer().begin(); it != hexapod->getLegContainer()->end(); ++it)
    {
      Leg leg = it->second();
      double dir = leg.getMirrorDir();
      if (jointPositions[index] == 1e10)
      {
	jointPositions[index] = 0.0;
	ROS_INFO("%s coxa joint set to: %f", leg.id_name.c_str(), jointPositions[index]);
      }
      if (jointPositions[index + 1] == 1e10)
      {
	jointPositions[index + 1] = dir * max(0.0, hexapod->min_max_femur_angle_[0]);
	ROS_INFO("%s femur joint set to: %f", leg.id_name.c_str(), jointPositions[index + 1]);
      }
      if (jointPositions[index + 2] == 1e10)
      {
	jointPositions[index + 2] = dir * max(0.0, hexapod->min_max_tibia_angle_[0]);
	ROS_INFO("%s tibia joint set to: %f", leg.id_name.c_str(), jointPositions[index + 2]);
      }
      params.start_up_sequence = false;
      index++;
    }
  }

  // Set initial leg angles
  int index = 0;
  typedef std::map<std::string, Leg*>::iterator it_type;
  for (it_type it = hexapod->getLegContainer().begin(); it != hexapod->getLegContainer()->end(); ++it)
  {
    Leg leg = it->second();
    double dir = leg.getMirrorDir();
    LegType leg_type = leg.getType();
    int num_joints = static_cast<int>(leg_type);
    switch(leg_type)
    {
      case (3_DOF):
      {
	Leg3DOF leg = it->second();
	leg.init(jointPositions[index + 0] + dir * params.physical_coxa_angle_offset[leg], 
		 -jointPositions[index + 1], jointPositions[index + 2] + dir * params.physical_tibia_angle_offset);
	break;
      }
      default: //TBD implement other DOF leg types
      {
	ROS_FATAL("Only 3 degree of freedom legs currently supported");
	break;
      }
    }
    index += num_joints;
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
  if (params.impedance_control)
  {
    impedanceControl();
  }

  // Hexapod state machine
  if (transitionSystemStateFlag)
  {
    transitionSystemState();
  }
  else if (systemState == RUNNING)
  {
    runningState();
  }
  hexapod->clampToLimits(legNameMap);
}

/***********************************************************************************************************************
 * State transition handler
***********************************************************************************************************************/
void StateController::transitionSystemState()
{
  // UNKNOWN -> OFF/PACKED/READY/RUNNING  if (systemState == UNKNOWN)
  if (systemState == UNKNOWN)
  {
    int checkPacked = 0;
    typedef std::map<std::string, Leg*>::iterator it_type;
    for (it_type it = hexapod->getLegContainer().begin(); it != hexapod->getLegContainer()->end(); ++it)
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
      if (!params.start_up_sequence)
      {
        ROS_FATAL("Hexapod currently in packed state and cannot run direct startup sequence.\nEither manually unpack "
                  "hexapod or set start_up_sequence to true in config file\n");
        ros::shutdown();
      }
      else
      {
        systemState = PACKED;
        ROS_INFO("Hexapod currently packed.\n");
      }
    }
    else if (!params.start_up_sequence)
    {
      ROS_WARN("start_up_sequence parameter is set to false, ensure hexapod is off the ground before transitioning "
               "system state.\n");
      systemState = OFF;
    }
    else
    {
      // double timeOut = 30;
      systemState = PACKED;
      ROS_WARN("Hexapod state is unknown. Future state transitions may be undesireable, recommend ensuring hexapod is "
               "off the ground before proceeding.\n");
      // ROS_WARN("Suspending controller for %f seconds. Any desired state transitions will begin after this period.\n",
      // timeOut);
      // sleep(timeOut);
      // ROS_WARN("Controller resuming.\n");
    }
  }
  // OFF -> !OFF (Start controller or directly transition to walking stance)
  else if (systemState == OFF && newSystemState != OFF)
  {
    // OFF -> RUNNING (Direct startup)
    if (newSystemState == RUNNING && !params.start_up_sequence)
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning directly to RUNNING state . . .\n");
      //walker->init(hexapod, params); TBD
      double res = poser->directStartup(hexapod);
      if (res == 1.0)
      {
        systemState = RUNNING;
        ROS_INFO("Direct startup sequence complete. Ready to walk.\n");
      }
    }
    // OFF -> PACKED/READY/RUNNING (Start controller)
    else
    {
      systemState = PACKED;
      ROS_INFO("Controller running.\n");
    }
  }
  // PACKED -> OFF (Suspend controller)
  else if (systemState == PACKED && newSystemState == OFF)
  {
    systemState = OFF;
    ROS_INFO("Controller suspended.\n");
  }
  // PACKED -> READY/RUNNING (Unpack Hexapod)
  else if (systemState == PACKED && (newSystemState == READY || newSystemState == RUNNING))
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to READY state . . .\n");
    int completed_count = 0;    
    if (poser->unpackLegs(2.0 / params.step_frequency))
    {
      systemState = READY;
      ROS_INFO("State transition complete. Hexapod is in READY state.\n");
    }
  }
  // READY -> PACKED/OFF (Pack Hexapod)
  else if (systemState == READY && (newSystemState == PACKED || newSystemState == OFF))
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to PACKED state . . .\n");
    if (poser->moveToJointPosition(packedJointPositions, 2.0 / params.step_frequency))
    {
      systemState = PACKED;
      ROS_INFO("State transition complete. Hexapod is in PACKED state.\n");
    }
  }
  // READY -> RUNNING (Initate start up sequence to step to walking stance)
  else if (systemState == READY && newSystemState == RUNNING)
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to RUNNING state . . .\n");
    if (poser->startUpSequence(walker->identityTipPositions, poser->getCurrentPose(), deltaZ, params.move_legs_sequentially))
    {
      systemState = RUNNING;
      ROS_INFO("State transition complete. Hexapod is in RUNNING state. Ready to walk.\n");
    }
  }
  // RUNNING -> !RUNNING (Initiate shut down sequence to step from walking stance to ready stance or suspend controller)
  else if (systemState == RUNNING && newSystemState != RUNNING)
  {
    // RUNNING -> OFF (Suspend controller)
    if (newSystemState == OFF && !params.start_up_sequence)
    {
      systemState = OFF;
      ROS_INFO("Controller suspended.\n");
    }
    else
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Hexapod transitioning to READY state . . .\n");
      if (poser->shutDownSequence(poser->getTipPositions(), poser->getCurrentPose(), deltaZ, params.move_legs_sequentially))
      {
        systemState = READY;
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
  if (systemState == newSystemState)
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
  if (cruiseControlMode == CRUISE_CONTROL_ON)
  {
    if (params.force_cruise_velocity)
    {
      linearVelocityInput = params.linear_cruise_velocity;
      angularVelocityInput = params.angular_cruise_velocity;
    }
    else
    {
      linearVelocityInput = linearCruiseVelocity;
      angularVelocityInput = angularCruiseVelocity;
    }
  }

  // Switch gait and update walker parameters
  if (gaitChangeFlag)
  {
    gaitChange();
  }
  // Dynamically adjust parameters and change stance if required
  else if (parameterAdjustFlag)
  {
    parameterAdjust();
  }
  // Toggle state of leg and transition between states
  else if (togglePrimaryLegState || toggleSecondaryLegState)
  {
    legStateToggle();
  }

  // Update tip positions unless hexapod is undergoing gait switch, parameter adjustment or leg state transition
  //(which all only occur once the hexapod has stopped walking)
  if (!((gaitChangeFlag || parameterAdjustFlag || togglePrimaryLegState || toggleSecondaryLegState) &&
        walker->walk_state_ == STOPPED))
  {
    
    // Update tip positions for walking legs
    walker->updateWalk(linearVelocityInput, angularVelocityInput, hexapod);

    // Update tip positions for manually controlled legs
    walker->updateManual(primaryLegSelection, primaryManualTipVelocity, secondaryLegSelection,
                         secondaryManualTipVelocity, hexapod);

    // Pose controller takes current tip positions from walker and applies pose compensation
    poser->updateStance(walker->tipPositions, params.auto_compensation && !params.imu_compensation);

    // Model uses posed tip positions and adds deltaZ from impedance controller and applies inverse kinematics on each
    // leg
    hexapod->updateLocal(poser->getTipPositions(), deltaZ);
  }
}

/***********************************************************************************************************************
 * Compensation
***********************************************************************************************************************/
void StateController::updatePose()
{
  // Manually set (joystick controlled) body compensation
  Pose new_pose;
  if (params.manual_compensation)
  {
    poser->manualCompensation(Vector3d(xJoy, yJoy, zJoy), Vector3d(rollJoy, pitchJoy, yawJoy), poseResetMode,
                              poser->defaultPose);
    new_pose = poser->manualPose;
  }
  else
  {
    new_pose = Pose::identity();
  }

  // Compensation to align centre of gravity evenly between tip positions on incline
  if (params.inclination_compensation)
  {
    poser->inclinationCompensation(imuData);
    new_pose.position_ += poser->inclinationPose.position_;
  }

  // Compensation to offset average deltaZ from impedance controller and keep body at specificied height
  if (params.impedance_control)
  {
    poser->impedanceControllerCompensation(deltaZ);
    new_pose.position_ += poser->deltaZPose.position_;
  }

  // Auto body compensation using IMU feedback
  if (params.imu_compensation)
  {
    poser->imuCompensation(imuData, poser->manualPose.rotation_);
    new_pose.rotation_ = poser->imuPose.rotation_;
  }
  // Automatic (non-feedback) body compensation
  else if (params.auto_compensation)
  {
    poser->autoCompensation();
    new_pose.position_ += poser->autoPoseDefault.position_;
    new_pose.rotation_ *= poser->autoPoseDefault.rotation_;  //(Quaternion)
  }
  hexapod->setCurrentPose(new_pose);
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
      if (walker->legSteppers[l][s].step_state_ != SWING)
      {
        legsInStance++;
      }
    }
  }

  // Calculate new stiffness based on imu orientation or walking cycle
  bool useIMUForStiffness = false;  // Not fully tested
  if (params.dynamic_stiffness && walker->walk_state_ != STOPPED)
  {
    if (params.imu_compensation && useIMUForStiffness)
    {
      impedance->updateStiffness(poser->current_pose_, walker->identityTipPositions);  // TBD CHECK
    }
    else
    {
      impedance->updateStiffness(walker);
    }
  }

  // Get current force value on leg and run impedance calculations to get a vertical tip offset (deltaZ)
  for (int l = 0; l < 3; l++)
  {
    for (int s = 0; s < 2; s++)
    {
      if (params.dynamic_stiffness || legsInStance == 6)
      {
        double maxForce = 0;
        double minForce = 0;
        if (useTipForce)
        {
          double forceOffset = 1255.0;
          tipForce[l][s] = tipForces[2 * l + s] - forceOffset;
          maxForce = 1000.0;
          minForce = 0.0;
        }
        else if (useJointEffort)
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

      if (hexapod->legs_[l][s].leg_state_ == WALKING)
      {
        impedance->updateImpedance(l, s, tipForce, deltaZ);
      }
    }
  }
}

/***********************************************************************************************************************
 * Dynamic Parameter adjustment
***********************************************************************************************************************/
void StateController::parameterAdjust()
{
  if (walker->walk_state_ == STOPPED)
  {
    if (!newParamSet)
    {
      switch (parameterSelection)
      {
        case (NO_PARAMETER_SELECTION):
        {
          break;
        }
        case (STEP_FREQUENCY):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.step_frequency / defaultParams.step_frequency;
          }
          params.step_frequency = minMax(defaultParams.step_frequency * paramScaler, 0.1, 3.0);
          walker->setGaitParams(params);
          poser->parameters = params;
          paramString = "step_frequency";
          paramVal = params.step_frequency;
          break;
        }
        case (STEP_CLEARANCE):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.step_clearance / defaultParams.step_clearance;
          }
          params.step_clearance = minMax(defaultParams.step_clearance * paramScaler, 0.01, 0.4);
          walker->init(hexapod, params);
          paramString = "step_clearance";
          paramVal = params.step_clearance;
          break;
        }
        case (BODY_CLEARANCE):
        {
          if (defaultParams.body_clearance == -1)
          {
            params.body_clearance = walker->body_clearance_;
            defaultParams.body_clearance = params.body_clearance;
          }
          if (paramScaler == -1)
          {
            paramScaler = params.body_clearance / defaultParams.body_clearance;
          }
          params.body_clearance = minMax(defaultParams.body_clearance * paramScaler, 0.1, 0.99);
          walker->init(hexapod, params);
          paramString = "body_clearance";
          paramVal = params.body_clearance;
          break;
        }
        case (LEG_SPAN_SCALE):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.leg_span_scale / defaultParams.leg_span_scale;
          }
          params.leg_span_scale = minMax(defaultParams.leg_span_scale * paramScaler, 0.1, 1.5);
          walker->init(hexapod, params);
          paramString = "leg_span_scale";
          paramVal = params.leg_span_scale;
          break;
        }
        case (VIRTUAL_MASS):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.virtual_mass / defaultParams.virtual_mass;
          }
          params.virtual_mass = minMax(defaultParams.virtual_mass * paramScaler, 0, 500);
          impedance->init(params);
          paramString = "virtual_mass";
          paramVal = params.virtual_mass;
          break;
        }
        case (VIRTUAL_STIFFNESS):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.virtual_stiffness / defaultParams.virtual_stiffness;
          }
          params.virtual_stiffness = minMax(defaultParams.virtual_stiffness * paramScaler, 0, 500);
          impedance->init(params);
          paramString = "virtual_stiffness";
          paramVal = params.virtual_stiffness;
          break;
        }
        case (VIRTUAL_DAMPING):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.virtual_damping_ratio / defaultParams.virtual_damping_ratio;
          }
          params.virtual_damping_ratio = minMax(defaultParams.virtual_damping_ratio * paramScaler, 0, 2.0);
          impedance->init(params);
          paramString = "virtual_damping_ratio";
          paramVal = params.virtual_damping_ratio;
          break;
        }
        case (FORCE_GAIN):
        {
          if (paramScaler == -1)
          {
            paramScaler = params.force_gain / defaultParams.force_gain;
          }
          params.force_gain = minMax(defaultParams.force_gain * paramScaler, 0, 2.0);
          impedance->init(params);
          paramString = "force_gain";
          paramVal = params.force_gain;
          break;
        }
        default:
        {
          paramString = "unknown";
          paramVal = -1;
          paramScaler = -1;
          ROS_WARN("Attempting to adjust unknown parameter.\n");
          break;
        }
      }
      newParamSet = true;
      ROS_INFO("Attempting to adjust '%s' parameter to %d%% of default (%f) . . .\n", paramString.c_str(),
               roundToInt(paramScaler * 100), paramVal);
    }
    else
    {
      // Update tip Positions for new parameter value
      double stepHeight = walker->maximum_body_height_ * walker->step_clearance_;
      double res =
          poser->stepLegsToPosition(walker->identityTipPositions, poser->current_pose_, deltaZ, TRIPOD_MODE, stepHeight);
      if (res == 1.0)
      {
        ROS_INFO("Parameter '%s' set to %d%% of default (%f).\n", paramString.c_str(), roundToInt(paramScaler * 100),
                 paramVal);
        parameterAdjustFlag = false;
        newParamSet = false;
      }
    }
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to adjust parameters . . .\n");
    linearVelocityInput = Vector2d(0.0, 0.0);
    angularVelocityInput = 0.0;
  }
}

/***********************************************************************************************************************
 * Gait change
***********************************************************************************************************************/
void StateController::gaitChange()
{
  if (walker->walk_state_ == STOPPED)
  {
    getGaitParameters(gaitSelection);
    walker->setGaitParams(params);
    ROS_INFO("Now using %s mode.\n", params.gait_type.c_str());
    gaitChangeFlag = false;
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to change gait . . .\n");
    linearVelocityInput = Vector2d(0.0, 0.0);
    angularVelocityInput = 0.0;
  }
}

/***********************************************************************************************************************
 * Leg State Toggle
***********************************************************************************************************************/
void StateController::legStateToggle()
{
  if (walker->walk_state_ == STOPPED)
  {
    // Choose primary or secondary leg state to transition
    Leg3DOF* transitioning_leg;
    int l;
    int s;
    if (togglePrimaryLegState)
    {
      transitioning_leg = hexapod->getLegByID(primaryLegSelection);
    }
    else if (toggleSecondaryLegState)
    {
      transitioning_leg = hexapod->getLegByID(secondaryLegSelection);
    }

    // Calculate default pose for new loading pattern
    poser->calculateDefaultPose();

    if (transitioning_leg->getLegState() == WALKING)
    {
      if (manualLegs < MAX_MANUAL_LEGS)
      {
        ROS_INFO_COND(leg.leg_state_ == WALKING, "%s leg transitioning to MANUAL state . . .\n",
                      legNameMap[l * 2 + s].c_str());
        transitioning_leg->setLegState(WALKING_TO_MANUAL);
      }
      else
      {
        ROS_INFO("Only allowed to have %d legs manually manipulated at one time.\n", MAX_MANUAL_LEGS);
        togglePrimaryLegState = false;
        toggleSecondaryLegState = false;
      }
    }
    else if (transitioning_leg->getLegState() == MANUAL)
    {
      ROS_INFO_COND(leg.leg_state_ == MANUAL, "%s leg transitioning to WALKING state . . .\n",
                    legNameMap[l * 2 + s].c_str());
      transitioning_leg->setLegState(MANUAL_TO_WALKING);
    }
    else if (transitioning_leg->getLegState() == WALKING_TO_MANUAL)
    {
      poseResetMode = IMMEDIATE_ALL_RESET;  // Set to ALL_RESET to force pose to new default pose
      //walker->tipPositions[l][s] = hexapod->local_tip_positions_[l][s];  // Override walker tip positions for manual
                                                                      // control
      //poser->tipPositions[l][s] = hexapod->local_tip_positions_[l][s];
      double res = poser->poseForLegManipulation(hexapod, transitioning_leg);

      if (params.dynamic_stiffness)
      {
        impedance->updateStiffness(res, l, s);
      }

      if (res == 1.0)
      {
        transitioning_leg->setLegState(MANUAL);
        ROS_INFO("%s leg set to state: MANUAL.\n", legNameMap[l * 2 + s].c_str());
        togglePrimaryLegState = false;
        toggleSecondaryLegState = false;
        poseResetMode = NO_RESET;
        manualLegs++;
      }
    }
    else if (transitioning_leg->getLegState() == MANUAL_TO_WALKING)
    {
      poseResetMode = IMMEDIATE_ALL_RESET;  // Set to ALL_RESET to force pose to new default pose
      //walker->tipPositions[l][s] = walker->identityTipPositions[l][s];  // Return walker tip positions to default
      double res = poser->poseForLegManipulation(hexapod, transitioning_leg);

      if (params.dynamic_stiffness)
      {
        impedance->updateStiffness(1.0 - res, l, s);
      }

      if (res == 1.0)
      {
        //walker->tipPositions[l][s] = walker->identityTipPositions[l][s];
        transitioning_leg->setLegState(WALKING);
        ROS_INFO("%s leg set to state: WALKING.\n", leg->getIDName().c_str());
        togglePrimaryLegState = false;
        toggleSecondaryLegState = false;
        poseResetMode = NO_RESET;
        manualLegs--;
      }
    }
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Stopping hexapod to transition leg state . . .\n");
    linearVelocityInput = Vector2d(0.0, 0.0);
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
      msg.local_tip_position.x = hexapod->local_tip_positions_[l][s][0];
      msg.local_tip_position.y = hexapod->local_tip_positions_[l][s][1];
      msg.local_tip_position.z = hexapod->local_tip_positions_[l][s][2];
      msg.poser_tip_positions.x = poser->tipPositions[l][s][0];
      msg.poser_tip_positions.y = poser->tipPositions[l][s][1];
      msg.poser_tip_positions.z = poser->tipPositions[l][s][2];
      msg.walker_tip_positions.x = walker->tipPositions[l][s][0];
      msg.walker_tip_positions.y = walker->tipPositions[l][s][1];
      msg.walker_tip_positions.z = walker->tipPositions[l][s][2];

      // Step progress
      msg.swing_progress.data = walker->legSteppers[l][s].swing_progress_;
      msg.stance_progress.data = walker->legSteppers[l][s].stance_progress_;

      // Impedance controller
      msg.tip_force.data = tipForce[l][s];
      msg.delta_z.data = deltaZ[l][s];
      msg.virtual_stiffness.data = impedance->virtualStiffness[l][s];

      legStatePublishers[l][s].publish(msg);

      // Publish leg state (ASC)
      std_msgs::Bool msg;
      if (walker->legSteppers[l][s].step_state_ == SWING ||
          (hexapod->legs_[l][s].leg_state_ != WALKING && hexapod->legs_[l][s].leg_state_ != MANUAL))
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
  msg.data.push_back(walker->current_linear_velocity_[0]);
  msg.data.push_back(walker->current_linear_velocity_[1]);
  msg.data.push_back(walker->current_angular_velocity_);
  bodyVelocityPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current pose (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishPose()
{
  geometry_msgs::Twist msg;
  msg.linear.x = poser->current_pose_.position_[0];
  msg.linear.y = poser->current_pose_.position_[1];
  msg.linear.z = poser->current_pose_.position_[2];
  msg.angular.x = poser->current_pose_.rotation_.toEulerAngles()[0];
  msg.angular.y = poser->current_pose_.rotation_.toEulerAngles()[1];
  msg.angular.z = poser->current_pose_.rotation_.toEulerAngles()[2];
  posePublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current rotation as per the IMU (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishIMUData()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(imuData.orientation.toEulerAngles()[0]);
  msg.data.push_back(imuData.orientation.toEulerAngles()[1]);
  msg.data.push_back(imuData.orientation.toEulerAngles()[2]);
  msg.data.push_back(imuData.linear_acceleration[0]);
  msg.data.push_back(imuData.linear_acceleration[1]);
  msg.data.push_back(imuData.linear_acceleration[2]);
  msg.data.push_back(imuData.angular_velocity[0]);
  msg.data.push_back(imuData.angular_velocity[1]);
  msg.data.push_back(imuData.angular_velocity[2]);
  IMUDataPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishRotationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser->rotationPositionError[0]);
  msg.data.push_back(poser->rotationPositionError[1]);
  msg.data.push_back(poser->rotationPositionError[2]);
  msg.data.push_back(poser->rotationAbsementError[0]);
  msg.data.push_back(poser->rotationAbsementError[1]);
  msg.data.push_back(poser->rotationAbsementError[2]);
  msg.data.push_back(poser->rotationVelocityError[0]);
  msg.data.push_back(poser->rotationVelocityError[1]);
  msg.data.push_back(poser->rotationVelocityError[2]);
  rotationPoseErrorPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishTranslationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser->translationPositionError[0]);
  msg.data.push_back(poser->translationPositionError[1]);
  msg.data.push_back(poser->translationPositionError[2]);
  msg.data.push_back(poser->translationAbsementError[0]);
  msg.data.push_back(poser->translationAbsementError[1]);
  msg.data.push_back(poser->translationAbsementError[2]);
  msg.data.push_back(poser->translationVelocityError[0]);
  msg.data.push_back(poser->translationVelocityError[1]);
  msg.data.push_back(poser->translationVelocityError[2]);
  translationPoseErrorPublisher.publish(msg);
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
      msg.data.push_back(impedance->zTipPositionError[l][s]);
      msg.data.push_back(impedance->zTipAbsementError[l][s]);
      msg.data.push_back(impedance->zTipVelocityError[l][s]);
    }
  }
  zTipErrorPublisher.publish(msg);
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
      debug.tip_positions_.insert(debug.tip_positions_.begin(),
                                walker->pose_.transformVector(hexapod->legs_[l][s].local_tip_position_));
      debug.static_tip_positions_.insert(debug.static_tip_positions_.begin(), hexapod->legs_[l][s].local_tip_position_);
    }
  }

  debug.drawRobot(hexapod->legs_[0][0].femur_offset_, hexapod->getJointPositions(walker->pose_), Vector4d(1, 1, 1, 1));
  debug.drawPoints(debug.tip_positions_, Vector4d(1, 0, 0, 1));  // Actual Tip Trajectory Paths
  // debug.drawPoints(debug.staticTipPositions, Vector4d(1,0,0,1)); //Static Single Tip Trajectory command

  if (debug.tip_positions_.size() > 2000)
  {
    debug.tip_positions_.erase(debug.tip_positions_.end() - 6, debug.tip_positions_.end());
  }
  if (debug.static_tip_positions_.size() >= 6 * (1 / (params.time_delta * walker->step_frequency_)))
  {
    debug.static_tip_positions_.clear();
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
      double yaw = dir * (hexapod->legs_[l][s].coxa_angle_ - params.physical_coxa_angle_offset[l]);
      double lift = dir * hexapod->legs_[l][s].femur_angle_;
      double knee = dir * hexapod->legs_[l][s].tibia_angle_ - dir * params.physical_tibia_angle_offset;

      double yawVel = 0;
      double liftVel = 0;
      double kneeVel = 0;

      if (firstIteration >= 6)  // First Iteration of ALL legs
      {
        yawVel = (yaw - hexapod->legs_[l][s].old_coxa_angle_) / params.time_delta;
        liftVel = (lift - hexapod->legs_[l][s].old_femur_angle) / params.time_delta;
        kneeVel = (knee - hexapod->legs_[l][s].old_tibia_angle_) / params.time_delta;

        if (abs(yawVel) > hexapod->joint_max_angular_speeds_[0])
        {
          ROS_WARN("%s_body_coxa joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   legNameMap[l * 2 + s].c_str(), yawVel, sign(yawVel) * hexapod->joint_max_angular_speeds_[0]);

          yawVel = sign(yawVel) * hexapod->joint_max_angular_speeds_[0];
          yaw = hexapod->legs_[l][s].old_coxa_angle_ + yawVel * params.time_delta;
        }
        if (abs(liftVel) > hexapod->joint_max_angular_speeds_[1])
        {
          ROS_WARN("%s_coxa_femour joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   legNameMap[l * 2 + s].c_str(), liftVel, sign(liftVel) * hexapod->joint_max_angular_speeds_[1]);

          liftVel = sign(liftVel) * hexapod->joint_max_angular_speeds_[1];
          lift = hexapod->legs_[l][s].old_femur_angle + liftVel * params.time_delta;
        }
        if (abs(kneeVel) > hexapod->joint_max_angular_speeds_[2])
        {
          ROS_WARN("%s_femour_tibia joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n",
                   legNameMap[l * 2 + s].c_str(), kneeVel, sign(kneeVel) * hexapod->joint_max_angular_speeds_[2]);

          kneeVel = sign(kneeVel) * hexapod->joint_max_angular_speeds_[2];
          knee = hexapod->legs_[l][s].old_tibia_angle_ + kneeVel * params.time_delta;
        }
      }
      else
      {
        firstIteration++;  // First Iteration of ALL legs
      }

      interface->setTargetAngle(l, s, 0, yaw);
      interface->setTargetAngle(l, s, 1, -lift);
      interface->setTargetAngle(l, s, 2, knee);

      if (!params.use_dynamixel_pro_interface)
      {
        interface->setVelocity(l, s, 0, yawVel);  // Doesn't cooperate with with Large Hexapod Dynamixel Pro drivers
      }
      interface->setVelocity(l, s, 1, -liftVel);
      interface->setVelocity(l, s, 2, kneeVel);

      hexapod->legs_[l][s].old_coxa_angle_ = yaw;
      hexapod->legs_[l][s].old_femur_angle = lift;
      hexapod->legs_[l][s].old_tibia_angle_ = knee;
    }
  }
  interface->publish();
}

/***********************************************************************************************************************
 * Input Body Velocity Topic Callback
***********************************************************************************************************************/
void StateController::bodyVelocityInputCallback(const geometry_msgs::Twist &input)
{
  linearVelocityInput = Vector2d(input.linear.x, input.linear.y);
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
void StateController::bodyPoseInputCallback(const geometry_msgs::Twist &twist)
{
  rollJoy = twist.angular.x;
  ;
  pitchJoy = twist.angular.y;
  yawJoy = twist.angular.z;
  xJoy = twist.linear.x;
  yJoy = twist.linear.y;
  zJoy = twist.linear.z;
}

/***********************************************************************************************************************
 * System state callback handling desired system state from hexapod_remote
***********************************************************************************************************************/
void StateController::systemStateCallback(const std_msgs::Int8 &input)
{
  if (!transitionSystemStateFlag)
  {
    newSystemState = static_cast<SystemState>(int(input.data));
    // If startUpSequence parameter is false then skip READY and PACKED states
    if (!params.start_up_sequence)
    {
      if (newSystemState == READY || newSystemState == PACKED)
      {
        newSystemState = OFF;
      }
    }

    if (newSystemState != systemState)
    {
      transitionSystemStateFlag = true;
    }
  }
}

/***********************************************************************************************************************
 * Gait Selection Callback
***********************************************************************************************************************/
void StateController::gaitSelectionCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    GaitDesignation newGaitSelection = static_cast<GaitDesignation>(int(input.data));
    if (newGaitSelection != gaitSelection && newGaitSelection != GAIT_UNDESIGNATED)
    {
      gaitSelection = newGaitSelection;
      gaitChangeFlag = true;
    }
  }
}

/***********************************************************************************************************************
 * Posing Mode Callback
***********************************************************************************************************************/
void StateController::posingModeCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    PosingMode newPosingMode = static_cast<PosingMode>(int(input.data));
    if (newPosingMode != posingMode)
    {
      posingMode = newPosingMode;
      switch (posingMode)
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
  if (systemState == RUNNING)
  {
    CruiseControlMode newCruiseControlMode = static_cast<CruiseControlMode>(int(input.data));
    if (newCruiseControlMode != cruiseControlMode)
    {
      cruiseControlMode = newCruiseControlMode;
      if (newCruiseControlMode == CRUISE_CONTROL_ON)
      {
        // Save current velocity input as cruise input
        linearCruiseVelocity = linearVelocityInput;
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
  if (systemState == RUNNING)
  {
    AutoNavigationMode newAutoNavigationMode = static_cast<AutoNavigationMode>(int(input.data));
    if (newAutoNavigationMode != autoNavigationMode)
    {
      autoNavigationMode = newAutoNavigationMode;
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
  if (systemState == RUNNING)
  {
    ParameterSelection newParameterSelection = static_cast<ParameterSelection>(int(input.data));
    if (newParameterSelection != parameterSelection)
    {
      paramScaler = -1;
      ROS_INFO("%s parameter currently selected.\n", parameterNameMap[newParameterSelection].c_str());
      parameterSelection = newParameterSelection;
    }
  }
}

/***********************************************************************************************************************
 * Parameter adjustment callback
***********************************************************************************************************************/
void StateController::parameterAdjustCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    if (input.data != 0.0 && !parameterAdjustFlag && parameterSelection != NO_PARAMETER_SELECTION)
    {
      if (paramScaler != -1)
      {
        paramScaler += input.data / paramAdjustSensitivity;
        paramScaler = minMax(paramScaler, 0.1, 3.0);  // Parameter scaler ranges from 10%->300%
      }
      parameterAdjustFlag = true;
    }
  }
}

/***********************************************************************************************************************
 * Pose reset mode callback
***********************************************************************************************************************/
void StateController::poseResetCallback(const std_msgs::Int8 &input)
{
  if (poseResetMode != IMMEDIATE_ALL_RESET)
  {
    poseResetMode = static_cast<PoseResetMode>(input.data);
  }
}

/***********************************************************************************************************************
 * Actuating Leg Primary Selection Callback
***********************************************************************************************************************/
void StateController::primaryLegSelectionCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    LegDesignation newPrimaryLegSelection = static_cast<LegDesignation>(input.data);
    if (primaryLegSelection != newPrimaryLegSelection)
    {
      if (newPrimaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("No leg currently selected for primary control.\n");
      }
      else
      {
        ROS_INFO("%s leg selected for primary control.\n", legNameMap[int(input.data)].c_str());
      }
      primaryLegSelection = newPrimaryLegSelection;
    }
  }
}

/***********************************************************************************************************************
 * Actuating Leg Secondary Selection Callback
***********************************************************************************************************************/
void StateController::secondaryLegSelectionCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    LegDesignation newSecondaryLegSelection = static_cast<LegDesignation>(input.data);
    if (secondaryLegSelection != newSecondaryLegSelection)
    {
      if (newSecondaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("No leg currently selected for secondary control.\n");
      }
      else
      {
        ROS_INFO("%s leg selected for secondary control.\n", legNameMap[int(input.data)].c_str());
      }
      secondaryLegSelection = newSecondaryLegSelection;
    }
  }
}

/***********************************************************************************************************************
 * Toggle Primary Leg State Callback
***********************************************************************************************************************/
void StateController::primaryLegStateCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    LegState newPrimaryLegState = static_cast<LegState>(int(input.data));
    if (newPrimaryLegState != primaryLegState)
    {
      if (primaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("Cannot toggle primary leg state as no leg is currently selected as primary.");
        ROS_INFO("Press left bumper to select a leg and try again.\n");
      }
      else if (toggleSecondaryLegState)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Cannot toggle primary leg state as secondary leg is currently "
                                           "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Please wait and try again.\n");
      }
      else
      {
        primaryLegState = newPrimaryLegState;
        togglePrimaryLegState = true;
      }
    }
  }
}

/***********************************************************************************************************************
 * Toggle Secondary Leg State Callback
***********************************************************************************************************************/
void StateController::secondaryLegStateCallback(const std_msgs::Int8 &input)
{
  if (systemState == RUNNING)
  {
    LegState newSecondaryLegState = static_cast<LegState>(int(input.data));
    if (newSecondaryLegState != secondaryLegState)
    {
      if (secondaryLegSelection == LEG_UNDESIGNATED)
      {
        ROS_INFO("Cannot toggle secondary leg state as no leg is currently selected as secondary.");
        ROS_INFO("Press left bumper to select a leg and try again.\n");
      }
      else if (togglePrimaryLegState)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Cannot toggle secondary leg state as secondary leg is currently "
                                           "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Please wait and try again.\n");
      }
      else
      {
        secondaryLegState = newSecondaryLegState;
        toggleSecondaryLegState = true;
      }
    }
  }
}

/***********************************************************************************************************************
 * IMU data callback
***********************************************************************************************************************/
void StateController::imuCallback(const sensor_msgs::Imu &data)
{
  Quat imuRotationOffset(params.imu_rotation_offset);

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
  imuData.orientation = (imuRotationOffset * rawOrientation) * imuRotationOffset.inverse();
  imuData.linear_acceleration = imuRotationOffset.toRotationMatrix() * rawLinearAcceleration;
  imuData.angular_velocity = imuRotationOffset.toRotationMatrix() * rawAngularVelocity;
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void StateController::jointStatesCallback(const sensor_msgs::JointState &jointStates)
{
  bool getEffortValues = (jointStates.effort.size() != 0);
  bool getVelocityValues = (jointStates.velocity.size() != 0);

  for (uint i = 0; i < jointStates.name.size(); i++)
  {
    const char *jointName = jointStates.name[i].c_str();
    if (!strcmp(jointName, "front_left_body_coxa") || !strcmp(jointName, "AL_coxa_joint"))
    {
      jointPositions[0] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[0] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[0] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_coxa_femour") || !strcmp(jointName, "AL_femur_joint"))
    {
      jointPositions[1] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[1] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[1] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_femour_tibia") || !strcmp(jointName, "AL_tibia_joint"))
    {
      jointPositions[2] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[2] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[2] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_body_coxa") || !strcmp(jointName, "AR_coxa_joint"))
    {
      jointPositions[3] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[3] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[3] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_coxa_femour") || !strcmp(jointName, "AR_femur_joint"))
    {
      jointPositions[4] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[4] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[4] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_femour_tibia") || !strcmp(jointName, "AR_tibia_joint"))
    {
      jointPositions[5] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[5] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[5] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_body_coxa") || !strcmp(jointName, "BL_coxa_joint"))
    {
      jointPositions[6] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[6] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[6] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_coxa_femour") || !strcmp(jointName, "BL_femur_joint"))
    {
      jointPositions[7] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[7] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[7] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_femour_tibia") || !strcmp(jointName, "BL_tibia_joint"))
    {
      jointPositions[8] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[8] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[8] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_body_coxa") || !strcmp(jointName, "BR_coxa_joint"))
    {
      jointPositions[9] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[9] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[9] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_coxa_femour") || !strcmp(jointName, "BR_femur_joint"))
    {
      jointPositions[10] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[10] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[10] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_femour_tibia") || !strcmp(jointName, "BR_tibia_joint"))
    {
      jointPositions[11] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[11] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[11] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_body_coxa") || !strcmp(jointName, "CL_coxa_joint"))
    {
      jointPositions[12] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[12] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[12] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_coxa_femour") || !strcmp(jointName, "CL_femur_joint"))
    {
      jointPositions[13] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[13] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[13] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_femour_tibia") || !strcmp(jointName, "CL_tibia_joint"))
    {
      jointPositions[14] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[14] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[14] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_body_coxa") || !strcmp(jointName, "CR_coxa_joint"))
    {
      jointPositions[15] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[15] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[15] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_coxa_femour") || !strcmp(jointName, "CR_femur_joint"))
    {
      jointPositions[16] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[16] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[16] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_femour_tibia") || !strcmp(jointName, "CR_tibia_joint"))
    {
      jointPositions[17] = jointStates.position[i];
      if (getVelocityValues)
      {
        jointVelocities[17] = jointStates.velocity[i];
      }
      if (getEffortValues)
      {
        jointEfforts[17] = jointStates.effort[i];
      }
    }

    // Check if all joint positions have been received from topic
    recievedAllJointPositions = true;
    for (int i = 0; i < 18; i++)
    {
      if (jointPositions[i] > 1e9)
      {
        recievedAllJointPositions = false;
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
  std::string baseParamString = "/hexapod/parameters/";
  std::string paramString;
  std::string forceGait;

  // Gait Parameters
  getGaitParameters(GAIT_UNDESIGNATED);

  // Hexapod Parameters
  if (!n.getParam(baseParamString + "hexapod_type", params.hexapod_type))
  {
    ROS_ERROR("Error reading parameter/s (hexapod_type) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(baseParamString + "time_delta", params.time_delta))
  {
    ROS_ERROR("Error reading parameter/s (time_delta) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(baseParamString + "imu_compensation", params.imu_compensation))
  {
    ROS_ERROR("Error reading parameter/s (imu_compensation) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(baseParamString + "auto_compensation", params.auto_compensation))
  {
    ROS_ERROR("Error reading parameter/s (auto_compensation) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(baseParamString + "manual_compensation", params.manual_compensation))
  {
    ROS_ERROR("Error reading parameter/s (manual_compensation) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(baseParamString + "inclination_compensation", params.inclination_compensation))
  {
    ROS_ERROR("Error reading parameter/s (inclination_compensation) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }

  /********************************************************************************************************************/
  // Offset Parameters
  // Root Offset Parameters
  std::vector<double> rootOffsetAL(3);
  paramString = baseParamString + "/physical_leg_offsets/";
  if (!n.getParam(paramString + "root_offset_AL", rootOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_AL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[0][0] = Map<Vector3d>(&rootOffsetAL[0], 3);
  }

  std::vector<double> rootOffsetAR(3);
  if (!n.getParam(paramString + "root_offset_AR", rootOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_AR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[0][1] = Map<Vector3d>(&rootOffsetAR[0], 3);
  }

  std::vector<double> rootOffsetBL(3);
  if (!n.getParam(paramString + "root_offset_BL", rootOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_BL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[1][0] = Map<Vector3d>(&rootOffsetBL[0], 3);
  }

  std::vector<double> rootOffsetBR(3);
  if (!n.getParam(paramString + "root_offset_BR", rootOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_BR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[1][1] = Map<Vector3d>(&rootOffsetBR[0], 3);
  }

  std::vector<double> rootOffsetCL(3);
  if (!n.getParam(paramString + "root_offset_CL", rootOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_CL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[2][0] = Map<Vector3d>(&rootOffsetCL[0], 3);
  }

  std::vector<double> rootOffsetCR(3);
  if (!n.getParam(paramString + "root_offset_CR", rootOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_CR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_offset[2][1] = Map<Vector3d>(&rootOffsetCR[0], 3);
  }

  /********************************************************************************************************************/
  // Hip Offset Parameters
  std::vector<double> hipOffsetAL(3);
  if (!n.getParam(paramString + "hip_offset_AL", hipOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_AL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[0][0] = Map<Vector3d>(&hipOffsetAL[0], 3);
  }

  std::vector<double> hipOffsetAR(3);
  if (!n.getParam(paramString + "hip_offset_AR", hipOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_AR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[0][1] = Map<Vector3d>(&hipOffsetAR[0], 3);
  }

  std::vector<double> hipOffsetBL(3);
  if (!n.getParam(paramString + "hip_offset_BL", hipOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_BL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[1][0] = Map<Vector3d>(&hipOffsetBL[0], 3);
  }

  std::vector<double> hipOffsetBR(3);
  if (!n.getParam(paramString + "hip_offset_BR", hipOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_BR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[1][1] = Map<Vector3d>(&hipOffsetBR[0], 3);
  }

  std::vector<double> hipOffsetCL(3);
  if (!n.getParam(paramString + "hip_offset_CL", hipOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_CL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[2][0] = Map<Vector3d>(&hipOffsetCL[0], 3);
  }

  std::vector<double> hipOffsetCR(3);
  if (!n.getParam(paramString + "hip_offset_CR", hipOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_CR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_offset[2][1] = Map<Vector3d>(&hipOffsetCR[0], 3);
  }

  /********************************************************************************************************************/
  // Knee Offset Parameters
  std::vector<double> kneeOffsetAL(3);
  if (!n.getParam(paramString + "knee_offset_AL", kneeOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_AL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[0][0] = Map<Vector3d>(&kneeOffsetAL[0], 3);
  }

  std::vector<double> kneeOffsetAR(3);
  if (!n.getParam(paramString + "knee_offset_AR", kneeOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_AR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[0][1] = Map<Vector3d>(&kneeOffsetAR[0], 3);
  }

  std::vector<double> kneeOffsetBL(3);
  if (!n.getParam(paramString + "knee_offset_BL", kneeOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_BL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[1][0] = Map<Vector3d>(&kneeOffsetBL[0], 3);
  }

  std::vector<double> kneeOffsetBR(3);
  if (!n.getParam(paramString + "knee_offset_BR", kneeOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_BR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[1][1] = Map<Vector3d>(&kneeOffsetBR[0], 3);
  }

  std::vector<double> kneeOffsetCL(3);
  if (!n.getParam(paramString + "knee_offset_CL", kneeOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_CL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[2][0] = Map<Vector3d>(&kneeOffsetCL[0], 3);
  }

  std::vector<double> kneeOffsetCR(3);
  if (!n.getParam(paramString + "knee_offset_CR", kneeOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_CR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_offset[2][1] = Map<Vector3d>(&kneeOffsetCR[0], 3);
  }

  /********************************************************************************************************************/
  // Tip Offset Parameters
  std::vector<double> tipOffsetAL(3);
  if (!n.getParam(paramString + "tip_offset_AL", tipOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_AL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[0][0] = Map<Vector3d>(&tipOffsetAL[0], 3);
  }

  std::vector<double> tipOffsetAR(3);
  if (!n.getParam(paramString + "tip_offset_AR", tipOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_AR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[0][1] = Map<Vector3d>(&tipOffsetAR[0], 3);
  }

  std::vector<double> tipOffsetBL(3);
  if (!n.getParam(paramString + "tip_offset_BL", tipOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_BL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[1][0] = Map<Vector3d>(&tipOffsetBL[0], 3);
  }

  std::vector<double> tipOffsetBR(3);
  if (!n.getParam(paramString + "tip_offset_BR", tipOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_BR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[1][1] = Map<Vector3d>(&tipOffsetBR[0], 3);
  }

  std::vector<double> tipOffsetCL(3);
  if (!n.getParam(paramString + "tip_offset_CL", tipOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_CL) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[2][0] = Map<Vector3d>(&tipOffsetCL[0], 3);
  }

  std::vector<double> tipOffsetCR(3);
  if (!n.getParam(paramString + "tip_offset_CR", tipOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_CR) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tip_offset[2][1] = Map<Vector3d>(&tipOffsetCR[0], 3);
  }

  /********************************************************************************************************************/
  // Yaw parameters
  std::vector<double> stanceLegYaws(3);
  if (!n.getParam(baseParamString + "stance_leg_yaws", stanceLegYaws))
  {
    ROS_ERROR("Error reading parameter/s (stance_leg_yaws) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.stance_coxa_angles = Map<Vector3d>(&stanceLegYaws[0], 3);
  }

  std::vector<double> physicalYawOffset(3);
  if (!n.getParam(baseParamString + "physical_yaw_offset", physicalYawOffset))
  {
    ROS_ERROR("Error reading parameter/s (physical_yaw_offset) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.physical_coxa_angle_offset = Map<Vector3d>(&physicalYawOffset[0], 3);
  }

  // Knee Offset parameter
  if (!n.getParam(baseParamString + "physical_knee_offset", params.physical_tibia_angle_offset))
  {
    ROS_ERROR("Error reading parameter/s (physical_knee_offset) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  /********************************************************************************************************************/
  // Joint Limit Parameters
  paramString = baseParamString + "/joint_limits/";
  std::vector<double> yawLimits(3);
  if (!n.getParam(paramString + "yaw_limits", yawLimits))
  {
    ROS_ERROR("Error reading parameter/s (yaw_limits) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.coxa_joint_limits = Map<Vector3d>(&yawLimits[0], 3);
  }

  std::vector<double> kneeLimits(2);
  if (!n.getParam(paramString + "knee_limits", kneeLimits))
  {
    ROS_ERROR("Error reading parameter/s (knee_limits) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.tibia_joint_limits = Map<Vector2d>(&kneeLimits[0], 2);
  }

  std::vector<double> hipLimits(2);
  if (!n.getParam(paramString + "hip_limits", hipLimits))
  {
    ROS_ERROR("Error reading parameter/s (hip_limits) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.femur_joint_limits = Map<Vector2d>(&hipLimits[0], 2);
  }

  std::vector<double> jointMaxAngularSpeeds(2);
  if (!n.getParam(baseParamString + "joint_max_angular_speeds", jointMaxAngularSpeeds))
  {
    ROS_ERROR("Error reading parameter/s (joint_max_angular_speed) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }
  else
  {
    params.joint_max_angular_speeds = Map<Vector3d>(&jointMaxAngularSpeeds[0], 2);
  }

  if (!n.getParam(baseParamString + "dynamixel_interface", params.use_dynamixel_pro_interface))
  {
    ROS_ERROR("Error reading parameter/s (dynamixel_interface) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  std::vector<double> imuRotationOffset(3);
  if (!n.getParam(baseParamString + "imu_rotation_offset", imuRotationOffset))
  {
    ROS_ERROR("Error reading parameter/s (imu_rotation_offset) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.imu_rotation_offset = Map<Vector3d>(&imuRotationOffset[0], 3);
  }

  /********************************************************************************************************************/
  // Walk Controller Parameters
  paramString = baseParamString + "/walk_controller/";

  if (!n.getParam(paramString + "step_frequency", params.step_frequency))
  {
    ROS_ERROR("Error reading parameter/s (step_frequency) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "step_clearance", params.step_clearance))
  {
    ROS_ERROR("Error reading parameter/s (step_clearance) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "step_depth", params.step_depth))
  {
    ROS_ERROR("Error reading parameter/s (step_depth) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "body_clearance", params.body_clearance))
  {
    ROS_ERROR("Error reading parameter/s (body_clearance) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "leg_span_scale", params.leg_span_scale))
  {
    ROS_ERROR("Error reading parameter/s (leg_span_scale) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "max_linear_acceleration", params.max_linear_acceleration))
  {
    ROS_ERROR("Error reading parameter/s (max_linear_acceleration) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "max_angular_acceleration", params.max_angular_acceleration))
  {
    ROS_ERROR("Error reading parameter/s (max_angular_acceleration) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }

  if (!n.getParam(paramString + "footprint_downscale", params.footprint_downscale))
  {
    ROS_ERROR("Error reading parameter/s (footprint_downscale) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "interface_setup_speed", params.interface_setup_speed))
  {
    ROS_ERROR("Error reading parameter/s (interface_setup_speed) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "velocity_input_mode", params.velocity_input_mode))
  {
    ROS_ERROR("Error reading parameter/s (velocity_input_mode) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "force_cruise_velocity", params.force_cruise_velocity))
  {
    ROS_ERROR("Error reading parameter/s (force_cruise_velocity) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  std::vector<double> linearCruiseVelocity(2);
  if (!n.getParam(paramString + "linear_cruise_velocity", linearCruiseVelocity))
  {
    ROS_ERROR("Error reading debug parameter/s (linear_cruise_velocity) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.linear_cruise_velocity = Map<Vector2d>(&linearCruiseVelocity[0], 2);
  }

  if (!n.getParam(paramString + "angular_cruise_velocity", params.angular_cruise_velocity))
  {
    ROS_ERROR("Error reading debug parameter/s (angular_cruise_velocity) from rosparam. Check config file is loaded "
              "and type is correct\n");
  }

  /********************************************************************************************************************/
  // Pose Controller Parameters
  paramString = baseParamString + "/pose_controller/";

  if (!n.getParam(paramString + "start_up_sequence", params.start_up_sequence))
  {
    ROS_ERROR("Error reading parameter/s (start_up_sequence) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "move_legs_sequentially", params.move_legs_sequentially))
  {
    ROS_ERROR("Error reading parameter/s (move_legs_sequentially) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "time_to_start", params.time_to_start))
  {
    ROS_ERROR("Error reading parameter/s (time_to_start) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  // IMU compensation parameters
  // Translation compensation
  paramString = baseParamString + "/pose_controller/imu_pose_compensation/translation_compensation/";
  if (!n.getParam(paramString + "proportional_gain", params.translation_compensation_proportional_gain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/proportional_gain) from rosparam. Check config file "
              "is loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "integral_gain", params.translation_compensation_integral_gain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/integral_gain) from rosparam. Check config file is "
              "loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "derivative_gain", params.translation_compensation_derivative_gain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/derivative_gain) from rosparam. Check config file "
              "is loaded and type is correct\n");
  }
  // Rotation Compensation
  paramString = baseParamString + "/pose_controller/imu_pose_compensation/rotation_compensation/";
  if (!n.getParam(paramString + "proportional_gain", params.rotation_compensation_proportional_gain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/proportional_gain) from rosparam. Check config file is "
              "loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "integral_gain", params.rotation_compensation_integral_gain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/integral_gain) from rosparam. Check config file is "
              "loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "derivative_gain", params.rotation_compensation_derivative_gain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/derivative_gain) from rosparam. Check config file is "
              "loaded and type is correct\n");
  }

  // Auto compensation parameters
  paramString = baseParamString + "/pose_controller/auto_pose_compensation/";
  if (!n.getParam(paramString + "pitch_amplitude", params.pitch_amplitude))
  {
    ROS_ERROR("Error reading parameter/s (pitch_amplitude) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "roll_amplitude", params.roll_amplitude))
  {
    ROS_ERROR("Error reading parameter/s (roll_amplitude) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "z_trans_amplitude", params.z_translation_amplitude))
  {
    ROS_ERROR("Error reading parameter/s (z_trans_amplitude) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  // Manual compensation parameters
  paramString = baseParamString + "/pose_controller/manual_pose_compensation/";
  std::vector<double> maxTranslation(3);
  if (!n.getParam(paramString + "max_translation", maxTranslation))
  {
    ROS_ERROR("Error reading parameter/s (max_translation) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.max_translation = Map<Vector3d>(&maxTranslation[0], 3);
  }

  std::vector<double> maxRotation(3);
  if (!n.getParam(paramString + "max_rotation", maxRotation))
  {
    ROS_ERROR("Error reading parameter/s (max_rotation) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
  else
  {
    params.max_rotation = Map<Vector3d>(&maxRotation[0], 3);
  }

  if (!n.getParam(paramString + "max_translation_velocity", params.max_translation_velocity))
  {
    ROS_ERROR("Error reading parameter/s (max_translation_velocity) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }

  if (!n.getParam(paramString + "max_rotation_velocity", params.max_rotation_velocity))
  {
    ROS_ERROR("Error reading parameter/s (max_rotation_velocity) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  paramString = baseParamString + "pose_controller/manual_leg_manipulation/";
  if (!n.getParam(paramString + "leg_manipulation_mode", params.leg_manipulation_mode))
  {
    ROS_ERROR("Error reading parameter/s (leg_manipulation_mode) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  /********************************************************************************************************************/

  paramString = baseParamString + "/pose_controller/packed_joint_positions/";
  std::vector<double> packedJointPositionsAL(3);
  if (!n.getParam(paramString + "AL_packed_joint_positions", packedJointPositionsAL))
  {
    ROS_ERROR("Error reading parameter/s (AL_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_AL = Map<Vector3d>(&packedJointPositionsAL[0], 3);
  }

  std::vector<double> packedJointPositionsAR(3);
  if (!n.getParam(paramString + "AR_packed_joint_positions", packedJointPositionsAR))
  {
    ROS_ERROR("Error reading parameter/s (AR_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_AR = Map<Vector3d>(&packedJointPositionsAR[0], 3);
  }

  std::vector<double> packedJointPositionsBL(3);
  if (!n.getParam(paramString + "BL_packed_joint_positions", packedJointPositionsBL))
  {
    ROS_ERROR("Error reading parameter/s (BL_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_BL = Map<Vector3d>(&packedJointPositionsBL[0], 3);
  }

  std::vector<double> packedJointPositionsBR(3);
  if (!n.getParam(paramString + "BR_packed_joint_positions", packedJointPositionsBR))
  {
    ROS_ERROR("Error reading parameter/s (BR_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_BR = Map<Vector3d>(&packedJointPositionsBR[0], 3);
  }

  std::vector<double> packedJointPositionsCL(3);
  if (!n.getParam(paramString + "CL_packed_joint_positions", packedJointPositionsCL))
  {
    ROS_ERROR("Error reading parameter/s (CL_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_CL = Map<Vector3d>(&packedJointPositionsCL[0], 3);
  }

  std::vector<double> packedJointPositionsCR(3);
  if (!n.getParam(paramString + "CR_packed_joint_positions", packedJointPositionsCR))
  {
    ROS_ERROR("Error reading parameter/s (CR_packed_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.packed_joint_positions_CR = Map<Vector3d>(&packedJointPositionsCR[0], 3);
  }

  /********************************************************************************************************************/
  paramString = baseParamString + "/pose_controller/unpacked_joint_positions/";
  std::vector<double> unpackedJointPositionsAL(3);
  if (!n.getParam(paramString + "AL_unpacked_joint_positions", unpackedJointPositionsAL))
  {
    ROS_ERROR("Error reading parameter/s (AL_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_AL = Map<Vector3d>(&unpackedJointPositionsAL[0], 3);
  }

  std::vector<double> unpackedJointPositionsAR(3);
  if (!n.getParam(paramString + "AR_unpacked_joint_positions", unpackedJointPositionsAR))
  {
    ROS_ERROR("Error reading parameter/s (AR_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_AR = Map<Vector3d>(&unpackedJointPositionsAR[0], 3);
  }

  std::vector<double> unpackedJointPositionsBL(3);
  if (!n.getParam(paramString + "BL_unpacked_joint_positions", unpackedJointPositionsBL))
  {
    ROS_ERROR("Error reading parameter/s (BL_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_BL = Map<Vector3d>(&unpackedJointPositionsBL[0], 3);
  }

  std::vector<double> unpackedJointPositionsBR(3);
  if (!n.getParam(paramString + "BR_unpacked_joint_positions", unpackedJointPositionsBR))
  {
    ROS_ERROR("Error reading parameter/s (BR_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_BR = Map<Vector3d>(&unpackedJointPositionsBR[0], 3);
  }

  std::vector<double> unpackedJointPositionsCL(3);
  if (!n.getParam(paramString + "CL_unpacked_joint_positions", unpackedJointPositionsCL))
  {
    ROS_ERROR("Error reading parameter/s (CL_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_CL = Map<Vector3d>(&unpackedJointPositionsCL[0], 3);
  }

  std::vector<double> unpackedJointPositionsCR(3);
  if (!n.getParam(paramString + "CR_unpacked_joint_positions", unpackedJointPositionsCR))
  {
    ROS_ERROR("Error reading parameter/s (CR_unpacked_joint_positions) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }
  else
  {
    params.unpacked_joint_positions_CR = Map<Vector3d>(&unpackedJointPositionsCR[0], 3);
  }

  /********************************************************************************************************************/
  // Impedance Control Parameters

  paramString = baseParamString + "/impedance_controller/";

  if (!n.getParam(paramString + "impedance_control", params.impedance_control))
  {
    ROS_ERROR("Error reading parameter/s (impedance_control) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "dynamic_stiffness", params.dynamic_stiffness))
  {
    ROS_ERROR("Error reading parameter/s (dynamic_stiffness) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "integrator_step_time", params.integrator_step_time))
  {
    ROS_ERROR("Error reading parameter/s (integrator_step_time) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "virtual_mass", params.virtual_mass))
  {
    ROS_ERROR("Error reading parameter/s (virtual_mass) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "virtual_stiffness", params.virtual_stiffness))
  {
    ROS_ERROR("Error reading parameter/s (virtual_stiffness) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "/load_stiffness_scaler", params.load_stiffness_scaler))
  {
    ROS_ERROR("Error reading parameter/s (load_stiffness_scaler) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "/swing_stiffness_scaler", params.swing_stiffness_scaler))
  {
    ROS_ERROR("Error reading parameter/s (swing_stiffness_scaler) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "virtual_damping_ratio", params.virtual_damping_ratio))
  {
    ROS_ERROR("Error reading parameter/s (virtual_damping_ratio) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "force_gain", params.force_gain))
  {
    ROS_ERROR("Error reading parameter/s (force_gain) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  if (!n.getParam(paramString + "impedance_input", params.impedance_input))
  {
    ROS_ERROR("Error reading parameter/s (impedance_input) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  /********************************************************************************************************************/
  // Debug Parameters

  paramString = "/hexapod/debug_parameters/";

  if (!n.getParam(paramString + "console_verbosity", params.console_verbosity))
  {
    ROS_ERROR("Error reading debug parameter/s (console_verbosity) from rosparam. Check config file is loaded and type "
              "is correct\n");
  }

  if (!n.getParam(paramString + "debug_move_to_joint_position", params.debug_moveToJointPosition))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_move_to_joint_position) from rosparam. Check config file is "
              "loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "debug_step_to_position", params.debug_stepToPosition))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_step_to_position) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }

  if (!n.getParam(paramString + "debug_swing_trajectory", params.debug_swing_trajectory))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_swing_trajectory) from rosparam. Check config file is loaded and "
              "type is correct\n");
  }

  if (!n.getParam(paramString + "debug_stance_trajectory", params.debug_stance_trajectory))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_stance_trajectory) from rosparam. Check config file is loaded "
              "and type is correct\n");
  }

  if (!n.getParam(paramString + "debug_manual_compensation_rotation", params.debug_manual_compensation_rotation))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_manual_compensation_rotation) from rosparam. Check config file "
              "is loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "debug_manual_compensation_translation", params.debug_manual_compensation_translation))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_manual_compensation_translation) from rosparam. Check config "
              "file is loaded and type is correct\n");
  }

  if (!n.getParam(paramString + "debug_rviz", params.debug_rviz))
  {
    ROS_ERROR("Error reading debug parameter/s (rviz) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  /********************************************************************************************************************/
  // Hexapod remote parameters (set via hexapod_remote launch file instead of by config files)
  paramString = "/hexapod_remote/param_adjust_sensitivity";
  if (!n.getParam(paramString, paramAdjustSensitivity))
  {
    ROS_ERROR("Error reading parameter/s (param_adjust_sensitivity) from rosparam. Check hexapod_remote is running and "
              "launch file properly set param\n");
    paramAdjustSensitivity = 10.0;  // Default to 10.0 (as per default in hexapod_remote)
  }
}

/***********************************************************************************************************************
 * Gets gait parameters from rosparam server
***********************************************************************************************************************/
void StateController::getGaitParameters(GaitDesignation gaitSelection)
{
  std::string baseParamString = "/hexapod/parameters/";
  std::string paramString;

  switch (gaitSelection)
  {
    case (TRIPOD_GAIT):
      params.gait_type = "tripod_gait";
      break;
    case (RIPPLE_GAIT):
      params.gait_type = "ripple_gait";
      break;
    case (WAVE_GAIT):
      params.gait_type = "wave_gait";
      break;
    case (AMBLE_GAIT):
      params.gait_type = "amble_gait";
      break;
    case (GAIT_UNDESIGNATED):
      if (!n.getParam(baseParamString + "gait_type", params.gait_type))
      {
        ROS_ERROR("Error reading parameter/s (gaitType) from rosparam. Check config file is loaded and type is "
                  "correct\n");
      }
      break;
  }

  baseParamString = "/hexapod/gait_parameters/";

  paramString = baseParamString + params.gait_type + "/stance_phase";
  if (!n.getParam(paramString, params.stance_phase))
  {
    ROS_ERROR("Error reading parameter/s (stance_phase) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  paramString = baseParamString + params.gait_type + "/swing_phase";
  if (!n.getParam(paramString, params.swing_phase))
  {
    ROS_ERROR("Error reading parameter/s (swing_phase) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  paramString = baseParamString + params.gait_type + "/phase_offset";
  if (!n.getParam(paramString, params.phase_offset))
  {
    ROS_ERROR("Error reading parameter/s (phase_offset) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }

  paramString = baseParamString + params.gait_type + "/offset_multiplier";
  if (!n.getParam(paramString, params.offset_multiplier))
  {
    ROS_ERROR("Error reading parameter/s (offset_multiplier) from rosparam. Check config file is loaded and type is "
              "correct\n");
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/
