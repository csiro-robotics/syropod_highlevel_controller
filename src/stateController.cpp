#include "simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * State controller contructor
***********************************************************************************************************************/
StateController::StateController(ros::NodeHandle n) : n_(n)
{
  // Get parameters from parameter server and initialises parameter map
  initParameters();

  // Initiate model and imu objects
  model_ = new Model(&params_);

  // Hexapod Remote topic subscriptions
  desired_velocity_subscriber_ = n_.subscribe("hexapod_remote/desired_velocity", 1, &StateController::bodyVelocityInputCallback, this);
  primary_tip_velocity_subscriber_ = n_.subscribe("hexapod_remote/primary_tip_velocity", 1, &StateController::primaryTipVelocityInputCallback, this);
  secondary_tip_velocity_subscriber_ = n_.subscribe("hexapod_remote/secondary_tip_velocity", 1, &StateController::secondaryTipVelocityInputCallback, this);
  desired_pose_subscriber_ = n_.subscribe("hexapod_remote/desired_pose", 1, &StateController::bodyPoseInputCallback, this);
  system_state_subscriber_ = n_.subscribe("hexapod_remote/system_state", 1, &StateController::systemStateCallback, this);
  robot_state_subscriber_ = n_.subscribe("hexapod_remote/robot_state", 1, &StateController::robotStateCallback, this);
  gait_selection_subscriber_ = n_.subscribe("hexapod_remote/gait_selection", 1, &StateController::gaitSelectionCallback, this);
  posing_mode_subscriber_ = n_.subscribe("hexapod_remote/posing_mode", 1, &StateController::posingModeCallback, this);
  cruise_control_mode_subscriber_ = n_.subscribe("hexapod_remote/cruise_control_mode", 1, &StateController::cruiseControlCallback, this);
  auto_navigation_mode_subscriber_ = n_.subscribe("hexapod_remote/auto_navigation_mode", 1, &StateController::autoNavigationCallback, this);
  parameter_selection_subscriber_ = n_.subscribe("hexapod_remote/parameter_selection", 1, &StateController::parameterSelectionCallback, this);
  parameter_adjustment_subscriber_ = n_.subscribe("hexapod_remote/parameter_adjustment", 1, &StateController::parameterAdjustCallback, this);
  primary_leg_selection_subscriber_ = n_.subscribe("hexapod_remote/primary_leg_selection", 1, &StateController::primaryLegSelectionCallback, this);
  primary_leg_state_subscriber_ = n_.subscribe("hexapod_remote/primary_leg_state", 1, &StateController::primaryLegStateCallback, this);
  secondary_leg_selection_subscriber_ = n_.subscribe("hexapod_remote/secondary_leg_selection", 1, &StateController::secondaryLegSelectionCallback, this);
  secondary_leg_state_subscriber_ = n_.subscribe("hexapod_remote/secondary_leg_state", 1, &StateController::secondaryLegStateCallback, this);
  pose_reset_mode_subscriber_ = n_.subscribe("hexapod_remote/pose_reset_mode", 1, &StateController::poseResetCallback, this);

  // Motor and other sensor topic subscriptions
  imu_data_subscriber_ = n_.subscribe("ig/imu/data_ned", 1, &StateController::imuCallback, this);
  joint_state_subscriber_ = n_.subscribe("/joint_states", 1000, &StateController::jointStatesCallback, this);
  tip_force_subscriber_ = n_.subscribe("/motor_encoders", 1, &StateController::tipForceCallback, this); //TBD New topic?
  tip_force_subscriber_AR_ = n_.subscribe("/AR_prs", 1, &StateController::tipForceCallbackAR, this); //TBD 
  tip_force_subscriber_BR_ = n_.subscribe("/BR_prs", 1, &StateController::tipForceCallbackBR, this);
  tip_force_subscriber_CR_ = n_.subscribe("/CR_prs", 1, &StateController::tipForceCallbackCR, this);
  tip_force_subscriber_CL_ = n_.subscribe("/CL_prs", 1, &StateController::tipForceCallbackCL, this);
  tip_force_subscriber_BL_ = n_.subscribe("/BL_prs", 1, &StateController::tipForceCallbackBL, this);
  tip_force_subscriber_AL_ = n_.subscribe("/AL_prs", 1, &StateController::tipForceCallbackAL, this);

  //Set up debugging publishers
  string node_name = ros::this_node::getName();
  pose_publisher_ = n_.advertise<geometry_msgs::Twist>(node_name + "/pose", 1000);
  imu_data_publisher_ = n_.advertise<Float32MultiArray>(node_name + "/imu_data", 1000);
  body_velocity_publisher_ = n_.advertise<Float32MultiArray>(node_name + "/body_velocity", 1000);
  rotation_pose_error_publisher_ = n_.advertise<Float32MultiArray>(node_name + "/rotation_pose_error", 1000);
  translation_pose_error_publisher_ = n_.advertise<Float32MultiArray>(node_name + "/translation_pose_error", 1000);

  // Set up leg state publishers within leg objects
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    string topic_name = node_name + "/" + leg->getIDName() + "/state";
    leg->setStatePublisher(n_.advertise<simple_hexapod_controller::legState>(topic_name, 1000));
    leg->setASCStatePublisher(n_.advertise<std_msgs::Bool>("/leg_state_" + leg->getIDName() + "_bool", 1)); //TBD Remove

    // If debugging in sagzebo, setup joint command publishers
    if (params_.debug_gazebo.data)
    {
      for (joint_it_ = leg->getJointContainer()->begin(); joint_it_ != leg->getJointContainer()->end(); ++joint_it_)
      {
        Joint* joint = joint_it_->second;
        string joint_name = joint->name;
        joint->gazebo_publisher = n_.advertise<std_msgs::Float64>("/hexapod/" + joint_name + "/command", 1000);
      }
    }
  }
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
  interface_ = new DynamixelMotorInterface(model_);
  interface_->setupSpeed(params_.interface_setup_speed.data); //TBD needed?

  // Set initial gait selection number for gait toggling
  if (params_.gait_type.data == "tripod_gait")
  {
    gait_selection_ = TRIPOD_GAIT;
  }
  else if (params_.gait_type.data == "ripple_gait")
  {
    gait_selection_ = RIPPLE_GAIT;
  }
  else if (params_.gait_type.data == "wave_gait")
  {
    gait_selection_ = WAVE_GAIT;
  }
  else if (params_.gait_type.data == "amble_gait")
  {
    gait_selection_ = AMBLE_GAIT;
  }

  // Create controller objects
  walker_ = new WalkController(model_, &params_);
  poser_ = new PoseController(model_, &params_);
  impedance_ = new ImpedanceController(model_, &params_);

  robot_state_ = UNKNOWN;

  initialised_ = true;
}

/***********************************************************************************************************************
 * State machine loop
***********************************************************************************************************************/
void StateController::loop()
{
  // Compensation - updates currentPose for body compensation
  if (robot_state_ != UNKNOWN)
  {
    poser_->updateCurrentPose(walker_->getBodyHeight(), walker_->getWalkState());
    walker_->setPoseState(poser_->getAutoPoseState()); // Sends pose state from poser to walker

    // Impedance control - updates deltaZ values
    if (params_.impedance_control.data)
    {
      impedanceControl();
    }
  }

  // Hexapod state machine
  if (transition_state_flag_)
  {
    transitionSystemState();
  }
  else if (robot_state_ == RUNNING)
  {
    runningState();
  }
}

/***********************************************************************************************************************
 * Impedance Control
***********************************************************************************************************************/
void StateController::impedanceControl() //TBD
{
  // Calculate new stiffness based on walking cycle
  if (walker_->getWalkState() != STOPPED && params_.dynamic_stiffness.data)
  {
    impedance_->updateStiffness(walker_);
  }
  // Get current force value on leg and run impedance calculations to get a vertical tip offset (deltaZ)
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    impedance_->updateImpedance(leg, params_.use_joint_effort.data);
  }
}

/***********************************************************************************************************************
 * State transition handler
***********************************************************************************************************************/
void StateController::transitionSystemState()
{
  // UNKNOWN -> OFF/PACKED/READY/RUNNING
  if (robot_state_ == UNKNOWN)
  {
    // Check how many joints/legs are in the packed state
    int legs_packed = 0;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg* leg = leg_it_->second;
      int joints_packed = 0;
      for (joint_it_ = leg->getJointContainer()->begin(); joint_it_ != leg->getJointContainer()->end(); ++joint_it_)
      {
        Joint* joint = joint_it_->second;
        double joint_tolerance = 0.01; //TBD Magic number
        joints_packed += int(abs(joint->current_position - joint->packed_position) < joint_tolerance);
      }
      legs_packed += int(joints_packed == leg->getNumJoints());
    }

    // Robot estimated to be in packed state
    if (legs_packed == model_->getLegCount())
    {
      if (!params_.start_up_sequence.data)
      {
        ROS_FATAL("\nHexapod currently in packed state and cannot run direct startup sequence.\n"
                  "Either manually unpack hexapod or set start_up_sequence to true in config file\n");
        ros::shutdown();
      }
      else
      {
        robot_state_ = PACKED;
        ROS_INFO("\nHexapod currently packed.\n");
      }
    }
    // Robot state unknown
    else if (!params_.start_up_sequence.data)
    {
      robot_state_ = OFF;
      ROS_WARN("\nstart_up_sequence parameter is set to false, "
               "ensure hexapod is off the ground before transitioning system state.\n");
    }
    else
    {
      robot_state_ = OFF;
      ROS_WARN("\nHexapod state is unknown. Future state transitions may be undesireable, "
               "recommend ensuring hexapod is off the ground before proceeding.\n");
    }
  }
  // OFF -> !OFF (Start controller or directly transition to walking stance)
  else if (robot_state_ == OFF && new_robot_state_ != OFF)
  {
    // OFF -> RUNNING (Direct startup)
    if (new_robot_state_ == RUNNING && !params_.start_up_sequence.data)
    {
      int progress = poser_->directStartup();
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nHexapod transitioning directly to RUNNING state (%d%%). . .\n", progress);
      if (progress == PROGRESS_COMPLETE)
      {
        robot_state_ = RUNNING;
        ROS_INFO("\nDirect startup sequence complete. Ready to walk.\n");
      }
    }
    // OFF -> PACKED/READY/RUNNING (Start controller)
    else
    {
      robot_state_ = PACKED;
      ROS_INFO("\nController running.\n");
    }
  }
  // PACKED -> OFF (Suspend controller)
  else if (robot_state_ == PACKED && new_robot_state_ == OFF)
  {
    robot_state_ = OFF;
    ROS_INFO("\nController suspended.\n");
  }
  // PACKED -> READY/RUNNING (Unpack Hexapod)
  else if (robot_state_ == PACKED && (new_robot_state_ == READY || new_robot_state_ == RUNNING))
  {
    int progress = poser_->unpackLegs(2.0 / params_.step_frequency.current_value);
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nHexapod transitioning to READY state (%d%%). . .\n", progress);
    if (progress == PROGRESS_COMPLETE)
    {
      robot_state_ = READY;
      ROS_INFO("\nState transition complete. Hexapod is in READY state.\n");
    }
  }
  // READY -> PACKED/OFF (Pack Hexapod)
  else if (robot_state_ == READY && (new_robot_state_ == PACKED || new_robot_state_ == OFF))
  {
    int progress = poser_->packLegs(2.0 / params_.step_frequency.current_value);
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nHexapod transitioning to PACKED state (%d%%). . .\n", progress);
    if (progress == PROGRESS_COMPLETE)
    {
      robot_state_ = PACKED;
      ROS_INFO("\nState transition complete. Hexapod is in PACKED state.\n");
    }
  }
  // READY -> RUNNING (Initate start up sequence to step to walking stance)
  else if (robot_state_ == READY && new_robot_state_ == RUNNING)
  {
    int progress = poser_->executeSequence(START_UP);
    string progress_string = (progress == -1 ? "Generating Sequence" : (numberToString(progress) + "%"));
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nHexapod transitioning to RUNNING state (%s). . .\n", progress_string.c_str());
    if (progress == PROGRESS_COMPLETE)
    {
      robot_state_ = RUNNING;
      ROS_INFO("\nState transition complete. Hexapod is in RUNNING state. Ready to walk.\n");
    }
  }
  // RUNNING -> !RUNNING (Initiate shut down sequence to step from walking stance to ready stance or suspend controller)
  else if (robot_state_ == RUNNING && new_robot_state_ != RUNNING)
  {
    // RUNNING -> OFF (Suspend controller)
    if (new_robot_state_ == OFF && !params_.start_up_sequence.data)
    {
      robot_state_ = OFF;
      ROS_INFO("\nController suspended.\n");
    }
    else
    {
      int progress = poser_->executeSequence(SHUT_DOWN);
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nHexapod transitioning to READY state (%d%%). . .\n", progress);
      if (progress == PROGRESS_COMPLETE)
      {
        robot_state_ = READY;
        ROS_INFO("\nState transition complete. Hexapod is in READY state.\n");
      }
    }
  }
  // Undefined system transition
  else
  {
    ROS_FATAL("\nUndefined system state transition was requested! Shutting down controller!\n");
    ros::shutdown();
  }

  // Transition complete
  if (robot_state_ == new_robot_state_)
  {
    transition_state_flag_ = false;
  }
}

/***********************************************************************************************************************
 * Running state
***********************************************************************************************************************/
void StateController::runningState()
{
  // Switch gait and update walker parameters
  if (gait_change_flag_)
  {
    changeGait();
  }
  // Dynamically adjust parameters and change stance if required
  else if (parameter_adjust_flag_)
  {
    adjustParameter();
  }
  // Toggle state of leg and transition between states
  else if (toggle_primary_leg_state_ || toggle_secondary_leg_state_)
  {
    legStateToggle();
  }
  // Cruise control (constant velocity input)
  else if (cruise_control_mode_ == CRUISE_CONTROL_ON)
  {
    linear_velocity_input_ = linear_cruise_velocity_;
    angular_velocity_input_ = angular_cruise_velocity_;
  }

  // Update tip positions unless hexapod is undergoing gait switch, parameter adjustment or leg state transition
  //(which all only occur once the hexapod has stopped walking)
  if (!((gait_change_flag_ || parameter_adjust_flag_ || toggle_primary_leg_state_ || toggle_secondary_leg_state_) && walker_->getWalkState() == STOPPED))
  {
    // Update tip positions for walking legs
    walker_->updateWalk(linear_velocity_input_, angular_velocity_input_);

    // Update tip positions for manually controlled legs
    walker_->updateManual(primary_leg_selection_, primary_tip_velocity_input_, secondary_leg_selection_, secondary_tip_velocity_input_);
    // Pose controller takes current tip positions from walker and applies pose compensation
    poser_->updateStance();
    // Model uses posed tip positions, adds deltaZ from impedance controller and applies inverse kinematics on each leg
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg* leg = leg_it_->second;
      LegPoser* leg_poser = leg->getLegPoser();
      leg->setDesiredTipPosition(leg_poser->getCurrentTipPosition());
      leg->applyIK(params_.debug_IK.data);
    }
  }
}

/***********************************************************************************************************************
 * Dynamic Parameter adjustment
***********************************************************************************************************************/
void StateController::adjustParameter()
{
  if (walker_->getWalkState() == STOPPED)
  {
    AdjustableParameter* p = dynamic_param_;
    if (!new_parameter_set_)
    {
      p->current_value = new_parameter_value_;
      walker_->init();
      impedance_->init();
      poser_->setAutoPoseParams();
      new_parameter_set_ = true;
      ROS_INFO("\nAttempting to adjust '%s' parameter to %f. (Default: %f, Min: %f, Max: %f) . . .\n", p->name.c_str(), p->current_value, p->default_value, p->min_value, p->max_value);
    }
    else
    {
      // Update tip Positions for new parameter value
      int progress = poser_->stepToNewStance();
      if (progress == 100)
      {
        ROS_INFO("\nParameter '%s' set to %f. (Default: %f, Min: %f, Max: %f)\n", p->name.c_str(), p->current_value, p->default_value, p->min_value, p->max_value);
        parameter_adjust_flag_ = false;
        new_parameter_set_ = false;
      }
    }
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nStopping hexapod to adjust parameters . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angular_velocity_input_ = 0.0;
  }
}

/***********************************************************************************************************************
 * Gait change
***********************************************************************************************************************/
void StateController::changeGait(void)
{
  if (walker_->getWalkState() == STOPPED)
  {
    initGaitParameters(gait_selection_);
    walker_->setGaitParams(&params_);
    if (params_.auto_compensation.data && params_.auto_pose_type.data == "auto")
    {
      initAutoPoseParameters();
      poser_->setAutoPoseParams();
    }
    ROS_INFO("\nNow using %s mode.\n", params_.gait_type.data.c_str());
    gait_change_flag_ = false;
  }
  // Force hexapod to stop walking
  else
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nStopping hexapod to change gait . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angular_velocity_input_ = 0.0;
  }
}

/***********************************************************************************************************************
 * Leg State Toggle
***********************************************************************************************************************/
void StateController::legStateToggle()
{
  if (walker_->getWalkState() == STOPPED)
  {
    // Choose primary or secondary leg state to transition
    Leg* leg; //Transitioning leg
    if (toggle_primary_leg_state_)
    {
      leg = model_->getLegByIDNumber(primary_leg_selection_);
    }
    else if (toggle_secondary_leg_state_)
    {
      leg = model_->getLegByIDNumber(secondary_leg_selection_);
    }
    string leg_name = leg->getIDName();

    // Calculate default pose for new loading pattern
    poser_->calculateDefaultPose();

    if (leg->getLegState() == WALKING)
    {
      if (manual_leg_count_ < MAX_MANUAL_LEGS)
      {
        ROS_INFO_COND(leg->getLegState() == WALKING, "\n%s leg transitioning to MANUAL state . . .\n", leg->getIDName().c_str());
        leg->setLegState(WALKING_TO_MANUAL);
      }
      else
      {
        ROS_INFO("\nOnly allowed to have %d legs manually manipulated at one time.\n", MAX_MANUAL_LEGS);
        toggle_primary_leg_state_ = false;
        toggle_secondary_leg_state_ = false;
      }
    }
    else if (leg->getLegState() == MANUAL)
    {
      ROS_INFO_COND(leg->getLegState() == MANUAL, "\n%s leg transitioning to WALKING state . . .\n", leg->getIDName().c_str());
      leg->setLegState(MANUAL_TO_WALKING);
    }
    else if (leg->getLegState() == WALKING_TO_MANUAL)
    {
      poser_->setPoseResetMode(IMMEDIATE_ALL_RESET);  // Set to ALL_RESET to force pose to new default pose
      int progress = poser_->poseForLegManipulation();
      /*
      if (params_.dynamic_stiffness.data)
      {
        impedance_->updateStiffness(leg, res);
      }
      */

      if (progress == 100)
      {
        leg->setLegState(MANUAL);
        ROS_INFO("\n%s leg set to state: MANUAL.\n", leg->getIDName().c_str());
        toggle_primary_leg_state_ = false;
        toggle_secondary_leg_state_ = false;
        poser_->setPoseResetMode(NO_RESET);
        manual_leg_count_++;
      }
    }
    else if (leg->getLegState() == MANUAL_TO_WALKING)
    {
      poser_->setPoseResetMode(IMMEDIATE_ALL_RESET);  // Set to ALL_RESET to force pose to new default pose
      int progress = poser_->poseForLegManipulation();
      /*
      if (params_.dynamic_stiffness.data)
      {
        impedance_->updateStiffness(leg, res);
      }
      */
      if (progress == 100)
      {
        leg->setLegState(WALKING);
        ROS_INFO("\n%s leg set to state: WALKING.\n", leg->getIDName().c_str());
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
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nStopping hexapod to transition leg state . . .\n");
    linear_velocity_input_ = Vector2d(0.0, 0.0);
    angular_velocity_input_ = 0.0;
  }
}

/***********************************************************************************************************************
 * Confirms/clamps desired joint positions and velocities and calls motor interface to publish desired joint state
***********************************************************************************************************************/
void StateController::publishDesiredJointState(void)
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    map<int, Joint*>::iterator joint_it;
    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;
      joint->prev_desired_position = joint->desired_position;
      if (params_.debug_gazebo.data)
      {
        std_msgs::Float64 msg;
        msg.data = joint->desired_position + joint->position_offset;
        joint->gazebo_publisher.publish(msg);
      }
    }
  }
  interface_->publish(); //Uses joint states stored in joint objects in model
}

/***********************************************************************************************************************
 * Publishes various state variables for each leg
***********************************************************************************************************************/
void StateController::publishLegState()
{

  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    simple_hexapod_controller::legState msg;
    Leg* leg = leg_it_->second;
    LegStepper* leg_stepper = leg->getLegStepper();
    LegPoser* leg_poser = leg->getLegPoser();
    msg.header.stamp = ros::Time::now();
    msg.leg_name.data = leg->getIDName().c_str();
    // Tip positions
    msg.local_tip_position.x = leg->getLocalTipPosition()[0];
    msg.local_tip_position.y = leg->getLocalTipPosition()[1];
    msg.local_tip_position.z = leg->getLocalTipPosition()[2];
    msg.poser_tip_positions.x = leg_poser->getCurrentTipPosition()[0];
    msg.poser_tip_positions.y = leg_poser->getCurrentTipPosition()[1];
    msg.poser_tip_positions.z = leg_poser->getCurrentTipPosition()[2];
    msg.walker_tip_positions.x = leg_stepper->getCurrentTipPosition()[0];
    msg.walker_tip_positions.y = leg_stepper->getCurrentTipPosition()[1];
    msg.walker_tip_positions.z = leg_stepper->getCurrentTipPosition()[2];

    //Joint positions/velocities
    for (joint_it_ = leg->getJointContainer()->begin(); joint_it_ != leg->getJointContainer()->end(); ++joint_it_)
    {
      Joint* joint = joint_it_->second;
      msg.joint_positions.data.push_back(joint->desired_position);
      msg.joint_velocities.data.push_back(joint->desired_velocity);
      msg.joint_efforts.data.push_back(joint->current_effort); //TBD desired effort
    }

    // Step progress
    msg.swing_progress.data = leg_stepper->getSwingProgress();
    msg.stance_progress.data = leg_stepper->getStanceProgress();

    // Leg specific auto pose
    Pose auto_pose = leg_poser->getAutoPose();
    msg.auto_pose.linear.x = auto_pose.position_[0];
    msg.auto_pose.linear.y = auto_pose.position_[1];
    msg.auto_pose.linear.z = auto_pose.position_[2];
    msg.auto_pose.angular.x = auto_pose.rotation_.toEulerAngles()[0];
    msg.auto_pose.angular.y = auto_pose.rotation_.toEulerAngles()[1];
    msg.auto_pose.angular.z = auto_pose.rotation_.toEulerAngles()[2];

    // Impedance controller
    msg.tip_force.data = leg->getTipForce();
    msg.delta_z.data = leg->getDeltaZ();
    msg.virtual_stiffness.data = leg->getVirtualStiffness();

    leg->publishState(msg);

    // Publish leg state (ASC)
    std_msgs::Bool asc_msg;
    if (leg_stepper->getStepState() == SWING ||
        (leg->getLegState() != WALKING && leg->getLegState() != MANUAL))
    {
      asc_msg.data = true;
    }
    else
    {
      asc_msg.data = false;
    }
    leg->publishASCState(asc_msg);
  }
}

/***********************************************************************************************************************
 * Publishes body velocity for debugging
***********************************************************************************************************************/
void StateController::publishBodyVelocity()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(walker_->getDesiredLinearVelocity()[0]);
  msg.data.push_back(walker_->getDesiredLinearVelocity()[1]);
  msg.data.push_back(walker_->getDesiredAngularVelocity());
  body_velocity_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current pose (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishPose()
{
  geometry_msgs::Twist msg;
  msg.linear.x = model_->getCurrentPose().position_[0];
  msg.linear.y = model_->getCurrentPose().position_[1];
  msg.linear.z = model_->getCurrentPose().position_[2];
  msg.angular.x = model_->getCurrentPose().rotation_.toEulerAngles()[0];
  msg.angular.y = model_->getCurrentPose().rotation_.toEulerAngles()[1];
  msg.angular.z = model_->getCurrentPose().rotation_.toEulerAngles()[2];
  pose_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current rotation as per the IMU (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishIMUData()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser_->getImuData().orientation.toEulerAngles()[0]);
  msg.data.push_back(poser_->getImuData().orientation.toEulerAngles()[1]);
  msg.data.push_back(poser_->getImuData().orientation.toEulerAngles()[2]);
  msg.data.push_back(poser_->getImuData().linear_acceleration[0]);
  msg.data.push_back(poser_->getImuData().linear_acceleration[1]);
  msg.data.push_back(poser_->getImuData().linear_acceleration[2]);
  msg.data.push_back(poser_->getImuData().angular_velocity[0]);
  msg.data.push_back(poser_->getImuData().angular_velocity[1]);
  msg.data.push_back(poser_->getImuData().angular_velocity[2]);
  imu_data_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishRotationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser_->getRotationAbsementError()[0]);
  msg.data.push_back(poser_->getRotationAbsementError()[1]);
  msg.data.push_back(poser_->getRotationAbsementError()[2]);
  msg.data.push_back(poser_->getRotationPositionError()[0]);
  msg.data.push_back(poser_->getRotationPositionError()[1]);
  msg.data.push_back(poser_->getRotationPositionError()[2]);
  msg.data.push_back(poser_->getRotationVelocityError()[0]);
  msg.data.push_back(poser_->getRotationVelocityError()[1]);
  msg.data.push_back(poser_->getRotationVelocityError()[2]);
  rotation_pose_error_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishTranslationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser_->getTranslationAbsementError()[0]);
  msg.data.push_back(poser_->getTranslationAbsementError()[1]);
  msg.data.push_back(poser_->getTranslationAbsementError()[2]);
  msg.data.push_back(poser_->getTranslationPositionError()[0]);
  msg.data.push_back(poser_->getTranslationPositionError()[1]);
  msg.data.push_back(poser_->getTranslationPositionError()[2]);
  msg.data.push_back(poser_->getTranslationVelocityError()[0]);
  msg.data.push_back(poser_->getTranslationVelocityError()[1]);
  msg.data.push_back(poser_->getTranslationVelocityError()[2]);
  translation_pose_error_publisher_.publish(msg);
}

/***********************************************************************************************************************
 * Draws robot in RVIZ for debugging
***********************************************************************************************************************/
void StateController::RVIZDebugging(bool static_display)
{
  Vector2d linear_velocity = walker_->getDesiredLinearVelocity(); //TBD Implement calculation of actual body velocity
  linear_velocity *= (static_display) ? 0.0 : params_.time_delta.data;
  double angular_velocity = walker_->getDesiredAngularVelocity();
  angular_velocity *= (static_display) ? 0.0 : params_.time_delta.data;

  debug_.updatePose(linear_velocity, angular_velocity, walker_->getBodyHeight());
  debug_.drawRobot(model_);
  debug_.drawPoints(model_, walker_, static_display);
}

/***********************************************************************************************************************
 * Input Body Velocity Topic Callback
***********************************************************************************************************************/
void StateController::bodyVelocityInputCallback(const geometry_msgs::Twist& input)
{
  linear_velocity_input_ = Vector2d(input.linear.x, input.linear.y);
  angular_velocity_input_ = input.angular.z;
}

/***********************************************************************************************************************
 * Input Primary Manual Tip Velocity Topic Callback
***********************************************************************************************************************/
void StateController::primaryTipVelocityInputCallback(const geometry_msgs::Point& input)
{
  primary_tip_velocity_input_ = Vector3d(input.x, input.y, input.z);
}

/***********************************************************************************************************************
 * Input Secondary Manual Tip Velocity Topic Callback
***********************************************************************************************************************/
void StateController::secondaryTipVelocityInputCallback(const geometry_msgs::Point& input)
{
  secondary_tip_velocity_input_ = Vector3d(input.x, input.y, input.z);
}

/***********************************************************************************************************************
 * Body Pose Topic Callback
***********************************************************************************************************************/
void StateController::bodyPoseInputCallback(const geometry_msgs::Twist& input)
{
  if (system_state_ != SUSPENDED && poser_ != NULL)
  {
    Vector3d rotation_input(input.angular.x, input.angular.y, input.angular.z);
    Vector3d translation_input(input.linear.x, input.linear.y, input.linear.z);
    poser_->setManualPoseInput(translation_input, rotation_input);
  }
}

/***********************************************************************************************************************
 * System state callback handling desired system state from hexapod_remote
***********************************************************************************************************************/
void StateController::systemStateCallback(const std_msgs::Int8& input)
{
  new_system_state_ = static_cast<SystemState>(int(input.data));
  if (system_state_ != new_system_state_)
  {
    system_state_ = new_system_state_;
    if (system_state_ == OPERATIONAL && initialised_)
    {
      ROS_INFO("\nController operation resumed.\n");
    }
  }
}

/***********************************************************************************************************************
 * System state callback handling desired system state from hexapod_remote
***********************************************************************************************************************/
void StateController::robotStateCallback(const std_msgs::Int8& input)
{
  RobotState input_state = static_cast<RobotState>(int(input.data));
  // If startUpSequence parameter is false then skip READY and PACKED states
  new_robot_state_ = input_state;
  if (!params_.start_up_sequence.data)
  {
    if (new_robot_state_ == READY)
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nSystem not able to transition to READY state, parameter start_up_sequence is false.\nPress START to skip this state and attempt to transition to the next.\n");
      new_robot_state_ = OFF;
    }
    else if (new_robot_state_ == PACKED)
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nSystem not able to transition to PACKED state, parameter start_up_sequence is false.\nPress START to skip this state and attempt to transition to the next.\n");
      new_robot_state_ = OFF;
    }
  }

  if (new_robot_state_ != robot_state_)
  {
    transition_state_flag_ = true;
  }
}

/***********************************************************************************************************************
 * Gait Selection Callback
***********************************************************************************************************************/
void StateController::gaitSelectionCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    GaitDesignation new_gait_selection = static_cast<GaitDesignation>(int(input.data));
    if (new_gait_selection != gait_selection_ && new_gait_selection != GAIT_UNDESIGNATED)
    {
      gait_selection_ = new_gait_selection;
      gait_change_flag_ = true;
    }
  }
}

/***********************************************************************************************************************
 * Posing Mode Callback
***********************************************************************************************************************/
void StateController::posingModeCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    PosingMode new_posing_mode = static_cast<PosingMode>(int(input.data));
    if (new_posing_mode != posing_mode_)
    {
      posing_mode_ = new_posing_mode;
      switch (posing_mode_) //Used only for user message, control handled by hexapod_remote
      {
        case (NO_POSING):
          ROS_INFO("\nPosing mode set to NO_POSING. "
                   "Body will not respond to manual posing input (except for reset commands).\n");
          break;
        case (X_Y_POSING):
          ROS_INFO("\nPosing mode set to X_Y_POSING. "
                   "Body will only respond to x/y translational manual posing input.\n");
          break;
        case (PITCH_ROLL_POSING):
          ROS_INFO("\nPosing mode set to PITCH_ROLL_POSING. "
                   "Body will only respond to pitch/roll rotational manual posing input.\n");
          break;
        case (Z_YAW_POSING):
          ROS_INFO("\nPosing mode set to Z_YAW_POSING. "
                   "Body will only respond to z translational and yaw rotational manual posing input.\n");
          break;
      }
    }
  }
}

/***********************************************************************************************************************
 * Cruise Control Callback
***********************************************************************************************************************/
void StateController::cruiseControlCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    CruiseControlMode new_cruise_control_mode = static_cast<CruiseControlMode>(int(input.data));
    if (new_cruise_control_mode != cruise_control_mode_)
    {
      cruise_control_mode_ = new_cruise_control_mode;
      if (new_cruise_control_mode == CRUISE_CONTROL_ON)
      {
        if (params_.force_cruise_velocity.data)
        {
          // Set cruise velocity according to parameters
          linear_cruise_velocity_[0] = params_.linear_cruise_velocity.data["x"];
          linear_cruise_velocity_[1] = params_.linear_cruise_velocity.data["y"];
          angular_cruise_velocity_ = params_.angular_cruise_velocity.data;
        }
        else
        {
          // Save current velocity input as cruise input
          linear_cruise_velocity_ = linear_velocity_input_;
          angular_cruise_velocity_ = angular_velocity_input_;
        }
        ROS_INFO("\nCruise control ON - Input velocity set to constant: Linear(X:Y): %f:%f, Angular(Z): %f\n",
                 linear_cruise_velocity_[0], linear_cruise_velocity_[1], angular_cruise_velocity_);
      }
      else if (new_cruise_control_mode == CRUISE_CONTROL_OFF)
      {
        ROS_INFO("\nCruise control OFF - Input velocity set by user.\n");
      }
    }
  }
}

/***********************************************************************************************************************
 * Auto Navigation Callback
***********************************************************************************************************************/
void StateController::autoNavigationCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    AutoNavigationMode new_auto_navigation_mode = static_cast<AutoNavigationMode>(int(input.data));
    if (new_auto_navigation_mode != auto_navigation_mode_)
    {
      auto_navigation_mode_ = new_auto_navigation_mode;
      bool auto_navigation_on = (auto_navigation_mode_ == AUTO_NAVIGATION_ON);
      ROS_INFO_COND(auto_navigation_on, "\nAuto Navigation mode ON. User input is being ignored.\n");
      ROS_INFO_COND(!auto_navigation_on, "\nAuto Navigation mode OFF. Control returned to user input.\n");
    }
  }
}

/***********************************************************************************************************************
 * Parameter selection callback
***********************************************************************************************************************/
void StateController::parameterSelectionCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    ParameterSelection new_parameter_selection = static_cast<ParameterSelection>(int(input.data));
    if (new_parameter_selection != parameter_selection_)
    {
      parameter_selection_ = new_parameter_selection;
      if (parameter_selection_ != NO_PARAMETER_SELECTION)
      {
        dynamic_param_ = params_.adjustable_map[parameter_selection_];
        ROS_INFO("\n%s parameter currently selected.\n", dynamic_param_->name.c_str());
      }
      else
      {
        ROS_INFO("\nNo parameter currently selected.\n");
      }
    }
  }
}

/***********************************************************************************************************************
 * Parameter adjustment callback
***********************************************************************************************************************/
void StateController::parameterAdjustCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    int adjust_direction = input.data; // -1 || 0 || 1 (Increase, no adjustment, decrease)
    if (adjust_direction != 0.0 && !parameter_adjust_flag_ && parameter_selection_ != NO_PARAMETER_SELECTION)
    {
      double parameter_adjustment = dynamic_param_->adjust_step;
      if (sign(dynamic_param_->adjust_step) != sign(adjust_direction)) //If directions differ
      {
        parameter_adjustment *= -1; //Change direction
      }
      new_parameter_value_ = dynamic_param_->current_value + parameter_adjustment;
      new_parameter_value_ = clamped(new_parameter_value_, dynamic_param_->min_value, dynamic_param_->max_value);
      parameter_adjust_flag_ = true;
    }
  }
}

/***********************************************************************************************************************
 * Parameter (Dynamic Reconfigure) adjustment callback
***********************************************************************************************************************/
void StateController::dynamicParameterCallback(simple_hexapod_controller::DynamicConfig& config, uint32_t level)
{
  if (robot_state_ == RUNNING)
  {
    parameter_adjust_flag_ = true;
    if (config.step_frequency != params_.step_frequency.current_value)
    {
      dynamic_param_ = &params_.step_frequency;
      new_parameter_value_ = clamped(config.step_frequency, dynamic_param_->min_value, dynamic_param_->max_value);
      config.step_frequency = new_parameter_value_;
    }
    else if (config.step_clearance != params_.step_clearance.current_value)
    {
      dynamic_param_ = &params_.step_clearance;
      new_parameter_value_ = clamped(config.step_clearance, dynamic_param_->min_value, dynamic_param_->max_value);
      config.step_clearance = new_parameter_value_;
    }
    else if (config.body_clearance != params_.body_clearance.current_value)
    {
      dynamic_param_ = &params_.body_clearance;
      new_parameter_value_ = clamped(config.body_clearance, dynamic_param_->min_value, dynamic_param_->max_value);
      config.body_clearance = new_parameter_value_;
    }
    else if (config.leg_span != params_.leg_span.current_value)
    {
      dynamic_param_ = &params_.leg_span;
      new_parameter_value_ = clamped(config.leg_span, dynamic_param_->min_value, dynamic_param_->max_value);
      config.leg_span = new_parameter_value_;
    }
    else if (config.virtual_mass != params_.virtual_mass.current_value)
    {
      dynamic_param_ = &params_.virtual_mass;
      new_parameter_value_ = clamped(config.virtual_mass, dynamic_param_->min_value, dynamic_param_->max_value);
      config.virtual_mass = new_parameter_value_;
    }
    else if (config.virtual_stiffness != params_.virtual_stiffness.current_value)
    {
      dynamic_param_ = &params_.virtual_stiffness;
      new_parameter_value_ = clamped(config.virtual_stiffness, dynamic_param_->min_value, dynamic_param_->max_value);
      config.virtual_stiffness = new_parameter_value_;
    }
    else if (config.virtual_damping_ratio != params_.virtual_damping_ratio.current_value)
    {
      dynamic_param_ = &params_.virtual_damping_ratio;
      new_parameter_value_ = clamped(config.virtual_damping_ratio, dynamic_param_->min_value, dynamic_param_->max_value);
      config.virtual_damping_ratio = new_parameter_value_;
    }
    else if (config.force_gain != params_.force_gain.current_value)
    {
      dynamic_param_ = &params_.force_gain;
      new_parameter_value_ = clamped(config.force_gain, dynamic_param_->min_value, dynamic_param_->max_value);
      config.force_gain = new_parameter_value_;
    }
    else
    {
      parameter_adjust_flag_ = false;
    }
    dynamic_reconfigure_server_->updateConfig(config);
  }
}

/***********************************************************************************************************************
 * Pose reset mode callback
***********************************************************************************************************************/
void StateController::poseResetCallback(const std_msgs::Int8& input)
{
  if (system_state_ != SUSPENDED && poser_ != NULL)
  {
    if (poser_->getPoseResetMode() != IMMEDIATE_ALL_RESET)
    {
      poser_->setPoseResetMode(static_cast<PoseResetMode>(input.data));
    }
  }
}

/***********************************************************************************************************************
 * Actuating Leg Primary Selection Callback
***********************************************************************************************************************/
void StateController::primaryLegSelectionCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    LegDesignation new_primary_leg_selection = static_cast<LegDesignation>(input.data);
    if (primary_leg_selection_ != new_primary_leg_selection)
    {
      primary_leg_selection_ = new_primary_leg_selection;
      if (new_primary_leg_selection != LEG_UNDESIGNATED)
      {
        primary_leg_ = model_->getLegByIDNumber(primary_leg_selection_);
        ROS_INFO("\n%s leg selected for primary control.\n", primary_leg_->getIDName().c_str());
      }
      else
      {
        ROS_INFO("\nNo leg currently selected for primary control.\n");
      }
    }
  }
}

/***********************************************************************************************************************
 * Actuating Leg Secondary Selection Callback
***********************************************************************************************************************/
void StateController::secondaryLegSelectionCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    LegDesignation new_secondary_leg_selection = static_cast<LegDesignation>(input.data);
    if (secondary_leg_selection_ != new_secondary_leg_selection)
    {
      secondary_leg_selection_ = new_secondary_leg_selection;
      if (new_secondary_leg_selection != LEG_UNDESIGNATED)
      {
        secondary_leg_ = model_->getLegByIDNumber(secondary_leg_selection_);
        ROS_INFO("\n%s leg selected for secondary control.\n", secondary_leg_->getIDName().c_str());
      }
      else
      {
        ROS_INFO("\nNo leg currently selected for secondary control.\n");
      }
    }
  }
}

/***********************************************************************************************************************
 * Toggle Primary Leg State Callback
***********************************************************************************************************************/
void StateController::primaryLegStateCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    LegState newPrimaryLegState = static_cast<LegState>(int(input.data));
    if (newPrimaryLegState != primary_leg_state_)
    {
      if (primary_leg_selection_ == LEG_UNDESIGNATED)
      {
        ROS_INFO("\nCannot toggle primary leg state as no leg is currently selected as primary.");
        ROS_INFO("\nPress left bumper to select a leg and try again.\n");
      }
      else if (toggle_secondary_leg_state_)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nCannot toggle primary leg state as secondary leg is currently "
                          "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nPlease wait and try again.\n");
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
void StateController::secondaryLegStateCallback(const std_msgs::Int8& input)
{
  if (robot_state_ == RUNNING)
  {
    LegState newSecondaryLegState = static_cast<LegState>(int(input.data));
    if (newSecondaryLegState != secondary_leg_state_)
    {
      if (secondary_leg_selection_ == LEG_UNDESIGNATED)
      {
        ROS_INFO("\nCannot toggle secondary leg state as no leg is currently selected as secondary.");
        ROS_INFO("\nPress left bumper to select a leg and try again.\n");
      }
      else if (toggle_primary_leg_state_)
      {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nCannot toggle secondary leg state as secondary leg is currently "
                          "transitioning states.");
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nPlease wait and try again.\n");
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
void StateController::imuCallback(const sensor_msgs::Imu& data)
{
  Vector3d euler_offset;
  euler_offset[0] = params_.imu_rotation_offset.data[0];
  euler_offset[1] = params_.imu_rotation_offset.data[1];
  euler_offset[2] = params_.imu_rotation_offset.data[2];
  Quat imu_rotation_offset(euler_offset);
  //TBD use tf

  Quat raw_orientation;
  raw_orientation.w = data.orientation.w;
  raw_orientation.x = data.orientation.x;
  raw_orientation.y = data.orientation.y;
  raw_orientation.z = data.orientation.z;

  Vector3d raw_linear_acceleration;
  raw_linear_acceleration[0] = data.linear_acceleration.x;
  raw_linear_acceleration[1] = data.linear_acceleration.y;
  raw_linear_acceleration[2] = data.linear_acceleration.z;

  Vector3d raw_angular_velocity;
  raw_angular_velocity[0] = data.angular_velocity.x;
  raw_angular_velocity[1] = data.angular_velocity.y;
  raw_angular_velocity[2] = data.angular_velocity.z;

  // Rotate raw imu data according to physical imu mounting
  poser_->setImuData((imu_rotation_offset * raw_orientation) * imu_rotation_offset.inverse(),
                     imu_rotation_offset.toRotationMatrix() * raw_linear_acceleration,
                     imu_rotation_offset.toRotationMatrix() * raw_angular_velocity);
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void StateController::jointStatesCallback(const sensor_msgs::JointState& joint_states)
{
  bool get_effort_values = (joint_states.effort.size() != 0);
  bool get_velocity_values = (joint_states.velocity.size() != 0);

  // Iterate through message and assign found state values to joint objects
  for (uint i = 0; i < joint_states.name.size(); ++i)
  {
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg* leg = leg_it_->second;
      string joint_name(joint_states.name[i]);
      Joint* joint = leg->getJointByIDName(joint_name);
      if (joint != NULL)
      {
        joint->current_position = joint_states.position[i] - joint->position_offset;
        if (get_velocity_values)
        {
          joint->current_velocity = joint_states.velocity[i];
        }
        if (get_effort_values)
        {
          joint->current_effort = joint_states.effort[i];
        }
      }
    }
  }
  // Check if all joint positions have been received from topic
  if (!joint_positions_initialised)
  {
    joint_positions_initialised = true;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
    {
      Leg* leg = leg_it_->second;
      map<int, Joint*>::iterator joint_it;
      for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
      {
        Joint* joint = joint_it->second;
        if (joint->current_position == UNASSIGNED_VALUE)
        {
          joint_positions_initialised = false;
        }
      }
    }
  }
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallback(const sensor_msgs::JointState& raw_tip_forces) //TBD redesign
{
  for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_)
  {
    Leg* leg = leg_it_->second;
    double force_offset = 1255.0;
    double max_force = 1000.0;
    double min_force = 0.0;
    double tip_force = clamped(raw_tip_forces.effort[leg->getIDNumber() * 2] - force_offset, min_force, max_force);
    leg->setTipForce(tip_force);
  }
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackAR(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("AR");
  double force_offset = 23930.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackBR(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("BR");
  double force_offset = 23845.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackCR(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("CR");
  double force_offset = 23930.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackCL(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("CL");
  double force_offset = 23895.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackBL(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("BL");
  double force_offset = 23775.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallbackAL(const std_msgs::UInt16& raw_tip_force) //TBD redesign
{
  Leg* leg = model_->getLegByIDName("AL");
  double force_offset = 23920.0;
  double max_force = 1000.0;
  double min_force = 0.0;
  double tip_force = clamped(raw_tip_force.data - force_offset, min_force, max_force);
  leg->setTipForce(tip_force);
}

/***********************************************************************************************************************
 * Initialises model by initialising all legs
***********************************************************************************************************************/
void StateController::initModel(bool use_default_joint_positions)
{
  model_->initLegs(use_default_joint_positions);
}

/***********************************************************************************************************************
 * Gets hexapod parameters from rosparam server
***********************************************************************************************************************/
void StateController::initParameters(void)
{
  // Control parameters
  params_.time_delta.init(n_, "time_delta");
  params_.imu_compensation.init(n_, "imu_compensation");
  params_.auto_compensation.init(n_, "auto_compensation");
  params_.manual_compensation.init(n_, "manual_compensation");
  params_.inclination_compensation.init(n_, "inclination_compensation");
  params_.impedance_control.init(n_, "impedance_control");
  params_.imu_rotation_offset.init(n_, "imu_rotation_offset");
  params_.interface_setup_speed.init(n_, "interface_setup_speed");
  // Model parameters
  params_.hexapod_type.init(n_, "hexapod_type");
  params_.leg_id.init(n_, "leg_id");
  params_.joint_id.init(n_, "joint_id");
  params_.link_id.init(n_, "link_id");
  params_.leg_DOF.init(n_, "leg_DOF");
  params_.leg_stance_yaws.init(n_, "leg_stance_yaws");
  // Walk controller parameters
  params_.gait_type.init(n_, "gait_type");
  params_.step_frequency.init(n_, "step_frequency");
  params_.step_clearance.init(n_, "step_clearance");
  params_.step_depth.init(n_, "step_depth");
  params_.body_clearance.init(n_, "body_clearance");
  params_.leg_span.init(n_, "leg_span");
  params_.velocity_input_mode.init(n_, "velocity_input_mode");
  params_.force_cruise_velocity.init(n_, "force_cruise_velocity");
  params_.linear_cruise_velocity.init(n_, "linear_cruise_velocity");
  params_.angular_cruise_velocity.init(n_, "angular_cruise_velocity");
  // Pose controller parameters
  params_.auto_pose_type.init(n_, "auto_pose_type");
  params_.start_up_sequence.init(n_, "start_up_sequence");
  params_.time_to_start.init(n_, "time_to_start");
  params_.rotation_pid_gains.init(n_, "rotation_pid_gains");
  params_.translation_pid_gains.init(n_, "translation_pid_gains");
  params_.max_translation.init(n_, "max_translation");
  params_.max_translation_velocity.init(n_, "max_translation_velocity");
  params_.max_rotation.init(n_, "max_rotation");
  params_.max_rotation_velocity.init(n_, "max_rotation_velocity");
  params_.leg_manipulation_mode.init(n_, "leg_manipulation_mode");
  // Impedance controller parameters
  params_.dynamic_stiffness.init(n_, "dynamic_stiffness");
  params_.use_joint_effort.init(n_, "use_joint_effort");
  params_.integrator_step_time.init(n_, "integrator_step_time");
  params_.virtual_mass.init(n_, "virtual_mass");
  params_.virtual_stiffness.init(n_, "virtual_stiffness");
  params_.load_stiffness_scaler.init(n_, "load_stiffness_scaler");
  params_.swing_stiffness_scaler.init(n_, "swing_stiffness_scaler");
  params_.virtual_damping_ratio.init(n_, "virtual_damping_ratio");
  params_.force_gain.init(n_, "force_gain");  
  // Debug Parameters
  params_.debug_gazebo.init(n_, "debug_gazebo");
  params_.debug_rviz.init(n_, "debug_rviz");
  params_.debug_rviz_static_display.init(n_, "debug_rviz_static_display");
  params_.console_verbosity.init(n_, "console_verbosity");
  params_.debug_moveToJointPosition.init(n_, "debug_move_to_joint_position");
  params_.debug_stepToPosition.init(n_, "debug_step_to_position");
  params_.debug_swing_trajectory.init(n_, "debug_swing_trajectory");
  params_.debug_stance_trajectory.init(n_, "debug_stance_trajectory");
  params_.debug_execute_sequence.init(n_, "debug_execute_sequence");
  params_.debug_IK.init(n_, "debug_ik");

  // Init all joint and link parameters per leg
  if (params_.leg_id.initialised && params_.joint_id.initialised && params_.link_id.initialised)
  {
    vector<string>::iterator leg_name_it;
    vector<string> leg_ids = params_.leg_id.data;
    int leg_id_num = 0;
    for (leg_name_it = leg_ids.begin(); leg_name_it != leg_ids.end(); ++leg_name_it, ++leg_id_num)
    {
      string leg_id_name = *leg_name_it;
      params_.link_parameters[leg_id_num][0].init(n_, leg_id_name + "_base_link_parameters");
      int num_joints = params_.leg_DOF.data[leg_id_name];
      for (int i = 1; i < num_joints + 1; ++i)
      {
        string link_name = params_.link_id.data[i];
        string joint_name = params_.joint_id.data[i - 1];
        string link_parameter_name = leg_id_name + "_" + link_name + "_link_parameters";
        string joint_parameter_name = leg_id_name + "_" + joint_name + "_joint_parameters";
        params_.link_parameters[leg_id_num][i].init(n_, link_parameter_name);
        params_.joint_parameters[leg_id_num][i - 1].init(n_, joint_parameter_name);
      }
    }
  }

  // Generate adjustable parameter map for parameter adjustment selection
  params_.adjustable_map.insert(AdjustableMapType::value_type(STEP_FREQUENCY, &params_.step_frequency));
  params_.adjustable_map.insert(AdjustableMapType::value_type(STEP_CLEARANCE, &params_.step_clearance));
  params_.adjustable_map.insert(AdjustableMapType::value_type(BODY_CLEARANCE, &params_.body_clearance));
  params_.adjustable_map.insert(AdjustableMapType::value_type(LEG_SPAN_SCALE, &params_.leg_span));
  params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_MASS, &params_.virtual_mass));
  params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_STIFFNESS, &params_.virtual_stiffness));
  params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_DAMPING, &params_.virtual_damping_ratio));
  params_.adjustable_map.insert(AdjustableMapType::value_type(FORCE_GAIN, &params_.force_gain));

  // Dynamic reconfigure server and callback setup
  dynamic_reconfigure_server_ = new dynamic_reconfigure::Server<simple_hexapod_controller::DynamicConfig>(mutex_);
  dynamic_reconfigure::Server<simple_hexapod_controller::DynamicConfig>::CallbackType callback_type;
  callback_type = boost::bind(&StateController::dynamicParameterCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(callback_type);

  // Set min/max/default values for dynamic reconfigure server
  simple_hexapod_controller::DynamicConfig config_max, config_min, config_default;
  config_max.step_frequency = params_.step_frequency.max_value;
  config_min.step_frequency = params_.step_frequency.min_value;
  config_default.step_frequency = params_.step_frequency.default_value;
  config_max.step_clearance = params_.step_clearance.max_value;
  config_min.step_clearance = params_.step_clearance.min_value;
  config_default.step_clearance = params_.step_clearance.default_value;
  config_max.body_clearance = params_.body_clearance.max_value;
  config_min.body_clearance = params_.body_clearance.min_value;
  config_default.body_clearance = params_.body_clearance.default_value;
  config_max.leg_span = params_.leg_span.max_value;
  config_min.leg_span = params_.leg_span.min_value;
  config_default.leg_span = params_.leg_span.default_value;
  config_max.virtual_mass = params_.virtual_mass.max_value;
  config_min.virtual_mass = params_.virtual_mass.min_value;
  config_default.virtual_mass = params_.virtual_mass.default_value;
  config_max.virtual_stiffness = params_.virtual_stiffness.max_value;
  config_min.virtual_stiffness = params_.virtual_stiffness.min_value;
  config_default.virtual_stiffness = params_.virtual_stiffness.default_value;
  config_max.virtual_damping_ratio = params_.virtual_damping_ratio.max_value;
  config_min.virtual_damping_ratio = params_.virtual_damping_ratio.min_value;
  config_default.virtual_damping_ratio = params_.virtual_damping_ratio.default_value;
  config_max.force_gain = params_.force_gain.max_value;
  config_min.force_gain = params_.force_gain.min_value;
  config_default.force_gain = params_.force_gain.default_value;

  dynamic_reconfigure_server_->setConfigMax(config_max);
  dynamic_reconfigure_server_->setConfigMin(config_min);
  dynamic_reconfigure_server_->setConfigDefault(config_default);
  dynamic_reconfigure_server_->updateConfig(config_default);

  initGaitParameters(GAIT_UNDESIGNATED);
  initAutoPoseParameters();
}

/***********************************************************************************************************************
 * Gets hexapod gait parameters from rosparam server
***********************************************************************************************************************/
void StateController::initGaitParameters(GaitDesignation gaitSelection)
{
  switch (gaitSelection)
  {
    case (TRIPOD_GAIT):
      params_.gait_type.data = "tripod_gait";
      break;
    case (RIPPLE_GAIT):
      params_.gait_type.data = "ripple_gait";
      break;
    case (WAVE_GAIT):
      params_.gait_type.data = "wave_gait";
      break;
    case (AMBLE_GAIT):
      params_.gait_type.data = "amble_gait";
      break;
    case (GAIT_UNDESIGNATED):
      params_.gait_type.init(n_, "gait_type");
      break;
  }

  string base_gait_parameters_name = "/hexapod/gait_parameters/";
  params_.stance_phase.init(n_, "stance_phase", base_gait_parameters_name + params_.gait_type.data + "/");
  params_.swing_phase.init(n_, "swing_phase", base_gait_parameters_name + params_.gait_type.data + "/");
  params_.phase_offset.init(n_, "phase_offset", base_gait_parameters_name + params_.gait_type.data + "/");
  params_.offset_multiplier.init(n_, "offset_multiplier", base_gait_parameters_name + params_.gait_type.data + "/");
}

/***********************************************************************************************************************
 * Gets hexapod auto pose parameters from rosparam server
***********************************************************************************************************************/
void StateController::initAutoPoseParameters(void)
{
  string base_auto_pose_parameters_name = "/hexapod/auto_pose_parameters/";
  if (params_.auto_pose_type.data == "auto")
  {
    base_auto_pose_parameters_name += (params_.gait_type.data + "_pose/");
  }
  else
  {
    base_auto_pose_parameters_name += (params_.auto_pose_type.data + "/");
  }

  params_.pose_frequency.init(n_, "pose_frequency", base_auto_pose_parameters_name);
  params_.pose_phase_length.init(n_, "pose_phase_length", base_auto_pose_parameters_name);
  params_.pose_phase_starts.init(n_, "pose_phase_starts", base_auto_pose_parameters_name);
  params_.pose_phase_ends.init(n_, "pose_phase_ends", base_auto_pose_parameters_name);
  params_.pose_negation_phase_starts.init(n_, "pose_negation_phase_starts", base_auto_pose_parameters_name);
  params_.pose_negation_phase_ends.init(n_, "pose_negation_phase_ends", base_auto_pose_parameters_name);
  params_.x_amplitudes.init(n_, "x_amplitudes", base_auto_pose_parameters_name);
  params_.y_amplitudes.init(n_, "y_amplitudes", base_auto_pose_parameters_name);
  params_.z_amplitudes.init(n_, "z_amplitudes", base_auto_pose_parameters_name);
  params_.roll_amplitudes.init(n_, "roll_amplitudes", base_auto_pose_parameters_name);
  params_.pitch_amplitudes.init(n_, "pitch_amplitudes", base_auto_pose_parameters_name);
  params_.yaw_amplitudes.init(n_, "yaw_amplitudes", base_auto_pose_parameters_name);
}
/***********************************************************************************************************************
***********************************************************************************************************************/
