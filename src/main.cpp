#include "../include/simple_hexapod_controller/stateController.h"
#include <ros/ros.h>

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_hexapod_controller");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~"); 
  
  StateController state(n);
  
  bool setLoggerLevelResult = false;
  if (state.params.consoleVerbosity == "debug")
  {
    setLoggerLevelResult = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  }
  else if (state.params.consoleVerbosity == "info")
  {
    setLoggerLevelResult = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  }
  else if (state.params.consoleVerbosity == "warning")
  {
    setLoggerLevelResult = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
  }
  else if (state.params.consoleVerbosity == "error")
  {
    setLoggerLevelResult = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  }
  else if (state.params.consoleVerbosity == "fatal")
  {
    setLoggerLevelResult = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  }
    
  if(setLoggerLevelResult) 
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  
  //Hexapod Remote topic subscriptions
  ros::Subscriber velocitySubscriber = n.subscribe("hexapod_remote/desired_velocity", 1, &StateController::joypadVelocityCallback, &state);
  ros::Subscriber poseSubscriber = n.subscribe("hexapod_remote/desired_pose", 1, &StateController::joypadPoseCallback, &state);
  ros::Subscriber legSelectSubscriber = n.subscribe("hexapod_remote/leg_selection", 1, &StateController::legSelectionCallback, &state);
  ros::Subscriber legStateSubscriber = n.subscribe("hexapod_remote/leg_state_toggle", 1, &StateController::legStateCallback, &state);
  ros::Subscriber paramSelectionSubscriber = n.subscribe("hexapod_remote/param_selection", 1, &StateController::paramSelectionCallback, &state);
  ros::Subscriber paramAdjustmentSubscriber = n.subscribe("hexapod_remote/param_adjust", 1, &StateController::paramAdjustCallback, &state);
  ros::Subscriber startTestSubscriber = n.subscribe("hexapod_remote/test_state_toggle", 1, &StateController::startTestCallback, &state);
  ros::Subscriber poseResetSubscriber = n.subscribe("hexapod_remote/pose_reset_mode", 1, &StateController::poseResetCallback, &state);  
  ros::Subscriber startSubscriber = n.subscribe("hexapod_remote/start_state", 1, &StateController::startCallback, &state);
  ros::Subscriber gaitSelectSubscriber = n.subscribe("hexapod_remote/gait_mode", 1, &StateController::gaitSelectionCallback, &state);
  
  //Motor and other sensor topic subscriptions
  ros::Subscriber imuDataSubscriber = n.subscribe("ig/imu/data", 1, &StateController::imuCallback, &state);  
  ros::Subscriber tipForceSubscriber = n.subscribe("/motor_encoders", 1, &StateController::tipForceCallback, &state);
  ros::Subscriber jointStatesSubscriber1 = n.subscribe("/hexapod_joint_state", 1000, &StateController::jointStatesCallback, &state);;
  ros::Subscriber jointStatesSubscriber2 = n.subscribe("/hexapod/joint_states", 1000, &StateController::jointStatesCallback, &state); ;
  
  //Debugging publishers
  state.legStatePublishers[0][0] = n.advertise<simple_hexapod_controller::legState>("/hexapod/front_left_leg/state", 1000);
  state.legStatePublishers[0][1] = n.advertise<simple_hexapod_controller::legState>("/hexapod/front_right_leg/state", 1000);
  state.legStatePublishers[1][0] = n.advertise<simple_hexapod_controller::legState>("/hexapod/middle_left_leg/state", 1000);
  state.legStatePublishers[1][1] = n.advertise<simple_hexapod_controller::legState>("/hexapod/middle_right_leg/state", 1000);
  state.legStatePublishers[2][0] = n.advertise<simple_hexapod_controller::legState>("/hexapod/rear_left_leg/state", 1000);
  state.legStatePublishers[2][1] = n.advertise<simple_hexapod_controller::legState>("/hexapod/rear_right_leg/state", 1000);
  
  state.posePublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/pose", 1000);
  state.IMURotationPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/imu_rotation", 1000);
  state.bodyVelocityPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/body_velocity", 1000);
  
  state.rotationPoseErrorPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/rotation_pose_error", 1000);
  state.translationPoseErrorPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/translation_pose_error", 1000);
  state.zTipErrorPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/z_tip_error", 1000);
  
  
  //Set ros rate from params
  ros::Rate r(roundToInt(1.0/state.params.timeDelta));
  
  //Display warning message concerning use of correct motor interface
  if (state.params.dynamixelInterface)
  {
    if (state.params.hexapodType == "large_hexapod")
    {
      ROS_WARN("Using dynamixel motor interface - will not work with Large Hexapod (simulations only).\nSet dynamixel_interface to false in config file if using Large Hexapod.\n");
    }
  }
  else
  {
    ROS_WARN("Using dynamixel PRO motor interface - will only work with Large Hexapod Robot (not simulations or small hexapod platforms).\nSet dynamixel_interface to true in config file if not using Large Hexapod.\n");
  }
  
  //RVIZ simulation warning message
  if (state.params.debug_rviz)
  {
    ROS_WARN("DEBUGGING USING RVIZ - CODE IS CPU INTENSIVE.\n");
  }
  
  //Loop waiting for start button press   
  while(!state.startFlag)
  {
    ROS_INFO_THROTTLE(5.0, "Press 'Start' to run controller . . .\n"); 
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Controller started.\n");
  
  
  //Wait specified time to aquire all published joint positions via callback
  if(!jointStatesSubscriber1 && !jointStatesSubscriber2)
  {
    ROS_WARN("Failed to subscribe to joint_states topic! - check to see if topic is being published.\n");
    state.params.startUpSequence = false;
  }
  else
  {    
    int spin = 2.0/state.params.timeDelta; //Max ros spin cycles to find joint positions
    while(spin--)
    {
      ros::spinOnce();
      r.sleep();
    }
  }  
  
  //Set joint position values and if needed wait for approval to use default joint position values 
  if (!state.jointPosFlag)
  {
    ROS_WARN("Failed to acquire ALL joint position values!\nPress B Button if you wish to continue with all unknown joint positions set to defaults . . .\n");
    
    //Wait for command to proceed using default joint position values
    while(!state.toggleLegState) //using toggleLegState for convenience only
    {
      ros::spinOnce();
      r.sleep();
    }      
    state.toggleLegState = false;
    state.setJointPositions(true); //Default joint positions used
  }
  else
  {
    state.setJointPositions(false); //Found joint positions used
  }
  
  //Check force data is available for impedanceControl
  if (state.params.impedanceControl)
  {
    if(tipForceSubscriber && state.params.impedanceInput == "tip_force")
    {
      state.useTipForce = true;
    }
    else if ((jointStatesSubscriber1 || jointStatesSubscriber2) && state.params.impedanceInput == "joint_effort")
    {
      state.useJointEffort = true;
    }
    else
    {
      ROS_WARN("Failed to subscribe to force data topic/s! Please check that topic is being published.\n");
      state.params.impedanceControl = false; 
    }
  }
  
  //Initiate state controller
  state.init();  
  
  //Enter ros loop
  while (ros::ok())
  {    
    //Controller shutdown command
    if (!state.params.startUpSequence && !state.startFlag)
    {
      ROS_INFO("Received shutdown order - shutting down the controller!\n");
      ros::shutdown();
    }    
    
    //State machine (state updating loop)
    state.loop();
    
    //Debugging publishers
    state.publishLegState(); 

    state.publishPose();
    state.publishIMURotation();
    state.publishBodyVelocity();

    state.publishRotationPoseError();
    state.publishTranslationPoseError();
    state.publishZTipError();
    
    
    //Debug using RVIZ
    if (state.params.debug_rviz)
    {
      state.RVIZDebugging();
    }
    
    //Publish desired joint angles
    state.publishJointValues();
    
    ros::spinOnce();
    r.sleep();

    state.debug.reset();
  }  
  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/


