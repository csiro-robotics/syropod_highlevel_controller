#include "simple_hexapod_controller/stateController.h"
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
  ros::Subscriber bodyVelocitySubscriber = n.subscribe("hexapod_remote/desired_velocity", 1, &StateController::bodyVelocityInputCallback, &state);
  ros::Subscriber primaryTipVelocitySubscriber = n.subscribe("hexapod_remote/primary_tip_velocity", 1, &StateController::primaryTipVelocityInputCallback, &state);
  ros::Subscriber secondaryTipVelocitySubscriber = n.subscribe("hexapod_remote/secondary_tip_velocity", 1, &StateController::secondaryTipVelocityInputCallback, &state);
  ros::Subscriber bodyPoseSubscriber = n.subscribe("hexapod_remote/desired_pose", 1, &StateController::bodyPoseInputCallback, &state);
  
  ros::Subscriber systemStateSubscriber = n.subscribe("hexapod_remote/system_state", 1, &StateController::systemStateCallback, &state);
  ros::Subscriber gaitSelectionSubscriber = n.subscribe("hexapod_remote/gait_selection", 1, &StateController::gaitSelectionCallback, &state);
  ros::Subscriber posingModeSubscriber = n.subscribe("hexapod_remote/posing_mode", 1, &StateController::posingModeCallback, &state);
  ros::Subscriber cruiseControlSubscriber = n.subscribe("hexapod_remote/cruise_control_mode", 1, &StateController::cruiseControlCallback, &state);
  ros::Subscriber autoNavigationSubscriber = n.subscribe("hexapod_remote/auto_navigation_mode", 1, &StateController::autoNavigationCallback, &state);
  
  ros::Subscriber parameterSelectionSubscriber = n.subscribe("hexapod_remote/parameter_selection", 1, &StateController::parameterSelectionCallback, &state);
  ros::Subscriber parameterAdjustmentSubscriber = n.subscribe("hexapod_remote/parameter_adjustment", 1, &StateController::parameterAdjustCallback, &state);
  
  ros::Subscriber primaryLegSelectSubscriber = n.subscribe("hexapod_remote/primary_leg_selection", 1, &StateController::primaryLegSelectionCallback, &state);
  ros::Subscriber primaryLegStateSubscriber = n.subscribe("hexapod_remote/primary_leg_state", 1, &StateController::primaryLegStateCallback, &state);
  ros::Subscriber secondaryLegSelectSubscriber = n.subscribe("hexapod_remote/secondary_leg_selection", 1, &StateController::secondaryLegSelectionCallback, &state);
  ros::Subscriber secondaryLegStateSubscriber = n.subscribe("hexapod_remote/secondary_leg_state", 1, &StateController::secondaryLegStateCallback, &state);
  
  ros::Subscriber poseResetSubscriber = n.subscribe("hexapod_remote/pose_reset_mode", 1, &StateController::poseResetCallback, &state);  
  
  //Motor and other sensor topic subscriptions
  ros::Subscriber imuDataSubscriber = n.subscribe("ig/imu/data_ned", 1, &StateController::imuCallback, &state);  
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
  
  //ASC magnetic feet specific publishers
  state.ascLegStatePublishers[0][0] = n.advertise<std_msgs::Bool>("leg_state_front_left_bool", 1);
  state.ascLegStatePublishers[0][1] = n.advertise<std_msgs::Bool>("leg_state_front_right_bool", 1);
  state.ascLegStatePublishers[1][0] = n.advertise<std_msgs::Bool>("leg_state_middle_left_bool", 1);
  state.ascLegStatePublishers[1][1] = n.advertise<std_msgs::Bool>("leg_state_middle_right_bool", 1);
  state.ascLegStatePublishers[2][0] = n.advertise<std_msgs::Bool>("leg_state_rear_left_bool", 1);
  state.ascLegStatePublishers[2][1] = n.advertise<std_msgs::Bool>("leg_state_rear_right_bool", 1);  
  
  state.posePublisher = n.advertise<geometry_msgs::Twist>("/hexapod/pose", 1000);
  state.IMUDataPublisher = n.advertise<std_msgs::Float32MultiArray>("/hexapod/imu_data", 1000);
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
  
  //Loop waiting for start button press 
  while(state.newSystemState == OFF)
  {
    if (!state.recievedAllJointPositions)
    {
      ROS_WARN_THROTTLE(THROTTLE_PERIOD, "Failed to acquire ALL joint position values!\nPress 'Start' to run controller with all unknown joint positions set to defaults . . .\n");
    }
    else
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "Press 'Start' to run controller . . .\n"); 
    }    
    ros::spinOnce();
    r.sleep();
  }
  state.setJointPositions(!state.recievedAllJointPositions);
  
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
    //State machine (state updating loop)
    state.loop();
    
    //Debugging publishers
    state.publishLegState(); 

    state.publishPose();
    state.publishIMUData();
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


