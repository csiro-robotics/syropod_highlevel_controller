#include "../include/simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");  
  
  StateController state(n);
  
  ros::Subscriber velocitySubscriber = n.subscribe("hexapod_remote/desired_velocity", 1, &StateController::joypadVelocityCallback, &state);
  ros::Subscriber poseSubscriber = n.subscribe("hexapod_remote/desired_pose", 1, &StateController::joypadPoseCallback, &state);
  //ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data", 1, &StateController::imuCallback);
  //ros::Subscriber imuControlSubscriber = n.subscribe("/joy", 1, &StateController::imuControllerCallback);
  ros::Subscriber legSelectSubscriber = n.subscribe("hexapod_remote/leg_selection", 1, &StateController::legSelectionCallback, &state);
  ros::Subscriber legStateSubscriber = n.subscribe("hexapod_remote/leg_state_toggle", 1, &StateController::legStateCallback, &state);
  ros::Subscriber paramSelectionSubscriber = n.subscribe("hexapod_remote/param_selection", 1, &StateController::paramSelectionCallback, &state);
  ros::Subscriber paramAdjustmentSubscriber = n.subscribe("hexapod_remote/param_adjust", 1, &StateController::paramAdjustCallback, &state);
  
  ros::Subscriber startSubscriber = n.subscribe("hexapod_remote/start_state", 1, &StateController::startCallback, &state);
  ros::Subscriber gaitSelectSubscriber = n.subscribe("hexapod_remote/gait_mode", 1, &StateController::gaitSelectionCallback, &state);
  ros::Subscriber tipForceSubscriber = n.subscribe("/motor_encoders", 1, &StateController::tipForceCallback, &state);
  ros::Subscriber jointStatesSubscriber;
  
  //Attempt to subscribe to one of two possible joint state topics
  jointStatesSubscriber = n.subscribe("/hexapod/joint_states", 1, &StateController::jointStatesCallback, &state);
  if (!jointStatesSubscriber)
  {
    jointStatesSubscriber = n.subscribe("/hexapod_joint_state", 1, &StateController::jointStatesCallback, &state); 
  }
  
  //ros::Publisher controlPub = n.advertise<geometry_msgs::Vector3>("controlsignal", 1000);
  state.tipPositionPublishers[0][0] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/front_left_tip_positions", 1000);
  state.tipPositionPublishers[0][1] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/front_right_tip_positions", 1000);
  state.tipPositionPublishers[1][0] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/middle_left_tip_positions", 1000);
  state.tipPositionPublishers[1][1] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/middle_right_tip_positions", 1000);
  state.tipPositionPublishers[2][0] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/rear_left_tip_positions", 1000);
  state.tipPositionPublishers[2][1] = n.advertise<std_msgs::Float32MultiArray>("/hexapod/rear_right_tip_positions", 1000);
  
  //Set ros rate from params
  ros::Rate r(roundToInt(1.0/state.params.timeDelta));
  
  //Display warning message concerning use of correct motor interface
  if (state.params.dynamixelInterface)
  {
    if (state.params.hexapodType == "large_hexapod")
    {
      cout << "WARNING: Using dynamixel motor interface - will not work with Large Hexapod (simulations only)." << endl; 
      cout << "Set dynamixel_interface to false in config file if using Large Hexapod.\n" << endl;
    }
  }
  else
  {
    cout << "WARNING: Using dynamixel PRO motor interface - will only work with Large Hexapod Robot (not simulations or small hexapod platforms)." << endl;
    cout << "Set dynamixel_interface to true in config file if not using Large Hexapod.\n" << endl;
  }
  
  //RVIZ simulation warning message
  if (state.params.debug_rviz)
  {
    cout << "WARNING: DEBUGGING USING RVIZ - CODE IS CPU INTENSIVE." << endl;
  }
  
  //Loop waiting for start button press
  cout << "Press 'Start' to run controller . . ." << endl;  
  while(!state.startFlag)
  {
    ros::spinOnce();
    r.sleep();
  }
  cout << "Controller started.\n" << endl;
  
  //Wait specified time to aquire all published joint positions via callback
  if(!jointStatesSubscriber)
  {
    cout << "WARNING: Failed to subscribe to joint_states topic! - check to see if topic is being published.\n" << endl;
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
    cout << "WARNING: Failed to acquire ALL joint position values!" << endl;
    cout << "Press B Button if you wish to continue with all unknown joint positions set to defaults . . .\n" << endl;
    
    //Wait for command to proceed using default joint position values
    while(!state.toggleLegState) //using toggleLegState for convenience only
    {
      ros::spinOnce();
      r.sleep();
    }      
    state.toggleLegState = false;
    state.setJointPositions(true);
  }
  else
  {
    state.setJointPositions(false);
  }
  
  //Check force data is available for impedanceControl
  if (state.params.impedanceControl)
  {
    if(tipForceSubscriber)
    {
      state.useTipForce = true;
    }
    else if (jointStatesSubscriber)
    {
      state.useJointEffort = true;
    }
    else
    {
      cout << "Failed to subscribe to force data topic/s! Please check that topic is being published.\n" << endl;
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
    
    //Tip position publisher for debugging
    state.publishTipPositions();    
    
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


