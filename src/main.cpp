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
  Parameters* params = state.getParameters();

  bool set_logger_level_result = false;
  switch(params->console_verbosity)
  {
    case("debug"):
      set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
      break;
    case("info"):
      set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
      break;
    case("warning"):
      set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
      break;
    case("error"):
      set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
      break;
    case("fatal"):
      set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
      break;
  }

  if (set_logger_level_result)
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Set ros rate from params
  ros::Rate r(roundToInt(1.0 / params->time_delta));

  // Display warning message concerning use of correct motor interface
  if (params->use_dynamixel_pro_interface)
  {
    ROS_WARN("Using dynamixel PRO motor interface - will only work with Large Hexapod Robot (not simulations or small "
             "hexapod platforms).\nSet use_dynamixel_pro_interface to false in config file if not using Large Hexapod.\n");    
  }
  else
  {
    if (params->hexapod_type == "large_hexapod")
    {
      ROS_WARN("Using dynamixel motor interface - will not work with Large Hexapod (simulations only).\nSet "
               "use_dynamixel_pro_interface to true in config file if using Large Hexapod.\n");
    }
  }

  // RVIZ simulation warning message
  if (params->debug_rviz)
  {
    ROS_WARN("DEBUGGING USING RVIZ - CODE IS CPU INTENSIVE.\n");
  }
  
  //Check topics are correctly subscribed
  if (!state.receivingJointStates())
  {
    ROS_WARN("Failed to subscribe to joint_states topic! - check to see if topic is being published.\n");
    params->start_up_sequence = false;
  }
  else if ((params->imu_compensation || params->inclination_compensation) && !state.receivingImuData())
  {
    ROS_WARN("Failed to subscribe to imu_data topic! - check to see if topic is being published.\n");
    params->imu_compensation = false;
    params->inclination_compensation = false;
  }
  else if (params->impedance_control)
  {
    if ((params->impedance_input == "tip_force" && !state.receivingTipForces()) ||
       (params->impedance_input == "joint_effort" && !state.receivingJointStates()))
    {
      ROS_WARN("Failed to subscribe to force data topic/s! Please check that topic is being published.\n");
      params->impedance_control = false;
    }
  }

  // Wait specified time to aquire all published joint positions via callback //TBD Still needed?, consider cutting
  int spin = 2.0 / params->time_delta;  // Max ros spin cycles to find joint positions
  while (spin--)
  {
    ros::spinOnce();
    r.sleep();
  }

  // Set start message
  bool use_default_joint_positions = !state.hasAllJointPositions();
  std::string start_message;
  if (use_default_joint_positions)
  {
    start_message = "Failed to acquire ALL joint position values!\nPress 'Start' to run controller with all unknown joint positions set to defaults . . .\n";
  }
  else
  {
    start_message = "Press 'Start' to run controller . . .\n";
  }
  
  // Loop waiting for start button press
  while (state.getSystemState() == WAITING_FOR_USER)
  {
    ROS_WARN_THROTTLE(THROTTLE_PERIOD, start_message.c_str());
    ros::spinOnce();
    r.sleep();
  }
  state.setJointPositions(use_default_joint_positions);

  state.init();
  while (ros::ok())
  {
    state.loop();
    state.publishLegState();
    state.publishPose();
    state.publishIMUData();
    state.publishBodyVelocity();
    state.publishRotationPoseError();
    state.publishTranslationPoseError();
    state.publishZTipError();
    if (params->debug_rviz)
    {
      state.RVIZDebugging();
    }
    state.publishJointValues();
    ros::spinOnce();
    r.sleep();
    state.resetDebug();
  }
  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
