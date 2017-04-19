#include "simple_hexapod_controller/stateController.h"

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

  if (params->console_verbosity.data == std::string("debug"))
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  }
  else if (params->console_verbosity.data == "info")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  }
  else if (params->console_verbosity.data == "warning")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
  }
  else if (params->console_verbosity.data == "error")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  }
  else if (params->console_verbosity.data == "fatal")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  }

  if (set_logger_level_result)
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Set ros rate from params
  ros::Rate r(roundToInt(1.0 / params->time_delta.data));

  //Check topics are correctly subscribed
  if (!state.receivingJointStates())
  {
    ROS_WARN("\nFailed to subscribe to joint_states topic! - check to see if topic is being published.\n");
    //params->start_up_sequence.data = false;
  }
  else if ((params->imu_compensation.data || params->inclination_compensation.data)
           && !state.receivingImuData())
  {
    ROS_WARN("\nFailed to subscribe to imu_data topic! - check to see if topic is being published.\n");
    params->imu_compensation.data = false;
    params->inclination_compensation.data = false;
  }
  else if (params->impedance_control.data)
  {
    if ((params->use_joint_effort.data && !state.receivingTipForces()) ||
        (!params->use_joint_effort.data && !state.receivingJointStates()))
    {
      ROS_WARN("\nFailed to subscribe to force data topic/s! Please check that topic is being published.\n");
      params->impedance_control.data = false;
    }
  }

  // Wait specified time to aquire all published joint positions via callback //TBD Still needed?, consider cutting
  int spin = 5.0 / params->time_delta.data;  // Max ros spin cycles to find joint positions

  while (spin--)
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nCommunicating with hardware . . .\n");
    ros::spinOnce();
    r.sleep();
  }

  // Set start message
  std::string start_message;
  bool use_default_joint_positions;

  if (state.areJointPostionsInitialised())
  {
    start_message = "\nPress 'Logitech' button to start controller . . .\n";
    use_default_joint_positions = false;
  }
  else
  {
    start_message = "\nFailed to initialise joint position values!\nPress 'Logitech' button to run controller initialising unknown joint positions to defaults . . .\n";
    use_default_joint_positions = true;
    //params->start_up_sequence.data = false;
  }

  // Loop waiting for start button press
  while (state.getSystemState() == SUSPENDED)
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "%s", start_message.c_str());
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("\nController started. Press START/BACK buttons to transition state of robot.\n");

  state.init(); // Must be initialised before initialising model with current joint state
  state.initModel(use_default_joint_positions);

  while (ros::ok())
  {
    if (state.getSystemState() != SUSPENDED)
    {
      state.loop();
      state.publishLegState();
      state.publishPose();
      //state.publishIMUData();
      state.publishBodyVelocity();

      //state.publishRotationPoseError();
      //state.publishTranslationPoseError();
      if (params->debug_rviz.data)
      {
        state.RVIZDebugging(params->debug_rviz_static_display.data);
      }

      state.publishDesiredJointState();
    }
    else
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController suspended. Press Logitech button to resume . . .\n");
    }

    ros::spinOnce();
    r.sleep();
    state.resetDebug();
  }

  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
