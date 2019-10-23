//*********************************************************************************************************************
// * CSIRO Autonomous Systems Laboratory
// * Queensland Centre for Advanced Technologies
// * PO Box 883, Kenmore, QLD 4069, Australia
// *
// * (c) Copyright CSIRO 2017
// *
// * All rights reserved, no part of this program may be used
// * without explicit permission of CSIRO
//*********************************************************************************************************************

#include "syropod_highlevel_controller/state_controller.h"

#define ACQUISTION_TIME 10 ///< Max time controller will wait to acquire intitial joint states (seconds)

/*******************************************************************************************************************//**
 * Main loop. Sets up ros environment including the node handle, rosconsole messaging, loop rate etc. Also creates and
 * initialises the 'StateController', calls the state controller loop and publishers, and sends messages for the user
 * interface.
***********************************************************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "shc");
  ros::NodeHandle n;

  StateController state;
  const Parameters& params = state.getParameters();

  bool set_logger_level_result = false;

  if (params.console_verbosity.data == std::string("debug"))
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  }
  else if (params.console_verbosity.data == "info")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  }
  else if (params.console_verbosity.data == "warning")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
  }
  else if (params.console_verbosity.data == "error")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  }
  else if (params.console_verbosity.data == "fatal")
  {
    set_logger_level_result = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  }

  if (set_logger_level_result)
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Set ros rate from params
  ros::Rate r(roundToInt(1.0 / params.time_delta.data));

  // Wait specified time to aquire all published joint positions via callback
  int spin = ACQUISTION_TIME / params.time_delta.data; //Spin cycles from time
  while (spin--)
  {
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nAcquiring robot state . . .\n");
    // End wait if joints are intitialised or debugging in rviz (joint states will never initialise).
    if (state.jointPositionsInitialised())
    {
      spin = 0;
    }
    ros::spinOnce();
    r.sleep();
  }

  // Set start message
  std::string start_message;
  bool use_default_joint_positions;
  if (state.jointPositionsInitialised())
  {
    start_message = "\nPress 'Logitech' button to start controller . . .\n";
    use_default_joint_positions = false;
  }
  else
  {
    start_message = "\nPress 'Logitech' button to run controller initialising unknown positions to defaults . . .\n";
    use_default_joint_positions = true;
  }

  // Loop waiting for start button press
  while (state.getSystemState() == SUSPENDED)
  {
    if (use_default_joint_positions)
    {
      ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nFailed to initialise joint position values!\n");
    }
    ROS_INFO_THROTTLE(THROTTLE_PERIOD, "%s", start_message.c_str());
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("\nController started. Press START/BACK buttons to transition state of robot.\n");

  state.init(); // Must be initialised before initialising model with current joint state
  state.initModel(use_default_joint_positions);
  
  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener(transform_buffer_);

  // Main loop
  while (ros::ok())
  {
    if (state.getSystemState() != SUSPENDED)
    {
      state.loop();
      state.publishLegState();
      state.publishVelocity();
      state.publishPose();
      state.publishWalkspace();
      state.publishRotationPoseError();
      state.publishFrameTransforms();

      if (params.debug_rviz.data)
      {
        state.RVIZDebugging();
      }

      state.publishDesiredJointState();
    }
    else
    {
      ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController suspended. Press Logitech button to resume . . .\n");
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
