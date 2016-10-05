#################################################################################################### 
####################################################################################################

Hexapod Parameters:

  /hexapod/parameters/hexapod_type:             
    String ID of the hexapod type associated with this set of config parameters. 
    Eg: "large_hexapod"
  
  /hexapod/parameters/time_delta: (default: 0.02)            
    Value used in setting ros loop frequency which denotes time period between cycles. (seconds)
  
  /hexapod/parameters/imu_compensation: (default: false)
    Bool sets whether IMU compensation mode is on/off. IMU compensation mode requires PID tuning 
    using /hexapod/parameters/pose_controller/imu_pose_compensation parameters.
  
  /hexapod/parameters/auto_compensation: (default: false)
    Bool sets whether Auto compenation mode is on/off. Auto compensation adjusts pitch/roll of body
    to compensate for body sag into lifted leg. Only implemented for Wave and Tripod gaits.
    
  /hexapod/parameters/manual_compensation: (default: true)
    Bool sets whether manual compensation mode is on/off. Manual compensation allows for the 
    manual posing of the body independent of the walking cycle.
  
  /hexapod/parameters/gait_type: (default: tripod_gait)
    String ID of the default gait to be used by the hexapod.
  
  /hexapod/parameters/physical_leg_offsets/root_offset_**:      
    3D vector describing the physical offset (in metres) of the root (first joint) of each leg from
    the body centre (origin).
    
  /hexapod/parameters/physical_leg_offsets/hip_offset_**:       
    3D vector describing the physical offset (in metres) of the hip (second joint) of each leg from
    the root joint of the same leg at a root joint value of zero.
    
  /hexapod/parameters/physical_leg_offsets/knee_offset_**:      
    3D vector describing the physical offset (in metres) of the knee (third joint) of each leg from
    the hip joint of the same leg at a hip joint value of zero.
    
  /hexapod/parameters/physical_leg_offsets/tip_offset_**:
    3D vector describing the physical offset (in metres) of the tip of each leg from the knee joint
    of the same leg at a knee joint value of zero.
  
  /hexapod/parameters/stance_leg_yaws:                  
    Denotes the required default stance offset angle (radians) of the yaw joint 'zero' from the 
    x-axis (lateral). 
    Eg: [0.785, 0, -0.785] sets the zero yaw position for the front legs to be 45 deg from the
      x-axis (lateral axis), 0 deg for the middle legs and -45 deg for the rear.

  /hexapod/parameters/physical_yaw_offsets:             
    Denotes the phyical yaw joint motor axis offset (radians) from the x-axis (lateral axis). 
    Eg: [0.523, 0.0, -0.523] means the front yaw motors are aligned 60 deg from the x-axis, the 
      middle motors are aligned parallel to the x-axis and the rear are mounted -60 deg from the 
      x-axis.
  
  /hexapod/parameters/joint_limits/yaw_limits:          
    Limits for angle of first joint of each leg position (radians). 
    Eg: [0.785, 0.523, 1.57] gives: 
      front legs a yaw range of -45->45 deg, 
      middle legs a yaw range of -60->60 deg and 
      rear legs a yaw range of -90->90 deg.
    
  /hexapod/parameters/joint_limits/hip_limits:          
    Upper and lower limits for each and every hip joint (radians). 
    Eg: [-0.785, 1.57] gives each hip joint a range of -45->90 deg.
    
  /hexapod/parameters/joint_limits/knee_limits:         
    Upper and lower limits for each and every knee joint (radians). 
    Eg: [-0.523, 3.14] gives each hip joint a range of -60->180 deg.
  
  /hexapod/parameters/joint_max_angular_speeds:         
    Maximum angular speeds for joint angles ([yaw, hip, knee]). If the calculated tip trajectory 
    requires exceeding these values then joint velocities are capped at max.
   
  /hexapod/parameters/dynamixel_interface: (default: true)
    Determines which motor interface to use (dynamixel or dynamixelPro) - set false if using 
    dynamixel pro motors and set true for all simulations.

#################################################################################################### 
    
Walk Controller Parameters:

  /hexapod/parameters/walk_controller/step_frequency: (default: 1.0)
    Number of full steps (full swing/stance transition) taken per second (in TRIPOD mode) (Hz). For
    other gaits the effective step frequency is adjusted according to swing ratio
    (see WalkController::setGaitParams())
    
  /hexapod/parameters/walk_controller/step_clearance: (default: 0.1)
    Defines the max step clearance height above ground level as a percentage of max body-height. 
    Eg: 0.1 gives 10% of max body height
    
  /hexapod/parameters/walk_controller/stance_depth: (default: 0.0)
    Defines the max 'depth' that the stance phase of the each step attains as a percentage of max
    body height. 
    
  /hexapod/parameters/walk_controller/body_clearance: (default: -1.0)
    Defines the max body clearance height above ground level as a percentage of max body-height. 
    A value of -1.0 sets the body clearance to be optimally calculated to maximise tip footprint 
    radius.
    
  /hexapod/parameters/walk_controller/leg_span_scale: (default: 1.0)
    Modifies the effective horizontal range of each leg in order to force a more narrow (<1.0) or 
    wide (>1.0) stance. The controller will crash if the forced leg span causes the body clearance 
    to no longer be achievable.
 
  /hexapod/parameters/walk_controller/max_linear_acceleration: (default: -1.0)
    Maximum linear acceleration of the body. Set to -1.0 to have it auto calculated for conservative
    operation.
    
  /hexapod/parameters/walk_controller/max_angular_acceleration: (default: -1.0)
   Maximum angular acceleration of the body. Set to -1.0 to have it auto calculated for conservative
   operation.
  
  /hexapod/parameters/walk_controller/footprint_downscale: (default: 0.8)
    Scales operating tip footprint workspace down from maximum calculated footprint to give the tip a
    factor of safety for instances where is may need to stray from the operating footprint in order 
    to maintain body velocity.
  
  /hexapod/parameters/walk_controller/interface_setup_speed: (default: 2.5)
    Sets speed of motors (rad/s).

####################################################################################################  
  
Pose Controller Parameters:

  /hexapod/parameters/pose_controller/start_up_sequence: (default: false)
    Determines if hexapod uses predefined unpacking/packing & startup/shutdown procedures. If false 
    the hexapod will move legs DIRECTLY from initial positions to required positions for walking.
    
  /hexapod/parameters/pose_controller/move_legs_sequentially: (default: false)
    Determines whether legs are moved sequentially or simultaneously in the chosen startup 
    procedure.
    
  /hexapod/parameters/pose_controller/time_to_start: (default: 12.0)
    Determines the length of time (seconds) in which to complete a DIRECT startup sequence 
    (start_up_sequence == false)
    
  /hexapod/parameters/pose_controller/imu_pose_compensation/**_compensation/**_gain:
    PID gains for translational/rotational imu pose compensation. WARNING: requires tuning to prevent
    unstable compensation, check all values before using imu compensation.

  /hexapod/parameters/pose_controller/auto_pose_compensation/**_amplitude: (default: 0.025)
    Sets amplitude of pitch, roll and vertical translation used in auto_compensation mode

  /hexapod/parameters/pose_controller/manual_pose_compensation/max_pose_time: (default: 1.0)
    Sets the maximum (and default) time (seconds) in which to complete manual pose actions.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_**:
    Sets the amplitude of the maximum roll/pitch/yaw variables used in manual rotation posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_**: (default: 0.05)
    Sets the amplitude of the min/max x/y/z translation variable used in manual translation posing.
    
  /hexapod/parameters/pose_controller/packed_joint_positions/**_packed_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "packed" state. (radians)
    
  /hexapod/parameters/pose_controller/unpacked_joint_positions/**_unpacked_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "unpacked" state. (radians)
    
####################################################################################################  

Impedance Control Parameters:

  /hexapod/parameters/impedance_controller/impedance_control: (default: true)
    Determines if impedance control is currently turned on/off.
    
  /hexapod/parameters/impedance_controller/impedance_input: (default: joint_effort)
    Determines input method for impedance. Either 'joint_effort' for using effort values from joint
    motors OR 'tip_force' for using pressure readings from tip sensors (MAX - Large Hexapod)
    
  /hexapod/parameters/impedance_controller/virtual_mass:
    Virtual mass variable used in impedance controller spring-mass-damper virtualisation.
    
  /hexapod/parameters/impedance_controller/integrator_step_time:
    Time step used in impedance controller function.
    
  /hexapod/parameters/impedance_controller/virtual_stiffness:
    Virtual stiffness variable used in impedance controller spring-mass-damper virtualisation.
    
  /hexapod/parameters/impedance_controller/virtual_damping_ratio:
    Virtual damping ratio variable used in impedance controller spring-mass-damper virtualisation.
    
  /hexapod/parameters/impedance_controller/force_gain:
    Gain used to scale impedance values used in impedance controller.
    
####################################################################################################  

Debug Parameters:

  /hexapod/debug_parameters/testing: (default: false)
    Turns on/off ability to perform 'test' runs with constant velocity input over a set time period.
    Tests start/stop using X Button.
    
  /hexapod/debug_parameters/test_time_length: (default: 0.0)
    Sets time length for test. A zero value sets an infinite test time length which can only be 
    manually ended.
    
  /hexapod/debug_parameters/test_velocity: (default: [0.0, 1.0])
    Sets constant velocity input used in test. Note default is a full forward velocity input
    
  /hexapod/debug_parameters/console_verbosity: (default: info)
    Sets rosconsole verbosity levels. Options include: 'debug', 'info', 'warning', 'error', 'fatal'
    
  /hexapod/debug_parameters/debug_move_to_joint_position: (default: false)
    Allows debug output from moveToJointPosition function in poseController.
    
  /hexapod/debug_parameters/debug_step_to_position: (default: false)
    Allows debug output from stepToPosition function in poseController.
    
  /hexapod/debug_parameters/debug_swing_trajectory: (default: false)
    Allows debug output from swing part of updatePosition function in walkController.
    
  /hexapod/debug_parameters/debug_stance_trajectory: (default: false)
    Allows debug output from stance part of updatePosition function in walkController.
    
  /hexapod/debug_parameters/debug_manual_compensation_translation: (default: false)
    Allows debug output from translation part of manualCompensation function in poseController.
    
  /hexapod/debug_parameters/debug_manual_compensation_rotation: (default: false)
    Allows debug output from rotation part of manualCompensation function in poseController.
    
  /hexapod/debug_parameters/debug_rviz: (default: false)
    Turns on output for use in simulation in rviz.
    
####################################################################################################  

Gait Parameters:

  /hexapod/gait_parameters/*gait_name*/stance_phase:      
    Ratio of step phase in the 'stance' state (on ground)
    
  /hexapod/gait_parameters/*gait_name*/swing_phase:       
    Ratio of step phase in the 'swing' state (in air)
    
  /hexapod/gait_parameters/*gait_name*/phase_offset:      
    Base ratio of step phase each leg is offset from each other.
    
  /hexapod/gait_parameters/*gait_name*/offset_multiplier: 
    Multiplier for phase_offset which determines phase offset for each leg. 
    Offset multiplier values are assigned in the following order: 
      front_left (leg = 0, side = 0), 
      front_right (leg = 0, side = 1),
      middle_left (leg = 1, side = 0),
      middle_right (leg = 1, side = 1),
      rear_left (leg = 2, side = 0),
      rear_right (leg = 2, side = 1)

  Eg:     tripod_gait has stance:swing:offset ratio of 50:40:45. 
          Assuming a total phase length of 90: 
            50 iterations (66.67%) of the step are stance, 
            40 iterations (33.33%) of the step are swing
                        
            The base phase offset is 45 iterations (16.67%).
            With a offset_multiplier of [0,1,1,0,0,1] legs:
              front_left, middle_right and rear_left have an effective offset
              of 0*45 = 0 iterations (0%)
              front_right, middle_left and rear_right have an effective offset
              of 1*45 = 50 iterations (50%)
            
  Eg:     wave_gait has stance:swing:offset ratio of 80:10:15. 
          Assuming a total phase length of 90: 
            80 iterations (88.89%) of the step are stance, 
            10 iterations (11.11%) of the step are swing 
            
            The base phase offset is 15 iterations (16.67%).
            With a offset_multiplier of [2,5,3,0,4,1] legs:
              front_left has an offset of 2*15 = 30 iterations (33.33%)
              front_right has an offset of 5*15 = 75 iterations (16.67%)
              middle_left has an offset of 3*15 = 45 iterations (50%)
              middle_right has an offset of 0*15 = 0 iterations (0%)
              rear_left has an offset of 4*15 = 60 iterations (66.67%)
              rear_right has an offset of 1*15 = 15 iterations (83.33%)
              
            Note that for this example the swing state starts at iteration 40*
            and therefore these offsets give a swing order of:
              front_left, rear_right, middle_right, front_right, rear_left, front_left etc.
              
          *swing_start = stance_phase(80)/2 = 40
          
####################################################################################################
#################################################################################################### 