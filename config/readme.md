#Syropod Parameter File Readme 

##General Parameters:

###/hexapod/parameters/hexapod_type:             
    String ID of the hexapod type associated with this set of config parameters.
	    (type: string)(example: "zee")
    
###hexapod/parameters/time_delta:            
	Value used in setting ros loop frequency which denotes time period between cycles.
		(type: double)(default: 0.02)(unit: seconds)
	
###/hexapod/parameters/imu_compensation:
    Sets whether IMU-Compensation mode is on/off. IMU compensation mode requires PID tuning and IMU correctly set up.
	    (type: bool)(default: false)
    
###/hexapod/parameters/auto_compensation: 
    Sets whether Auto-Compenation mode is on/off. Auto compensation adjusts pitch/roll of body to compensate for body sag into lifted leg.
	    (type: bool)(default: false)
    
###/hexapod/parameters/manual_compensation:
    Sets whether Manual-Compensation mode is on/off. Manual compensation allows for the manual posing of the body independent of the walking cycle and additive to auto-compensation.
    (type: bool)(default: true)
  
###/hexapod/parameters/gait_type:
    String ID of the default gait to be used by the hexapod.
	    (type: string)(default: tripod_gait)

##Physical Parameters

###/hexapod/parameters/physical_leg_offsets/root_offset_**:      
    3D vector describing the physical offset of the root (first joint) of each leg from
    the body centre (origin).
	    (type: [double, double, double])(unit: metres)
    
###/hexapod/parameters/physical_leg_offsets/hip_offset_**:       
    3D vector describing the physical offset of the hip (second joint) of each leg from
    the root joint of the same leg at a root joint value of zero.
	    (type: [double, double, double])(unit: metres)
    
###/hexapod/parameters/physical_leg_offsets/knee_offset_**:      
    3D vector describing the physical offset of the knee (third joint) of each leg from
    the hip joint of the same leg at a hip joint value of zero.
	    (type: [double, double, double])(unit: metres)
    
###/hexapod/parameters/physical_leg_offsets/tip_offset_**:
    3D vector describing the physical offset of the tip of each leg from the knee joint
    of the same leg at a knee joint value of zero.
	    (type: [double, double, double])(unit: metres)
  
###/hexapod/parameters/stance_leg_yaws:                  
    Denotes the required default stance offset angle of the yaw joint 'zero' from the 
    x-axis (lateral).
	    (type: [double, double, double])(unit: radians)
	    (example: [0.785, 0, -0.785] sets the zero yaw position for the front legs to be 45 deg from the x-axis (lateral axis), 0 deg for the middle legs and -45 deg for the rear.)

###/hexapod/parameters/physical_yaw_offsets:             
    Denotes the physical yaw joint motor axis offset from the x-axis (lateral axis).
	    (type: [double, double, double])(unit: radians)
	    (example: [0.523, 0.0, -0.523] means the front yaw motors are aligned 60 deg from the x-axis, the middle motors are aligned parallel to the x-axis and the rear are mounted -60 deg from the x-axis.)

###/hexapod/parameters/physical_knee_offset:
	Denotes the physical knee joint motor axis offset.
		 (type: double)(unit: radians)
  
###/hexapod/parameters/joint_limits/yaw_limits:          
    Limits for angle of first joint of each leg position.
	    (type: [double, double, double])(unit: radians) 
	    (example: [0.785, 0.523, 1.57] gives: front legs a yaw range of -45->45 deg, middle legs a yaw range of -60->60 deg and rear legs a yaw range of -90->90 deg.)
    
###/hexapod/parameters/joint_limits/hip_limits:          
    Upper and lower limits for each and every hip joint. 
	    (type: [double, double])(unit: radians) 
	    (example: [-0.785, 1.57] gives each hip joint a range of -45->90 deg.)
    
###/hexapod/parameters/joint_limits/knee_limits:         
    Upper and lower limits for each and every knee joint.
	    (type: [double, double])(unit: radians)  
	    (example: [-0.523, 3.14] gives each hip joint a range of -60->180 deg.)
  
###/hexapod/parameters/joint_max_angular_speeds:         
    Maximum angular speeds for joint angles ([yaw, hip, knee]). If the calculated tip trajectory requires exceeding these values then joint velocities are capped at max.
	    (type: [double, double, double])(unit: radians/s) 
   
###/hexapod/parameters/dynamixel_interface:
    Determines which motor interface to use (dynamixel or dynamixelPro) - set false if using dynamixel pro motors and set true for all simulations.
	    (type: bool)(default: true)

##Walk Controller Parameters:

###/hexapod/parameters/walk_controller/step_frequency:
    Number of full steps (full swing/stance transition) taken per second (in TRIPOD mode). For other gaits the effective step frequency is adjusted according to swing ratio defined in the gait parameters.
	    (type: double) (default: 1.0)(unit: steps/s (Hz))
    
###/hexapod/parameters/walk_controller/step_clearance: 
    Defines the max step clearance height above ground level as a percentage of max body-height.
	    (type: double)(default:0.1)(unit: percentage (i.e. 0.0->1.0))
    
###/hexapod/parameters/walk_controller/stance_depth: 
    Defines the max 'depth' that the stance phase of the each step attains as a percentage of max body height. 
	    (type: double)(default:0.0)(unit: percentage (i.e. 0.0->1.0))
    
###/hexapod/parameters/walk_controller/body_clearance: 
    Defines the max body clearance height above ground level as a percentage of max body-height. A value of -1.0 sets the body clearance to be optimally calculated to maximise tip footprint radius and therefore allowable stride length.
	    (type: double)(default: -1.0)(unit: percentage (i.e. 0.0->1.0))
    
###/hexapod/parameters/walk_controller/leg_span_scale:
    Modifies the effective horizontal range of each leg in order to force a more narrow (<1.0) or wide (>1.0) stance. The controller will crash if the forced leg span causes the body clearance to no longer be achievable.
	    (type: double)(default: 1.0)(unit: percentage (i.e. 0.0->1.0))
 
###/hexapod/parameters/walk_controller/max_linear_acceleration:
    Maximum linear acceleration of the body. Set to -1.0 to have it auto calculated for conservative operation.
	    (type: double)(default: -1.0)(unit: m/s/s)
    
###/hexapod/parameters/walk_controller/max_angular_acceleration: 
	Maximum angular acceleration of the body. Set to -1.0 to have it auto calculated for conservative operation.
		(type: double)(default: -1.0)(unit: rad/s/s)
  
###/hexapod/parameters/walk_controller/footprint_downscale:
    Scales operating tip footprint workspace down from maximum calculated footprint to give the tip a factor of safety for instances where is may need to stray from the operating footprint in order to maintain body velocity.
	    (type: double)(default: 0.8)(unit: percentage (i.e. 0.0->1.0))
  
###/hexapod/parameters/walk_controller/interface_setup_speed:
	Sets speed of motors (rad/s).
		(type: double)(default: 2.5)(unit: rad/s)

###/hexapod/parameters/walk_controller/velocity_input_mode:
	Determines if velocity input mode is 'throttle' or 'real'.
	Throttle mode means the walk controller expects a value of 0.0->1.0  for a percentage of the max attainable speed.
	Real mode means the walk controller expects a real-world velocity input (metres/s & rad/s) and will attempt to meet that input (within limitations)
		(type: string)(default: throttle)
		
##Pose Controller Parameters:

###/hexapod/parameters/pose_controller/start_up_sequence:
    Determines if hexapod uses predefined unpacking/packing & startup/shutdown procedures. If false the hexapod will move legs DIRECTLY from initial positions to required positions for walking.
		(type: bool)(default: false)
    
###/hexapod/parameters/pose_controller/move_legs_sequentially:
    Determines whether legs are moved sequentially or simultaneously in the chosen startup 
    procedure.
	    (type: bool)(default: false)
    
###/hexapod/parameters/pose_controller/time_to_start:
    Determines the length of time in which to complete a DIRECT startup sequence 
    (i.e. start_up_sequence == false)
	    (type: bool)(default: 12.0)(unit: seconds)
    
###/hexapod/parameters/pose_controller/imu_pose_compensation/**_compensation/**_gain:
    PID gains for translational/rotational imu pose compensation. WARNING: requires tuning to prevent unstable compensation, check all values before using imu compensation.
	    (type: double)(default: 0.0)

###/hexapod/parameters/pose_controller/auto_pose_compensation/**_amplitude: 
    Sets amplitude of pitch, roll and vertical translation used in auto_compensation mode.
	    (type: double)(default: 0.01)(unit: radians or metres)

###/hexapod/parameters/pose_controller/manual_pose_compensation/max_translation: 
    Sets the maximum translational posing of the body along the x,y,z axes.
	    (type: [double, double, double])(unit: metres)
	    
###/hexapod/parameters/pose_controller/manual_pose_compensation/max_rotation: 
    Sets the maximum rotational posing of the body around the x,y,z axes.
	    (type: [double, double, double])(unit: radians)
    
###/hexapod/parameters/pose_controller/manual_pose_compensation/max_translation_velocity: 
    Sets the maximum translational posing VELOCITY of the body along the x,y,z axes.
	    (type: double)(default: 0.05)(unit: metres/s) 
	    
###/hexapod/parameters/pose_controller/manual_pose_compensation/max_rotation_velocity: 
    Sets the maximum rotational posing VELOCITY of the body around the x,y,z axes.
	    (type: double)(default: 0.01)(unit: rad/s) 

###/hexapod/parameters/pose_controller/manual_leg_manipulation/leg_manipulation_mode:
	Sets the leg manipulation mode between controlling the tip position ('tip_control') vs each individual joint position ('joint_control').
		(type: string)(default: joint_control)		
    
###/hexapod/parameters/pose_controller/packed_joint_positions/**_packed_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "packed" state.
	    (type: [double, double, double])(unit: radians)
    
###/hexapod/parameters/pose_controller/unpacked_joint_positions/**_unpacked_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "unpacked" state.
	    (type: [double, double, double])(unit: radians)
    
##Impedance Control Parameters:

###/hexapod/parameters/impedance_controller/impedance_control:
    Determines if impedance control is currently turned on/off.
	     (type: bool)(default: true)
	     
###/hexapod/parameters/impedance_controller/dynamic_stiffness:
    Determines if dynamic stiffness mode is on where set virtual stiffness is scaled determined on whether an individual leg is swinging or adjacent to a swinging leg.
	     (type: bool)(default: true)
    
###/hexapod/parameters/impedance_controller/impedance_input: (default: joint_effort)
    Determines input method for impedance. Either 'joint_effort' for using effort values from joint motors OR 'tip_force' for using pressure readings from tip sensors (MAX - Large Hexapod).
		 (type: string)(default: joint_effort)
    
###/hexapod/parameters/impedance_controller/virtual_mass:
    Virtual mass variable used in impedance controller spring-mass-damper virtualisation.
	    (type: double)
    
###/hexapod/parameters/impedance_controller/integrator_step_time:
    Time step used in impedance controller function.
	    (type: double)
    
###/hexapod/parameters/impedance_controller/virtual_stiffness:
    Virtual stiffness variable used in impedance controller spring-mass-damper virtualisation.
	    (type: double)

###/hexapod/parameters/impedance_controller/load_stiffness_scaler:
    Value used to scale the set virtual stiffness up when an individual leg is adjacent to a swinging leg and therefore under increased load.
	    (type: double)
	    
###/hexapod/parameters/impedance_controller/swing_stiffness_scaler:
    Value used to scale the set virtual stiffness down when an individual leg is swinging.
	    (type: double)
    
###/hexapod/parameters/impedance_controller/virtual_damping_ratio:
    Virtual damping ratio variable used in impedance controller spring-mass-damper virtualisation.
	    (type: double)
    
###/hexapod/parameters/impedance_controller/force_gain:
    Gain used to scale impedance values used in impedance controller.
	    (type: double)

##Debug Parameters:

###/hexapod/debug_parameters/testing:
    Turns on/off ability to perform 'test' runs with constant velocity input over a set time period. Tests start/stop using X Button.
	    (type: bool)(default: false)
    
###/hexapod/debug_parameters/test_time_length:
    Sets time length for test. A zero value sets an infinite test time length which can only be manually ended.
	    (type: bool)(default: 0.0)
    
###/hexapod/debug_parameters/test_linear_velocity:
    Sets constant linear body velocity input used in test. Note default is a full forward velocity input.
		(type: [double, double])(default: [0.0, 1.0])(unit: percentage (i.e. 0.0->1.0)) 

###/hexapod/debug_parameters/test_angular_velocity:
    Sets constant angular body velocity input used in test.
		(type: double)(default: 0.0)(unit: percentage (i.e. 0.0->1.0)) 
    
###/hexapod/debug_parameters/console_verbosity: 
    Sets rosconsole verbosity levels. Options include: 'debug', 'info', 'warning', 'error', 'fatal'.
	    (type: string)(default: info)
    
###/hexapod/debug_parameters/debug_move_to_joint_position:
    Allows debug output from moveToJointPosition function in poseController.
	    (type: bool)(default: false)
    
###/hexapod/debug_parameters/debug_step_to_position:
    Allows debug output from stepToPosition function in poseController.
        (type: bool)(default: false)
    
###/hexapod/debug_parameters/debug_swing_trajectory: 
    Allows debug output from swing part of updatePosition function in walkController.
        (type: bool)(default: false)
    
###/hexapod/debug_parameters/debug_stance_trajectory:
    Allows debug output from stance part of updatePosition function in walkController.
        (type: bool)(default: false)
    
###/hexapod/debug_parameters/debug_manual_compensation_translation: 
    Allows debug output from translation part of manualCompensation function in poseController.
        (type: bool)(default: false)
    
    
###/hexapod/debug_parameters/debug_manual_compensation_rotation: 
    Allows debug output from rotation part of manualCompensation function in poseController.
        (type: bool)(default: false)
    
###/hexapod/debug_parameters/debug_rviz:
    Turns on output for use in simulation in rviz.
        (type: bool)(default: false)
    
##Gait Parameters:

###/hexapod/gait_parameters/*gait_name*/stance_phase:      
    Ratio of step phase in the 'stance' state (on ground)
	    (type: int)
    
###/hexapod/gait_parameters/*gait_name*/swing_phase:       
    Ratio of step phase in the 'swing' state (in air)
	    (type: int)
    
###/hexapod/gait_parameters/*gait_name*/phase_offset:      
    Base ratio of step phase each leg is offset from each other.
	    (type: int)
    
###/hexapod/gait_parameters/*gait_name*/offset_multiplier: 
    Multiplier for phase_offset which determines phase offset for each leg. 
	    (type: [int, int, int, int, int, int])
    Offset multiplier values are assigned in the following order: 
      front_left (leg = 0, side = 0), 
      front_right (leg = 0, side = 1),
      middle_left (leg = 1, side = 0),
      middle_right (leg = 1, side = 1),
      rear_left (leg = 2, side = 0),
      rear_right (leg = 2, side = 1)

	  Example:     tripod_gait has stance:swing:offset ratio of 1:1:1. 
		          Assuming a total phase length of 1000: 
			            500 iterations (50%) of the step are stance, 
			            500 iterations (50%) of the step are swing
                        
	            The base phase offset is 500 iterations (50%).
	            With a offset_multiplier of [0,1,1,0,0,1] legs:
	              front_left, middle_right and rear_left have an effective offset
	              of 0*500 = 0 iterations (0%)
	              front_right, middle_left and rear_right have an effective offset
	              of 1*500 = 500 iterations (50%)
            
	  Example:     wave_gait has stance:swing:offset ratio of 5:1:1. 
		          Assuming a total phase length of 600: 
		            500 iterations (83.33%) of the step are stance, 
		            100 iterations (16.67%) of the step are swing 
            
	            The base phase offset is 100 iterations (16.67%).
	            With a offset_multiplier of [2,5,3,0,4,1] legs:
	              front_left has an offset of 2*100 = 200 iterations (33.33%)
	              front_right has an offset of 5*100 = 500 iterations (83.33%)
	              middle_left has an offset of 3*100 = 300 iterations (50%)
	              middle_right has an offset of 0*100 = 0 iterations (0%)
	              rear_left has an offset of 4*100 = 400 iterations (66.67%)
	              rear_right has an offset of 1*100 = 100 iterations (16.67%)
              
            Note that for this example the swing state starts at iteration 250
            (swing_start = stance_phase/2 = 500/2 = 250) and therefore these offsets
            give a swing order of:
              front_left, 
              rear_right, 
              middle_right, 
              front_right, 
              rear_left, 
              front_left
 