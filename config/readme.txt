#################################################################################################### 
####################################################################################################

Hexapod Parameters:

  /hexapod/parameters/hexapod_type:             
    String ID of the hexapod type associated with this set of config parameters. 
    Eg: "large_hexapod"
  
  /hexapod/parameters/time_delta: (default: 0.02)            
    Value used in setting ros loop frequency which denotes time period between cycles. (seconds)
  
  /hexapod/parameters/imu_compensation: (default: false)
    Bool sets whether IMU compensation mode is on/off. IMU compensation mode currently is not 
    working so do not use.
  
  /hexapod/parameters/auto_compensation: (default: false)
    Bool sets whether Auto compenation mode is on/off. Auto compensation mode currently is not 
    working so do not use.
    
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

  /hexapod/parameters/physical_leg_offsets:             
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
    
  /hexapod/parameters/walk_controller/body_clearance: (default: -1.0)
    Defines the max body clearance height above ground level as a percentage of max body-height. 
    A value of -1.0 sets the body clearance to be optimally calculated to maximise tip footprint 
    radius.
    
  /hexapod/parameters/walk_controller/leg_span_scale: (default: 1.0)
    Modifies the effective horizontal range of each leg in order to force a more narrow (<1.0) or 
    wide (>1.0) stance. The controller will crash if the forced leg span causes the body clearance 
    to no longer be achievable.
  
  /hexapod/parameters/walk_controller/leg_state_correction: (default: false)
    Bool turns on/off future mode (not fully implemented yet) which allows for the correction of 
    tip position based on early footfall detection or late liftoff detection.
  
  /hexapod/parameters/walk_controller/max_acceleration: (default: 0.1)
    Maximum linear acceleration of the body.
    
  /hexapod/parameters/walk_controller/max_curvature_speed: (default: 0.4)
   Maximum angular acceleration of the body.
  
  /hexapod/parameters/walk_controller/step_curvature_allowance: (default: 1.0)
    Modifies the effective step clearance in calculation of body clearance and restricts step 
    curvature to within tolerances. Value is a percentage of default. (i.e. 1.0 gives zero change 
    to step clearance)
  
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

  /hexapod/parameters/pose_controller/auto_pose_compensation/pitch_amplitude: (default: 0.025)
    Sets amplitude of pitch used in auto_compensation mode
    
  /hexapod/parameters/pose_controller/auto_pose_compensation/roll_amplitude: (default: 0.025)
    Sets amplitude of roll used in auto_compensation mode

  /hexapod/parameters/pose_controller/manual_pose_compensation/max_pose_time: (default: 1.0)
    Sets the maximum (and default) time (seconds) in which to complete manual pose actions.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_roll: (default: 0.075)
    Sets the amplitude of the maximum roll variable used in manual roll posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_pitch: (default: -0.075)
    Sets the amplitude of the maximum pitch variable used in manual pitch posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_yaw: (default: 0.2)
    Sets the amplitude of the maximum yaw variable used in manual yaw posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_x: (default: 0.05)
    Sets the amplitude of the maximum x translation variable used in manual x translation posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_y: (default: 0.05)
    Sets the amplitude of the maximum y translation variable used in manual y translation posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/max_z: (default: 0.05)
    Sets the amplitude of the maximum z translation variable used in manual z translation posing.
    
  /hexapod/parameters/pose_controller/manual_pose_compensation/min_z: (default: 0.05)
    Sets the amplitude of the minimum z translation variable used in manual z translation posing.

  /hexapod/parameters/pose_controller/packed_joint_positions/**_packed_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "packed" state. (radians)
    
  /hexapod/parameters/pose_controller/unpacked_joint_positions/**_unpacked_joint_positions:
    Denotes the joint angles for each joint [yaw, hip, knee] in the "unpacked" state. (radians)
    
####################################################################################################  

Gait Parameters:

  /hexapod/gait_parameters/*gait_name*/stance_phase:      
    Ratio of step phase in the 'stance' state (on ground)
    
  /hexapod/gait_parameters/*gait_name*/swing_phase:       
    Ratio of step phase in the 'swing' state (in air)
    
  /hexapod/gait_parameters/*gait_name*/transition_phase:  
    Ratio of step phase for each 'transition' state (2 transitions: stance->swing, swing->stance)
    
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

  Eg:     tripod_gait has stance:swing:transition:offset ratio of 4:4:1:5. 
          Assuming a total phase length of 100: 
            40 iterations (40%) of the step are stance, 
            40 iterations (40%) of the step are swing and
            10 iterations (10%) for each of 2 transition states 
            ((stance->swing and swing->stance) for a total of 20 iterations) 
            
            The base phase offset is 50 iterations (50%).
            With a offset_multiplier of [0,1,1,0,0,1] legs:
              front_left, middle_right and rear_left have an effective offset
              of 0*50 = 0 iterations (0%)
              front_right, middle_left and rear_right have an effective offset
              of 1*50 = 50 iterations (50%)
            
  Eg:     wave_gait has stance:swing:transition:offset ratio of 78:8:2:15. 
          Assuming a total phase length of 900: 
            780 iterations (87%) of the step are stance, 
            80 iterations (9%) of the step are swing and 
            20 iterations (2%) for each of 2 transition states 
            ((stance->swing and swing->stance) for a total of 40 iterations) 
            
            The base phase offset is 150 iterations (17%).
            With a offset_multiplier of [2,1,3,0,4,5] legs:
              front_left has an offset of 2*150 = 300 iterations (33.33%)
              front_right has an offset of 1*150 = 150 iterations (16.67%)
              middle_left has an offset of 3*150 = 450 iterations (50%)
              middle_right has an offset of 0*150 = 0 iterations (0%)
              rear_left has an offset of 4*150 = 600 iterations (66.67%)
              rear_right has an offset of 5*150 = 750 iterations (83.33%)
              
            Note that for this example the swing state starts at iteration 410*
            and therefore these offsets give a swing order of:
              front_left, front_right, middle_right, rear_right, middle_left
              
          *swing_start = (stance_phase(780)/2) + transition_phase(20) = 410
          
####################################################################################################
#################################################################################################### 