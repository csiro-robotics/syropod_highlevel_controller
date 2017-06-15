# Syropod Parameter File 
## (config/\*SYROPOD_NAME\*.yaml) 

## Control Parameters:
    
### syropod/parameters/time_delta:
    Value used in setting ros loop frequency which denotes time period between cycles.
      (type: double)
      (default: 0.02)
      (unit: seconds)
		
### /syropod/parameters/manual_posing:
    Sets whether manual posing system is on/off. Manual posing allows for the manual posing of the body independent of
    the walking cycle and additive to any other posing.
      (type: bool)
      (default: true)
    
### /syropod/parameters/auto_posing: 
    Sets whether auto posing system is on/off. Auto posing adjusts pose of body according to custom posing cycle 
    defined in config/auto_pose.yaml parameters. (see Auto Pose Parameter File below)
      (type: bool)
      (default: false)
    
### /syropod/parameters/impedance_control:
    Determines if impedance control is currently turned on/off.
      (type: bool)
      (default: false)
    
### /syropod/parameters/inclination_posing:
    Determines if inclination posing compensation system is on/off. This system adds x/y linear posing to the robot 
    body to align the assumed centre of mass of the body directly above the support polygon of the legs whilst on 
    inclined terrain, as detected by onboard IMU. (Requires IMU correctly set up.)
      (type: bool)
      (default: false)
	
### /syropod/parameters/imu_posing:
    Determines if imu posing compensation system is on/off. This system adds pitch/roll posing to the robot body to
    align it perpendicularly to gravity as detected via on-board IMU. (Requires PID tuning and IMU correctly set up.)
      (type: bool)
      (default: false)

## Hardware Parameters:
  
### /syropod/parameters/individual_control_interface:
    Determines if desired joint position commands are output on a 'per joint' basis. 
    (i.e. Each joint has it's own joint position command topic.)
      (type: bool)
      (default: true)
    
### /syropod/parameters/combined_control_interface:
    Determines if desired joint state commands are output combined on a single topic. (eg: "/desired_joint_states)
      (type: bool)
      (default: true)
    
### /syropod/parameters/imu_rotation_offset:
    The static euler angle rotation to apply to all IMU data in order to rotate it into the correct robot frame 
    coordinate system.
      (type: [double, double, double])
      (unit: radians)
	    
## Model Parameters
### /syropod/parameters/syropod_type:
    String ID of the Syropod type associated with this set of config parameters.
      (type: string)
      (example: "max")
    
### /syropod/parameters/leg_id:
    Array of strings which are used to name and identify each leg in the robot model. Legs should be identified 
    clockwise from the front-right most leg.
      (type: [string, string, string, ... string])
      (example: [AR, BR, CR, CL, BL, AL])
    
### /syropod/parameters/joint_id:
    Array of strings which are used to name and identify each 'potential' joint in a leg in the robot model. Joints 
    should be identified from the robot body to the leg tip.
      (type: [string, string, string, ... string])
      (example: [coxa, femur, tibia])
    
### /syropod/parameters/link_id:
    Array of strings which are used to name and identify each 'potential' link in a leg in the robot model. Links 
    should be identified from a MANDATORY link named 'base' at the robot body and down to the leg tip.
      (type: [string, string, string, ... string])
      (example: [base, coxa, femur, tibia])
    
### /syropod/parameters/leg_DOF:
    Map of leg_id (see /syropod/parameters/leg_id) and corresponding degrees of freedom (number of joints) in that leg.
      (type: {string: double, string: double, string: double, ... string: double})
      (example: {AR: 3, BR: 3, CR: 3, CL: 5, BL: 3, AL: 3})
    
### /syropod/parameters/leg_stance_yaws:
    Map of leg_id and the corresponding stance plane yaw about the z-axis in the robot frame.
      (type: {string: double, string: double, string: double, ... string: double})
      (example: {AR: -0.785, BR: -1.57, CR: -2.355, CL: 2.355, BL: 1.57, AL: 0.785})
      (unit: radians)

## Joint Parameters

### /syropod/parameters/\*LEG_ID\*_\*JOINT_ID\*_joint_parameters:
    Map of parameters for each joint corresponding to various joint characteristics:
      offset: The difference between the zero point for the joint in the robot model and the zero point for the motor 
        actuating that joint in hardware.
      min: The minimum limit of the joint position in the robot model.
      max: The maximum limit of the joint position in the robot model.
      packed: The joint position in the robot model which defines the joint as being in a 'packed' state.
      unpacked: The joint position in the robot model which defines the joint as being in a 'unpacked' state.
      max_vel: The maximum allowable joint velocity in the robot model.
      (type: {string: double, string: double, string: double, string: double, string: double, string: double})
      (example: AR_coxa_joint_parameters:  {offset: 0.0, min: -0.785, max: 0.785, packed: -1.57, unpacked 0.0, max_vel: 5.0})
      (unit: radians)
    
### /syropod/parameters/\*LEG_ID\*_\*JOINT_ID\*_link_parameters:
    Map of parameters for each link corresponding to classical Denavit–Hartenberg parameters for describing the 
    transformation of joints connected by the link:
      d: The DH parameter representing offset along previous z-axis to the common normal. (unit: metres)
      theta: The DH parameter representing angle about previous z axis, from old x-axis to new x-axis. (unit: radians)
      r: The DH parameter representing length of the common normal. (unit: metres)
      alpha: The DH parameter representing angle about common normal, form old z-axis to new z-axis. (unit: radians)
      (type: {string: double, string: double, string: double, string: double})
      (example: AR_femur_link_parameters: {d: 0.0, theta: 0.0, r: 0.07000, alpha: 0.0})

## Walk Controller Parameters:

### /syropod/parameters/gait_type:
    String ID of the default gait to be used by the Syropod.
      (type: string)
      (default: tripod_gait)

### /syropod/parameters/step_frequency:
    Number of full steps (full swing/stance transition) taken per second (in TRIPOD mode). For other gaits the 
    effective step frequency is adjusted according to swing ratio defined in the gait parameters.
    Note: This is an dynamically adjustable parameter and thus consists of a map of values which describe the 
    possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 1.0, min: 0.001, max: 2.0, step: 0.1})
      (unit: steps/s (Hz))
    
### /syropod/parameters/step_clearance: 
    Defines the desired clearance of the leg tip above the default position during swing period of step cycle.
    Note: This is an dynamically adjustable parameter and thus consists of a map of values which describe the 
    possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.1, min: 0.01, max: 0.05, step: 0.005})
      (unit: metres)
    
### /syropod/parameters/step_depth: 
    Defines the desired depth of the leg tip below the default position during stance period of step cycle. height. 
    Note: This is an dynamically adjustable parameter and thus consists of a map of values which describe the 
    possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.0, min: 0.0, max: 0.05, step: 0.005})
      (unit: metres)
    
### /syropod/parameters/body_clearance:
    Defines the desired clearance of the body above the default tip positions, limited to maximum possible height 
    determined by morphology.
    Note: This is an dynamically adjustable parameter and thus consists of a map of values which describe the 
    possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.3, min: 0.1, max: 0.5, step: 0.05})
      (unit: metres)
    
### /syropod/parameters/leg_span:
    Placeholder - currently does nothing.
    Note: This is an dynamically adjustable parameter and thus consists of a map of values which describe the 
    possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.0, min: 0.0, max: 0.0, step: 0.0})
      (unit: metres)
      
### /syropod/parameters/velocity_input_mode:
    String which defines the type of velocity input required:
      throttle: The controller expects all velocity inputs to be between 0.0 and 1.0 which describe a real world 
        velocity input of zero to the maximum possible body velocities.
      real: The controller expects all velocity inputs to be in real world SI units, limited to maximum possible 
        body velocities.
      (type: string)
      (default: throttle)
      
### /syropod/parameters/force_cruise_velocity:
    Bool which denotes whether defined cruise velocity inputs (see below) are used in cruise control mode. 
    Alternatively, the velocity inputs at the time of cruise control activation will be kept constant.
      (type: bool)
      (default: true)
    
### /syropod/parameters/linear_cruise_velocity:
    A map of linear velocities and the values to be used as constant linear body velocity input when 
    'force_cruise_velocity' is true and cruise control is active.
      (type: {string: double, string: double})
      (default: {x: 0.001, y: 0.0)
      (units: metres/s)
    
### /syropod/parameters/angular_cruise_velocity:
    An angular velocity value to be used as constant angular body velocity input when 'force_cruise_velocity' is true
    and cruise control is active.
      (type: double)
      (default: 0.0)
      (units: radians/s)
		
## Pose Controller Parameters:

### /syropod/parameters/auto_pose_type:
    String which defines the auto-posing cycle to be used (if auto posing feature is activated).
    These auto-posing cycle names are defined in config/auto_pose.yaml. Setting this parameters to 'auto'
    will append the current gait selection name and attempt to choose an associate auto-pose cycle 
    (i.e. tripod_gait_pose).
      (type: string)
      (default: auto)

### /syropod/parameters/start_up_sequence:
    Determines if robot attempts unpacking/packing & startup/shutdown procedures. If false the robot will move 
    legs DIRECTLY from initial positions to required positions for walking.
      (type: bool)
      (default: false)
    
### /syropod/parameters/pose_controller/time_to_start:
    Determines the length of time in which to complete a DIRECT startup sequence (i.e. start_up_sequence == false).
	    (type: bool)
	    (default: 12.0)
	    (unit: seconds)
    
### /syropod/parameters/rotation_pid_gains:
    A map of PID controller gains for the IMU posing system. (Requires PID tuning to develop gain values)
      (type: {string: double, string: double, string: double})
      (default: {p: 0.0, i: 0.0, d: 0.0})

### /syropod/parameters/max_translation: 
    Map defining the maximum linear translational posing of the body along the x,y,z axes.
	    (type: {string: double, string: double, string: double})
	    (example: {x: 0.1, y: 0.2, z: 0.3})
	    (unit: metres)
	    
### /syropod/parameters/max_rotation: 
    Map defining the maximum angular rotational posing of the body around the x,y,z axes (roll/pitch/yaw).
	    (type: {string: double, string: double, string: double})
	    (example: {roll: 0.2, pitch: 0.4, yaw: 0.6})
	    (unit: radians)
    
### /syropod/parameters/max_translation_velocity: 
    Sets the maximum translational posing VELOCITY of the body along the x,y,z axes.
	    (type: double)
	    (default: 0.05)
	    (unit: metres/s) 
	    
### /syropod/parameters/max_rotation_velocity: 
    Sets the maximum rotational posing VELOCITY of the body around the x,y,z axes (roll/pitch/yaw).
	    (type: double)
	    (default: 0.01)
	    (unit: rad/s) 

### /syropod/parameters/leg_manipulation_mode:
	Sets the leg manipulation mode between controlling the tip position ('tip_control') vs each individual joint 
	position ('joint_control').
		(type: string)
		(default: tip_control)
    
## Impedance Control Parameters:
	     
### /syropod/parameters/dynamic_stiffness:
    Determines if dynamic stiffness mode is on where set virtual stiffness is scaled determined on whether an 
    individual leg is swinging or adjacent to a swinging leg.
	     (type: bool)
	     (default: true)
    
### /syropod/parameters/use_joint_effort:
    Determines if force input method for impedance control is estimated using joint efforts. 
    Set to false if robot has tip force sensing capabilities.
		 (type: bool)
		 (default: false)
    
### /syropod/parameters/virtual_mass:
    Virtual mass variable used in impedance controller spring-mass-damper virtualisation.
    Note: This is a dynamically adjustable parameter and thus consists of a map of values which describe the possible 
    values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 10, min: 1, max: 100, step: 5})
      (unit: unitless)
    
### /syropod/parameters/integrator_step_time:
    Time step used in impedance controller ODE solver.
	    (type: double)
	    (default: 0.5)
    
### /syropod/parameters/virtual_stiffness:
    Virtual stiffness variable used in impedance controller spring-mass-damper virtualisation.
    Note: This is a dynamically adjustable parameter and thus consists of a map of values which describe the possible 
    values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 10, min: 1, max: 50, step: 5})
      (unit: unitless)

### /syropod/parameters/load_stiffness_scaler:
    Value used to scale the default virtual stiffness up when an individual leg is adjacent to a swinging leg and 
    therefore under increased load.
	    (type: double)
	    (default: 5.0)
	    
### /syropod/parameters/swing_stiffness_scaler:
    Value used to scale the default virtual stiffness down when an individual leg is swinging.
	    (type: double)
	    (default: 0.1)
    
### /syropod/parameters/virtual_damping_ratio:
    Virtual damping ratio variable used in impedance controller spring-mass-damper virtualisation.
	  Note: This is a dynamically adjustable parameter and thus consists of a map of values which describe the 
	  possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.8, min: 0.1, max: 10.0, step: 0.05})
      (unit: unitless)
    
### /syropod/parameters/force_gain:
    Gain used to scale input tip force values used in impedance controller.
	  Note: This is a dynamically adjustable parameter and thus consists of a map of values which describe the 
	  possible values of this parameter:
      default: The default parameter value.
      min: The minimum allowed parameter value.
      max: The maximum allowed parameter value.
      step: The increment/decrement step of this value when adjusted.
      (type: {string: double, string: double, string: double, string: double})
      (default: {default: 0.1, min: 0.001, max: 100.0, step: 1.0})
      (unit: unitless)

## Debug Parameters:
    
### /syropod/parameters/console_verbosity: 
    Sets rosconsole verbosity levels. Options include: 'debug', 'info', 'warning', 'error', 'fatal'.
	    (type: string)
	    (default: info)
    
### /syropod/parameters/debug_move_to_joint_position:
    Allows debug output from moveToJointPosition function in pose controller.
	    (type: bool)
	    (default: false)
    
### /syropod/parameters/debug_step_to_position:
    Allows debug output from stepToPosition function in pose controller.
        (type: bool)
        (default: false)
    
### /syropod/parameters/debug_swing_trajectory: 
    Allows debug output from swing trajectory generation in walk controller.
        (type: bool)
        (default: false)
    
### /syropod/parameters/debug_stance_trajectory:
    Allows debug output from stance trajectory generation in walk controller.
        (type: bool)
        (default: false)
    
### /syropod/parameters/execute_sequence: 
    Allows debug output from executeSequence function in pose controller. Also used to optimise joint 'unpacked' 
    parameter.
        (type: bool)
        (default: false)
        
### /syropod/parameters/debug_IK:
    Allows debug output from inverse kinematics engine.
        (type: bool)
        (default: false)
    
### /syropod/parameters/debug_rviz:
    Turns on output for use in simulation in RVIZ.
        (type: bool)
        (default: false)
        
### /syropod/parameters/debug_rviz_static_display:
    Determines if RVIZ visualisation has a static body position.
        (type: bool)
        (default: false)
    
------------------------------------------------------------------------------------------------------------------------
    
# Gait Parameters File 
## (config/gait.yaml)

### /syropod/gait_parameters/\*GAIT_NAME\*/stance_phase:
    Ratio of step phase in the 'stance' state (on ground)
	    (type: int)
    
### /syropod/gait_parameters/\*GAIT_NAME\*/swing_phase:       
    Ratio of step phase in the 'swing' state (in air)
	    (type: int)
    
### /syropod/gait_parameters/\*GAIT_NAME\*/phase_offset:      
    Base ratio of step phase each leg is offset from each other.
	    (type: int)
    
### /syropod/gait_parameters/\*GAIT_NAME\*/offset_multiplier: 
    Multiplier for phase_offset which determines phase offset for each leg. 
      (type: [int, int, int, ... int])
    Offset multiplier values are assigned clockwise from the front-right most leg (as per leg_id parameter)
      (eg: AR, BR, CR, CL, BL, AL)

    Example:
      tripod_gait has stance:swing:offset ratio of 2:2:2. 
      Assuming a total phase length of 1000: 
        500 iterations (50%) of the step are stance, 
        500 iterations (50%) of the step are swing
                        
        The base phase offset is 500 iterations (50%).
        With a offset_multiplier of [0,1,0,1,0,1], legs:
          AR, CR and BL legs have an effective offset of 0*500 = 0 iterations (0%)
          BL, CL and AL legs have an effective offset of 1*500 = 500 iterations (50%)
            
    Example: 
      wave_gait has stance:swing:offset ratio of 10:2:2. 
      Assuming a total phase length of 600: 
        500 iterations (83.33%) of the step are stance, 
        100 iterations (16.67%) of the step are swing 
            
        The base phase offset is 100 iterations (16.67%).
        With a offset_multiplier of [2,3,4,1,0,5], legs:
          AR has an offset of 2*100 = 200 iterations (33.33%)
          BR has an offset of 3*100 = 300 iterations (50%)
          CR has an offset of 4*100 = 400 iterations (66.67%)
          CL has an offset of 1*100 = 100 iterations (16.67%)
          BL has an offset of 0*100 = 0 iterations (0%)
          AL has an offset of 5*100 = 500 iterations (83.33%)
              
          Note that for this example the swing state starts at iteration 250
          (swing_start = stance_phase/2 = 500/2 = 250) and therefore these offsets
          give a swing order of:
            AR, CL, BL, AL, CR, BR, AR ...
            
------------------------------------------------------------------------------------------------------------------------
            
# Auto Pose Parameters 
## (config/auto_pose.yaml)

### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_frequency:
    The frequency at which this auto pose cycle repeats. Set to -1.0 to sync with step cycle.
      (type: double)
      (default: -1.0)
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_phase_length:
    The phase length of the base auto pose cycle.
      (type: int)
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_phase_starts:
    An array of phases which signify the start for each of any number of auto_pose cycles. Auto pose cycles may overlap
    and if so will superpose.
      (type: [int, int, int ... int])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_phase_ends:
    An array of phases which signify the end for each of any number of auto_pose cycles. Auto pose cycles may overlap
    and if so will superpose.
      (type: [int, int, int ... int])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_negation_phase_starts:
    A map of phases which signify the start for each to negate any auto posing applied to it by auto-pose cycles.
      (type: {string: int, string: int, string: int, ... string: int})

### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pose_negation_phase_ends:
    A map of phases which signify the end for each leg to negate any auto posing applied to it by auto-pose cycles.
      (type: {string: int, string: int, string: int, ... string: int})
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/x_amplitudes:
    An array of values which define the amplitude of translational linear body posing in the x-axis for each of any 
    number of auto pose cycles.
    (type: [double, double, double ... double])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/y_amplitudes:
    An array of values which define the amplitude of translational linear body posing in the y-axis for each of any 
    number of auto pose cycles.
      (type: [double, double, double ... double])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/z_amplitudes:
    An array of values which define the amplitude of translational linear body posing in the z-axis for each of any 
    number of auto pose cycles.
      (type: [double, double, double ... double])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/roll_amplitudes:
    An array of values which define the amplitude of rotational angular body posing about the x-axis (roll) for each 
    of any number of auto pose cycles.
      (type: [double, double, double ... double])
    
### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/pitch_amplitudes:
    An array of values which define the amplitude of rotational angular body posing about the y-axis (pitch) for each 
    of any number of auto pose cycles.
      (type: [double, double, double ... double])

### /syropod/auto_pose_parameters/\*AUTO_POSE_NAME\*/yaw_amplitudes:
    An array of values which define the amplitude of rotational angular body posing about the z-axis (yaw) for each 
    of any number of auto pose cycles.
      (type: [double, double, double ... double])
