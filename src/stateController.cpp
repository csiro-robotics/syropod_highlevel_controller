#include "../include/simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * State controller contructor
***********************************************************************************************************************/
StateController::StateController(ros::NodeHandle nodeHandle): n(nodeHandle)
{  
  //Get parameters from parameter server
  getParameters();
  defaultParams = params;
    
  //Initiate model and imu objects
  hexapod = new Model(params); 
  
  //Populate joint position array with excessive value
  for (int i=0; i<18; i++)
  {
    jointPositions[i] = 1e10;
    jointVelocities[i] = 0.0;
    jointEfforts[i] = 0.0;
    if (i < 6)
    {
      tipForces[i] = 0;
    }
  }
}

/***********************************************************************************************************************
 * State controller destructor
***********************************************************************************************************************/
StateController::~StateController()
{
  delete hexapod;
  delete walker;
  delete poser;
  delete impedance;
}

/***********************************************************************************************************************
 * Init state controller
***********************************************************************************************************************/
void StateController::init() 
{    
  //Setup motor interface
  if (params.dynamixelInterface)
  {
    interface = new DynamixelMotorInterface();  
  }
  else
  {
    interface = new DynamixelProMotorInterface();
  }
  
  interface->setupSpeed(params.interfaceSetupSpeed);
  
  //Set initial gait selection number for gait toggling
  if (params.gaitType == "tripod_gait")
  {
    gait = TRIPOD_GAIT;
  }
  else if (params.gaitType == "ripple_gait")
  {
    gait = RIPPLE_GAIT;
  }
  else if (params.gaitType == "wave_gait")
  {
    gait = WAVE_GAIT;
  }
  else if (params.gaitType == "amble_gait")
  {
    gait = AMBLE_GAIT;
  }
  
  // Create controller objects
  walker = new WalkController(hexapod, params);
  poser = new PoseController(hexapod, walker, params);
  impedance = new ImpedanceController(params);
  
  //Get unpacked/packed joint positions from params
  unpackedJointPositions[0][0] = params.unpackedJointPositionsAL;
  unpackedJointPositions[0][1] = params.unpackedJointPositionsAR;
  unpackedJointPositions[1][0] = params.unpackedJointPositionsBL;
  unpackedJointPositions[1][1] = params.unpackedJointPositionsBR;
  unpackedJointPositions[2][0] = params.unpackedJointPositionsCL;
  unpackedJointPositions[2][1] = params.unpackedJointPositionsCR; 
  packedJointPositions[0][0] = params.packedJointPositionsAL;
  packedJointPositions[0][1] = params.packedJointPositionsAR;
  packedJointPositions[1][0] = params.packedJointPositionsBL;
  packedJointPositions[1][1] = params.packedJointPositionsBR;
  packedJointPositions[2][0] = params.packedJointPositionsCL;
  packedJointPositions[2][1] = params.packedJointPositionsCR; 
  
  linearVelocityInput = Vector2d(0.0,0.0);
}

/***********************************************************************************************************************
 * Sets joint values in model according to aquired joint positions (set to default values if required)
***********************************************************************************************************************/
void StateController::setJointPositions(bool useDefaults)
{
  if (useDefaults)
  {
    for (int leg = 0; leg<3; leg++)
    {
      for (int side = 0; side<2; side++)
      {
        int index = leg*6+(side == 0 ? 0 : 3);
        double dir = side==0 ? -1 : 1;
        if (jointPositions[index] == 1e10)
        {
          jointPositions[index] = 0.0;
          ROS_INFO("Leg: %d:%d body-coxa joint set to: %f", leg, side, jointPositions[index]);
        }
        if (jointPositions[index+1] == 1e10)
        {
          jointPositions[index+1] = dir*max(0.0,hexapod->minMaxHipLift[0]);
	  ROS_INFO("Leg: %d:%d coxa-femour joint set to: %f", leg, side, jointPositions[index+1]);
        }
        if (jointPositions[index+2] == 1e10)
        {
          
          jointPositions[index+2] = dir*max(0.0,hexapod->minMaxKneeBend[0]);
	  ROS_INFO("Leg: %d:%d femour-tibia joint set to: %f", leg, side, jointPositions[index+2]);
        }
      }
      params.startUpSequence = false;
    }
  }
  
  //Set initial leg angles
  for (int leg = 0; leg<3; leg++)
  {
    for (int side = 0; side<2; side++)
    {
      double dir = side==0 ? -1 : 1;
      int index = leg*6+(side == 0 ? 0 : 3);
      hexapod->setLegStartAngles(side, leg, dir*Vector3d(jointPositions[index+0]+dir*params.physicalYawOffset[leg],
                                                        -jointPositions[index+1], 
                                                        jointPositions[index+2]+dir*params.physicalKneeOffset));
    }
  }  
}

/***********************************************************************************************************************
 * State machine loop
***********************************************************************************************************************/
void StateController::loop()
{
  //Compensation
  compensation();
    
  //Impedance control
  if (params.impedanceControl)
  { 
    impedanceControl();
  }
  
  //Hexapod state machine
  switch (state)
  {
    //Hexapod is in a state of unpacking itself into neutral 'unpacked' position
    case(UNPACK):
    {
      unpackState();
      break;
    }
      
    //Hexapod steps safely from unpacked position into the operational 'walking' position
    case(STARTUP):
    {                
      startUpState();
      break;
    }       
    
    //Hexapod is in operational state (walking/posing/actuating etc.)
    case(RUNNING):
    {   
      runningState(); 
      break; 
    }         
    
    //Hexapod steps from operational position into a safe position where it can then begin packing procedure
    case(SHUTDOWN):
    {    
      shutDownState();
      break;
    }

    //Hexapod transitions to packed state
    case(PACK):
    {
      packState();
      break;
    }   
    
    //Hexapod in packed state
    case(PACKED):
    {       
      packedState();
      break;
    }        
    
    //State unknown due to first iteration of controller
    case(UNKNOWN):
    {
      unknownState();    
      break;
    }
    
    //No startup/shutdown ability requested - move legs directly to walk position  
    case(DIRECT):
    {
      directState();
      break;
    }
  } 
}

/***********************************************************************************************************************
 * Unpack state
***********************************************************************************************************************/
void StateController::unpackState()
{
  //Move joints directly to unpacked positions and then transition to next state
  if (poser->moveToJointPosition(unpackedJointPositions, 2.0/params.stepFrequency))
  {          
    state = STARTUP;
    ROS_INFO("Hexapod unpacked. Running startup sequence . . .\n");
  }  
}

/***********************************************************************************************************************
 * StartUp state
***********************************************************************************************************************/
void StateController::startUpState()
{
  //Run through start up sequence (stepping to defined positions) and then transition to next state
  if (poser->startUpSequence(walker->identityTipPositions, params.moveLegsSequentially))
  {    
    state = RUNNING;
    ROS_INFO("Startup sequence complete. Ready to walk.\n");
  }
}

/***********************************************************************************************************************
 * Shutdown state
***********************************************************************************************************************/
void StateController::shutDownState()
{
  //Run through shut down sequence (stepping to defined positions) and then transition to next state
  if (poser->shutDownSequence(walker->identityTipPositions, params.moveLegsSequentially))
  {
    state = PACK;
    ROS_INFO("Shutdown sequence complete. Packing hexapod . . .\n");
  } 
}

/***********************************************************************************************************************
 * Pack state
***********************************************************************************************************************/
void StateController::packState()
{
  //Move joints directly to packed positions and then transition to next state
  if (poser->moveToJointPosition(packedJointPositions, 2.0/params.stepFrequency))
  {
    state = PACKED;
    ROS_INFO("Hexapod packing complete.\n");  
  }
}

/***********************************************************************************************************************
 * Packed state
***********************************************************************************************************************/
void StateController::packedState()
{
  //Checks for start flag and transitions to next state
  if (startFlag)
  {
    state = UNPACK;
    ROS_INFO("Unpacking hexapod . . .\n");
  }
}

/***********************************************************************************************************************
 * Direct state
***********************************************************************************************************************/
void StateController::directState()
{
  //Checks for start flag and then directly moves tips to walking position and transitions to next state
  if (startFlag)
  {
    int mode = params.moveLegsSequentially ? SEQUENTIAL_MODE:NO_STEP_MODE;
    
    if (poser->stepToPosition(hexapod->stanceTipPositions, deltaZ, mode, 0, params.timeToStart))
    {
      state = RUNNING;
      ROS_INFO("Startup sequence complete. Ready to walk.\n");
    }
  }
}

/***********************************************************************************************************************
 * Unknown state
***********************************************************************************************************************/
void StateController::unknownState()
{
  int checkPacked = 0;
  for (int l=0; l<3; l++)
  {
    for (int s=0; s<2; s++)
    {
      //Check all current joint positions are 'close' to packed joint positions
      double tolerance = 0.1;
      checkPacked += abs(hexapod->legs[l][s].yaw - packedJointPositions[l][s][0]) < tolerance &&
                      abs(hexapod->legs[l][s].liftAngle - packedJointPositions[l][s][1]) < tolerance &&
                      abs(hexapod->legs[l][s].kneeAngle - packedJointPositions[l][s][2]) < tolerance;
    }
  }
  if (checkPacked == 6) //All joints in each leg are approximately in the packed position
  {
    if (!params.startUpSequence)
    {
      ROS_ERROR("Hexapod currently in packed state and cannot run direct startup sequence.\nEither manually unpack hexapod or set start_up_sequence to true in config file\n");
      ROS_ASSERT(false);
    }
    else
    {
      state = PACKED;
      ROS_INFO("Hexapod currently packed.\n");  
    }
  }
  else if (!params.startUpSequence)
  {    
    state = DIRECT;
    ROS_WARN("WARNING! Running direct startup sequence - assuming hexapod is not on the ground\n");
    ROS_INFO("Running startup sequence (Complete in %f seconds) . . .\n", params.timeToStart);       
  }
  else if (startFlag)
  {
    state = STARTUP;
    ROS_INFO("Hexapod unpacked. Running startup sequence . . .\n");
  } 
}

/***********************************************************************************************************************
 * Running state
***********************************************************************************************************************/
void StateController::runningState()
{     
  //Switch gait
  if (changeGait)
  {
    gaitChange();
  }        
            
  //Leg Selection for toggling state          
  if (toggleLegState)
  {
    legStateToggle();
  }
  
  //Dynamic Parameter Adjustment
  if (adjustParam)
  {
    paramAdjust();
  }
  else
  {     
    if (params.testing)
    {    
      if (testState == TEST_RUNNING && (runningTime < params.testTimeLength || params.testTimeLength == 0.0))
      {
	ROS_DEBUG_COND(runningTime == 0 && params.testTimeLength > 0.0, "Test started. Will finish automatically in %f seconds. Press X to stop test early.\n", params.testTimeLength);
	ROS_DEBUG_COND(runningTime == 0 && params.testTimeLength == 0.0, "Test started. No time limit. Press X to stop test.\n");
	runningTime += params.timeDelta;
	linearVelocityInput = params.testLinearVelocity;
	angularVelocityInput = params.testAngularVelocity;
      }
      else
      {
	ROS_DEBUG_COND(testState == TEST_RUNNING, "Test ended as scheduled.\n");
	ROS_DEBUG_COND(runningTime > 0.0, "Test ended manually.\n");
	runningTime = 0.0;
	testState = TEST_ENDED;
      }
    }
     
    //Update Walker 
    walker->updateWalk(linearVelocityInput, angularVelocityInput, deltaZ); 
  }
  
  //Check for shutdown cue
  if (!startFlag && params.startUpSequence)
  {
    state = SHUTDOWN;
    ROS_INFO("Running shutdown sequence . . .\n");
  }
}

/***********************************************************************************************************************
 * Compensation
***********************************************************************************************************************/
void StateController::compensation()
{  
  //Manually set (joystick controlled) body compensation 
  if (params.manualCompensation)
  {          
    Pose targetPose = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitchJoy,rollJoy,yawJoy));
    poser->manualCompensation(targetPose, poseResetMode); //Updates current pose  
    poser->currentPose = poser->manualPose;
  } 
  
  //Compensation to align centre of gravity evenly between tip positions on incline
  if (params.inclinationCompensation)
  {
    poser->inclinationCompensation(imuData);    
    poser->currentPose.position += poser->inclinationPose.position;    
  } 
  
  //Compensation to offset average deltaZ from impedance controller and keep body at specificied height
  if (params.impedanceControl)
  {
    poser->impedanceControllerCompensation(deltaZ);
    poser->currentPose.position += poser->deltaZPose.position;
  }
  
  //Auto body compensation using IMU feedback
  if (params.imuCompensation)
  {    
    poser->imuCompensation(imuData, poser->manualPose.rotation); 
    poser->currentPose.rotation = poser->imuPose.rotation;  
  } 
  //Automatic (non-feedback) body compensation
  else if (params.autoCompensation)    
  {   
    poser->autoCompensation();
    poser->currentPose.position += poser->autoPoseDefault.position;
    poser->currentPose.rotation[1] += poser->autoPoseDefault.rotation[1];
    poser->currentPose.rotation[2] += poser->autoPoseDefault.rotation[2];    
  }    
  
  //Update walking stance based on desired pose
  poser->updateStance(walker->identityTipPositions, params.autoCompensation && !params.imuCompensation);
}

/***********************************************************************************************************************
 * Impedance Control
***********************************************************************************************************************/
void StateController::impedanceControl()
{     
  //Check how many legs are in stance phase
  int legsInStance = 0; 
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      if (walker->legSteppers[l][s].state != SWING)
      {
        legsInStance++;
      }
    }
  }  
  
  //Calculate new stiffness based on imu orientation or walking cycle
  bool useIMUForStiffness = false; //Not fully tested
  if (params.dynamicStiffness)
  {
    if (params.imuCompensation && useIMUForStiffness)
    {
      impedance->updateStiffness(poser->currentPose, walker->identityTipPositions); //TBD CHECK 
    } 
    else
    {
      impedance->updateStiffness(walker);
    }
  }
  
  //Get current force value on leg and run impedance calculations to get a vertical tip offset (deltaZ)
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {  
      if (params.dynamicStiffness || legsInStance == 6)
      {
	double maxForce = 0;
	double minForce = 0;
	if (useTipForce)
	{
	  double forceOffset = 1255.0;   
	  tipForce[l][s] = tipForces[2*l+s] - forceOffset;
	  maxForce = 1000.0;
	  minForce = 0.0;
	}
	else if (useJointEffort)
	{                
	  int index = 6*l+3*s+1;
	  int dir = (s==0) ? -1:1;
	  tipForce[l][s] = dir*(jointEfforts[index]);
	  maxForce = 1e9;
	  minForce = 0.0;
	}  
	//Ensure force is within limits
	tipForce[l][s] = min(tipForce[l][s], maxForce);
	tipForce[l][s] = max(tipForce[l][s], minForce);
      }
      
      if (hexapod->legs[l][s].state == WALKING)
      {
	impedance->updateImpedance(l, s, tipForce, deltaZ);
      }
      else
      {
	impedance->virtualStiffness[l][s] = 0; //No effect - just for debugging
      }
    }
  }
}

/***********************************************************************************************************************
 * Dynamic Parameter adjustment
***********************************************************************************************************************/
void StateController::paramAdjust()
{ 
  //Force hexapod to stop walking
  linearVelocityInput = Vector2d(0.0,0.0);
  angularVelocityInput = 0.0;
  
  if (walker->state == STOPPED)
  {
    std::string paramString;
    double paramVal;
    switch(paramSelection)
    {
      case(NO_PARAM_SELECTION):
      {
        break;
      }
      case(STEP_FREQUENCY):
      {
        if (paramScaler == -1)
        {
          paramScaler = params.stepFrequency/defaultParams.stepFrequency;
        }
        params.stepFrequency = minMax(defaultParams.stepFrequency*paramScaler, 0.1, 3.0);
        walker->setGaitParams(params);
        poser->params = params;
        paramString = "step_frequency";
        paramVal = params.stepFrequency;
        break;
      }
      case(STEP_CLEARANCE):
      {
        if (paramScaler == -1)
        {
          paramScaler = params.stepClearance/defaultParams.stepClearance;
        }
        params.stepClearance = minMax(defaultParams.stepClearance*paramScaler, 0.01, 0.4); 
        walker->init(hexapod, params);
        paramString = "step_clearance";
        paramVal = params.stepClearance;
        break;
      }
      case(BODY_CLEARANCE):
      {                  
        if (defaultParams.bodyClearance == -1)
        {
          params.bodyClearance = walker->bodyClearance;
          defaultParams.bodyClearance = params.bodyClearance;
        }                   
        if (paramScaler == -1)
        {
          paramScaler = params.bodyClearance/defaultParams.bodyClearance;
        }                  
        params.bodyClearance = minMax(defaultParams.bodyClearance*paramScaler, 0.1, 0.99);                     
        walker->init(hexapod, params);
        paramString = "body_clearance";
        paramVal = params.bodyClearance;                  
        break;
      }                
      case(LEG_SPAN_SCALE):
      {                                  
        if (paramScaler == -1)
        {
          paramScaler = params.legSpanScale/defaultParams.legSpanScale;
        }                  
        params.legSpanScale = minMax(defaultParams.legSpanScale*paramScaler, 0.1, 1.5);                     
        walker->init(hexapod, params);
        paramString = "leg_span_scale";
        paramVal = params.legSpanScale;                  
        break;
      }
      case(VIRTUAL_MASS):
      {
	if (paramScaler == -1)
        {
          paramScaler = params.virtualMass/defaultParams.virtualMass;
        }                  
        params.virtualMass = minMax(defaultParams.virtualMass*paramScaler, 0, 500);                     
        impedance->init(params);
        paramString = "virtual_mass";
        paramVal = params.virtualMass;                  
        break;
      }
      case(VIRTUAL_STIFFNESS):
      {
	if (paramScaler == -1)
	{
          paramScaler = params.virtualStiffness/defaultParams.virtualStiffness;
        }                  
        params.virtualStiffness = minMax(defaultParams.virtualStiffness*paramScaler, 0, 500);                     
        impedance->init(params);
        paramString = "virtual_stiffness";
        paramVal = params.virtualStiffness;                  
        break;
      }
      case(VIRTUAL_DAMPING):
      {
	if (paramScaler == -1)
	{
          paramScaler = params.virtualDampingRatio/defaultParams.virtualDampingRatio;
        }                  
        params.virtualDampingRatio = minMax(defaultParams.virtualDampingRatio*paramScaler, 0, 2.0);                     
        impedance->init(params);
        paramString = "virtual_damping_ratio";
        paramVal = params.virtualDampingRatio;                  
        break;
      }
      case(FORCE_GAIN):
      {
	if (paramScaler == -1)
	{
          paramScaler = params.forceGain/defaultParams.forceGain;
        }                  
        params.forceGain = minMax(defaultParams.forceGain*paramScaler, 0, 2.0);                     
        impedance->init(params);
        paramString = "force_gain";
        paramVal = params.forceGain;                  
        break;
      }
      default:
      {
        ROS_WARN("Attempting to adjust unknown parameter.\n");
        break;
      }
    }              
    //Update tip Positions for new parameter value
    double stepHeight = walker->maximumBodyHeight*walker->stepClearance;
    if (poser->stepToPosition(hexapod->stanceTipPositions, deltaZ, TRIPOD_MODE, stepHeight, 1.0/walker->stepFrequency))
    {    
      ROS_INFO("Parameter '%s' set to %d%% of default (%f).\n", paramString.c_str(), roundToInt(paramScaler*100), paramVal);
      adjustParam = false;
    } 
  }
}

/***********************************************************************************************************************
 * Gait change
***********************************************************************************************************************/
void StateController::gaitChange()
{ 
  //Force hexapod to stop walking
  linearVelocityInput = Vector2d(0.0,0.0);
  angularVelocityInput = 0.0;
  
  if (walker->state == STOPPED)
  {
    switch (gait)
    {
      case (TRIPOD_GAIT):
        getGaitParameters("tripod_gait");
        break;
      case(RIPPLE_GAIT):
        getGaitParameters("ripple_gait"); 
        break;
      case(WAVE_GAIT):
        getGaitParameters("wave_gait");
        break;
      case(AMBLE_GAIT):
	getGaitParameters("amble_gait");
        break;
      default:
        ROS_WARN("Attempting to change to unknown gait.\n");
        break;
    }   
    walker->setGaitParams(params);
    poser->params = params;
    impedance->params = params;
    ROS_INFO("Now using %s mode.\n", params.gaitType.c_str());
    changeGait = false;
  }
}

/***********************************************************************************************************************
 * Leg State Toggle
***********************************************************************************************************************/
void StateController::legStateToggle()
{ 
  //Force hexapod to stop walking
  linearVelocityInput = Vector2d(0.0,0.0);
  angularVelocityInput = 0.0;
  
  if (walker->state == STOPPED)
  {
    int l = legSelection/2;
    int s = legSelection%2;
    if (hexapod->legs[l][s].state == WALKING)
    {        
      hexapod->legs[l][s].state = WALKING_TO_OFF;
    }
    else if (hexapod->legs[l][s].state == OFF)
    {
      hexapod->legs[l][s].state = OFF_TO_WALKING;
    }
    else if (hexapod->legs[l][s].state == WALKING_TO_OFF)
    {
      Vector3d targetTipPositions[3][2] = hexapod->localTipPositions;
      double stepHeight = walker->stepClearance*walker->maximumBodyHeight;
      targetTipPositions[l][s][2] = hexapod->stanceTipPositions[l][s][2] + stepHeight;
      if (poser->stepToPosition(targetTipPositions, deltaZ, NO_STEP_MODE, 0.0, 1.0/params.stepFrequency))
      {
	toggleLegState = false;
	hexapod->legs[l][s].state = OFF;
	ROS_INFO("Leg: %d:%d set to state: OFF.\n", legSelection/2, legSelection%2);
      }
    }
    else if (hexapod->legs[l][s].state == OFF_TO_WALKING)
    {
      Vector3d targetTipPositions[3][2] = hexapod->localTipPositions;
      targetTipPositions[l][s][2] = hexapod->stanceTipPositions[l][s][2];
      if (poser->stepToPosition(targetTipPositions, deltaZ, NO_STEP_MODE, 0.0, 1.0/params.stepFrequency))
      {
	toggleLegState = false;
	hexapod->legs[l][s].state = WALKING;
	ROS_INFO("Leg: %d:%d set to state: WALKING.\n", legSelection/2, legSelection%2);
      }    
    }
  }
}

/***********************************************************************************************************************
 * Publishes local tip positions for debugging
***********************************************************************************************************************/
void StateController::publishLocalTipPositions()
{
  std_msgs::Float32MultiArray msg;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {            
      msg.data.clear();
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[0]);
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[1]);
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[2]);
      localTipPositionPublishers[l][s].publish(msg);
    }
  }
}

/***********************************************************************************************************************
 * Publishes tip positions (before any adjustments from pose or impedance control) for debugging
***********************************************************************************************************************/
void StateController::publishWalkerTipPositions()
{
  std_msgs::Float32MultiArray msg;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {            
      msg.data.clear();
      msg.data.push_back(walker->legSteppers[l][s].currentTipPosition[0]);
      msg.data.push_back(walker->legSteppers[l][s].currentTipPosition[1]);
      msg.data.push_back(walker->legSteppers[l][s].currentTipPosition[2]);
      walkerTipPositionPublishers[l][s].publish(msg);
    }
  }
}

/***********************************************************************************************************************
 * Publishes tip positions (before any adjustments from pose or impedance control) for debugging
***********************************************************************************************************************/
void StateController::publishStanceTipPositions()
{
  std_msgs::Float32MultiArray msg;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {            
      msg.data.clear();
      msg.data.push_back(hexapod->legs[l][s].stanceTipPosition[0]);
      msg.data.push_back(hexapod->legs[l][s].stanceTipPosition[1]);
      msg.data.push_back(hexapod->legs[l][s].stanceTipPosition[2]);
      stanceTipPositionPublishers[l][s].publish(msg);
    }
  }
}

/***********************************************************************************************************************
 * Publishes tip velocities (before any adjustments from pose or impedance control) for debugging
***********************************************************************************************************************/
void StateController::publishWalkerTipVelocities()
{
  std_msgs::Float32MultiArray msg;
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {            
      msg.data.clear();
      msg.data.push_back(walker->legSteppers[l][s].currentTipVelocity[0]);
      msg.data.push_back(walker->legSteppers[l][s].currentTipVelocity[1]);
      msg.data.push_back(walker->legSteppers[l][s].currentTipVelocity[2]);
      walkerTipVelocityPublishers[l][s].publish(msg);
    }
  }
}

/***********************************************************************************************************************
 * Publishes tip forces for debugging
***********************************************************************************************************************/
void StateController::publishTipForces()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  for (int l = 0; l<3; l++)
  { 
    for (int s = 0; s<2; s++)
    {         
      msg.data.push_back(tipForce[l][s]);      
    }
  }
  tipForcePublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes deltaZ for debugging
***********************************************************************************************************************/
void StateController::publishDeltaZ()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  for (int l = 0; l<3; l++)
  { 
    for (int s = 0; s<2; s++)
    {         
      msg.data.push_back(deltaZ[l][s]);      
    }
  }
  deltaZPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current pose (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishPose()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser->currentPose.rotation[1]);  
  msg.data.push_back(poser->currentPose.rotation[2]); 
  msg.data.push_back(poser->currentPose.rotation[3]); 
  msg.data.push_back(poser->currentPose.position[0]); 
  msg.data.push_back(poser->currentPose.position[1]); 
  msg.data.push_back(poser->currentPose.position[2]); 
  posePublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes current rotation as per the IMU (roll, pitch, yaw, x, y, z) for debugging
***********************************************************************************************************************/
void StateController::publishIMURotation()
{
  Quat orientation;
  
  //Get orientation data from IMU
  orientation.w = imuData.orientation.w;
  orientation.x = imuData.orientation.x;
  orientation.y = imuData.orientation.y;
  orientation.z = imuData.orientation.z;
  
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(orientation.toEulerAngles()[0]);  
  msg.data.push_back(orientation.toEulerAngles()[1]);
  msg.data.push_back(orientation.toEulerAngles()[2]); 
  IMURotationPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes stiffness for debugging
***********************************************************************************************************************/
void StateController::publishStiffness()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(impedance->virtualStiffness[0][0]);  
  msg.data.push_back(impedance->virtualStiffness[0][1]); 
  msg.data.push_back(impedance->virtualStiffness[1][0]); 
  msg.data.push_back(impedance->virtualStiffness[1][1]); 
  msg.data.push_back(impedance->virtualStiffness[2][0]); 
  msg.data.push_back(impedance->virtualStiffness[2][1]); 
  stiffnessPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishRotationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser->rotationPositionError[0]);  
  msg.data.push_back(poser->rotationPositionError[1]); 
  msg.data.push_back(poser->rotationPositionError[2]); 
  msg.data.push_back(poser->rotationAbsementError[0]);
  msg.data.push_back(poser->rotationAbsementError[1]);
  msg.data.push_back(poser->rotationAbsementError[2]);  
  msg.data.push_back(poser->rotationVelocityError[0]);
  msg.data.push_back(poser->rotationVelocityError[1]);
  msg.data.push_back(poser->rotationVelocityError[2]);
  rotationPoseErrorPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishTranslationPoseError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(poser->translationPositionError[0]);  
  msg.data.push_back(poser->translationPositionError[1]); 
  msg.data.push_back(poser->translationPositionError[2]); 
  msg.data.push_back(poser->translationAbsementError[0]);
  msg.data.push_back(poser->translationAbsementError[1]);
  msg.data.push_back(poser->translationAbsementError[2]);  
  msg.data.push_back(poser->translationVelocityError[0]);
  msg.data.push_back(poser->translationVelocityError[1]);
  msg.data.push_back(poser->translationVelocityError[2]);
  translationPoseErrorPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes pose angle and position error for debugging
***********************************************************************************************************************/
void StateController::publishZTipError()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  for (int l = 0; l<3; l++)
  { 
    for (int s = 0; s<2; s++)
    {         
      msg.data.push_back(impedance->zTipPositionError[l][s]); 
      msg.data.push_back(impedance->zTipAbsementError[l][s]); 
      msg.data.push_back(impedance->zTipVelocityError[l][s]);      
    }
  }  
  zTipErrorPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Publishes body velocity for debugging
***********************************************************************************************************************/
void StateController::publishBodyVelocity()
{
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.data.push_back(walker->currentLinearVelocity[0]);
  msg.data.push_back(walker->currentLinearVelocity[1]);
  msg.data.push_back(walker->currentAngularVelocity);
  bodyVelocityPublisher.publish(msg);
}

/***********************************************************************************************************************
 * Draws robot in RVIZ for debugging
***********************************************************************************************************************/
void StateController::RVIZDebugging()
{    
  for (int s = 0; s<2; s++)
  {
    for (int l = 0; l<3; l++)
    {
      debug.tipPositions.insert(debug.tipPositions.begin(), walker->pose.transformVector(hexapod->legs[l][s].localTipPosition));
      debug.staticTipPositions.insert(debug.staticTipPositions.begin(), hexapod->legs[l][s].localTipPosition);
    }
  }
  
  debug.drawRobot(hexapod->legs[0][0].rootOffset, hexapod->getJointPositions(walker->pose), Vector4d(1,1,1,1));    
  debug.drawPoints(debug.tipPositions, Vector4d(1,0,0,1)); //Actual Tip Trajectory Paths
  //debug.drawPoints(debug.staticTipPositions, Vector4d(1,0,0,1)); //Static Single Tip Trajectory command
  
  if (debug.tipPositions.size() > 2000)
  {
    debug.tipPositions.erase(debug.tipPositions.end()-6,debug.tipPositions.end());
  }
  if (debug.staticTipPositions.size() >= 6*(1/(params.timeDelta*walker->stepFrequency)))
  {
    debug.staticTipPositions.clear();
  }
}

/***********************************************************************************************************************
 * Clamps joint velocities and publishes angle and velocity data to motor interface
***********************************************************************************************************************/
void StateController::publishJointValues()
{    
  for (int s = 0; s<2; s++)
  {
    double dir = s==0 ? -1 : 1;
    for (int l = 0; l<3; l++)
    {            
      double yaw = dir*(hexapod->legs[l][s].yaw - params.physicalYawOffset[l]);
      double lift = dir*hexapod->legs[l][s].liftAngle;
      double knee = dir*hexapod->legs[l][s].kneeAngle - dir*params.physicalKneeOffset;
      
      double yawVel = 0;
      double liftVel = 0;
      double kneeVel = 0;
      
      if (firstIteration >= 6)  //First Iteration of ALL legs
      {       
        yawVel = (yaw - hexapod->legs[l][s].oldYaw)/params.timeDelta;
        liftVel = (lift - hexapod->legs[l][s].oldLiftAngle)/params.timeDelta;
        kneeVel = (knee - hexapod->legs[l][s].oldKneeAngle)/params.timeDelta;
        
        if (abs(yawVel) > hexapod->jointMaxAngularSpeeds[0])
        {
          ROS_WARN("Leg: %d:%d body_coxa joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n", 
		   l, s, yawVel, sign(yawVel)*hexapod->jointMaxAngularSpeeds[0]);
	  
          yawVel = sign(yawVel)*hexapod->jointMaxAngularSpeeds[0];
          yaw = hexapod->legs[l][s].oldYaw + yawVel*params.timeDelta;
        }
        if (abs(liftVel) > hexapod->jointMaxAngularSpeeds[1])
        {
	  ROS_WARN("Leg: %d:%d coxa_femour joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n", 
		   l, s, liftVel, sign(liftVel)*hexapod->jointMaxAngularSpeeds[1]);  
	  
          liftVel = sign(liftVel)*hexapod->jointMaxAngularSpeeds[1];
          lift = hexapod->legs[l][s].oldLiftAngle + liftVel*params.timeDelta;
        }
        if (abs(kneeVel) > hexapod->jointMaxAngularSpeeds[2])
        {
	  ROS_WARN("Leg: %d:%d femour_tibia joint velocity (%f) exceeds maximum (%f)) - CLAMPING TO MAXIMUM!\n", 
		   l, s, kneeVel, sign(kneeVel)*hexapod->jointMaxAngularSpeeds[2]);
	  
          kneeVel = sign(kneeVel)*hexapod->jointMaxAngularSpeeds[2];
          knee = hexapod->legs[l][s].oldKneeAngle + kneeVel*params.timeDelta;
        }
      }
      else
      {
        firstIteration++; //First Iteration of ALL legs
      }
      
      interface->setTargetAngle(l, s, 0, yaw);
      interface->setTargetAngle(l, s, 1, -lift);
      interface->setTargetAngle(l, s, 2, knee);
              
      if (params.dynamixelInterface)
      {
	interface->setVelocity(l, s, 0, yawVel); //Doesn't cooperate with with Large Hexapod Dynamixel Pro drivers
      }
      interface->setVelocity(l, s, 1, -liftVel);
      interface->setVelocity(l, s, 2, kneeVel);
        
      hexapod->legs[l][s].oldYaw = yaw;
      hexapod->legs[l][s].oldLiftAngle = lift;
      hexapod->legs[l][s].oldKneeAngle = knee;
    }
  }    
  interface->publish();
}

/***********************************************************************************************************************
 * This callback increments the gains in the controller
***********************************************************************************************************************/
void StateController::paramSelectionCallback(const std_msgs::Int8 &input)
{
  if (state == RUNNING && input.data != paramSelection)
  {
    paramScaler = -1;
    switch (input.data)
    {
      case(NO_PARAM_SELECTION):
      {
        paramSelection = NO_PARAM_SELECTION;
        ROS_INFO("No parameters currently selected.\n");
        break;
      }
      case(STEP_FREQUENCY):
      {
        paramSelection = STEP_FREQUENCY;
        ROS_INFO("step_frequency parameter selected.\n");
        break;
      }
      case(STEP_CLEARANCE):
      {
        paramSelection = STEP_CLEARANCE;
        ROS_INFO("step_clearance parameter selected.\n");
        break;
      }
      case(BODY_CLEARANCE):
      {
        paramSelection = BODY_CLEARANCE;
        ROS_INFO("body_clearance parameter selected.\n");
        break;
      }      
      case(LEG_SPAN_SCALE):
      {
        paramSelection = LEG_SPAN_SCALE;
        ROS_INFO("leg_span_scale parameter selected.\n");
        break;
      }
      case(VIRTUAL_MASS):
      {
        paramSelection = VIRTUAL_MASS;
        ROS_INFO("virtual_mass impedance parameter selected.\n");
        break;
      }
      case(VIRTUAL_STIFFNESS):
      {
        paramSelection = VIRTUAL_STIFFNESS;
        ROS_INFO("virtual_stiffness impedance parameter selected.\n");
        break;
      }      
      case(VIRTUAL_DAMPING):
      {
        paramSelection = VIRTUAL_DAMPING;
        ROS_INFO("virtual_damping impedance parameter selected.\n");
        break;
      }
      case(FORCE_GAIN):
      {
        paramSelection = FORCE_GAIN;
        ROS_INFO("force_gain impedance parameter selected.\n");
        break;
      }
      default:
      {
        paramSelection = NO_PARAM_SELECTION;
        ROS_INFO("Unknown parameter selection requested from control input. No parameters currently selected.\n");
        break;
      }
    }
  }
}

/***********************************************************************************************************************
 * Toggle Leg State Callback
***********************************************************************************************************************/
void StateController::paramAdjustCallback(const std_msgs::Int8 &input)
{
  if (input.data != 0.0 && !adjustParam && paramSelection != NO_PARAM_SELECTION)
  {
    if (paramScaler != -1)
    {
      paramScaler += input.data/paramAdjustSensitivity;
      paramScaler = minMax(paramScaler, 0.1, 3.0);      //Parameter scaler ranges from 10%->300%
      
      ROS_INFO("Adjusting selected parameter . . . (WARNING: Changing parameters may crash controller)\n");
    }
    else
    {
      ROS_INFO("Checking selected parameter value . . .\n");
    }
    adjustParam = true;
  }
}

/***********************************************************************************************************************
 * Gait Selection Callback
***********************************************************************************************************************/
void StateController::gaitSelectionCallback(const std_msgs::Int8 &input)
{
  if (input.data != gait && state == RUNNING)
  {
    switch (input.data)
    {
      case(-1):
        break;
      case(TRIPOD_GAIT):
        ROS_INFO("Transitioning to tripod_gait mode . . .\n");
        gait = TRIPOD_GAIT;
        break;
      case(RIPPLE_GAIT):
        ROS_INFO("Transitioning to ripple_gait mode . . .\n");
        gait = RIPPLE_GAIT;
        break;
      case(WAVE_GAIT):
        ROS_INFO("Transitioning to wave_gait mode . . .\n");
        gait = WAVE_GAIT;
        break;
      case (AMBLE_GAIT):
	ROS_INFO("Transitioning to amble_gait mode . . .\n");
        gait = AMBLE_GAIT;
        break;
      default:
        ROS_WARN("Unknown gait requested from control input.\n");
    }
    if (gait == input.data)
    {
      changeGait = true;
    }
  }
}

/***********************************************************************************************************************
 * Actuating Leg Selection Callback
***********************************************************************************************************************/
void StateController::legSelectionCallback(const std_msgs::Int8 &input)
{
  if (state == RUNNING && !toggleLegState)
  {
    switch (int(input.data))
    {
      case(-1):
        break;
      case(FRONT_LEFT):
	legSelection = FRONT_LEFT;
	ROS_INFO("Front left leg selected.\n");
        break;
      case(FRONT_RIGHT):
	legSelection = FRONT_RIGHT;
	ROS_INFO("Front right leg selected.\n");
        break;
      case(MIDDLE_LEFT):
	legSelection = MIDDLE_LEFT;
	ROS_INFO("Middle left leg selected.\n");
        break;
      case(MIDDLE_RIGHT):
	legSelection = MIDDLE_RIGHT;
	ROS_INFO("Middle right leg selected.\n");
        break;
      case(REAR_LEFT):
	legSelection = REAR_LEFT;
	ROS_INFO("Rear left leg selected.\n");
        break;
      case(REAR_RIGHT):
	legSelection = REAR_RIGHT;
	ROS_INFO("Rear right leg selected.\n");
        break;
      default:
        ROS_WARN("Unknown leg selection requested from control input.\n");
    }
  }
}

/***********************************************************************************************************************
 * Joypad Velocity Topic Callback
***********************************************************************************************************************/
void StateController::joypadVelocityCallback(const geometry_msgs::Twist &twist)
{ 
  linearVelocityInput = Vector2d(twist.linear.x, twist.linear.y);
  angularVelocityInput = twist.angular.x;
}

/***********************************************************************************************************************
 * Joypad Pose Topic Callback
***********************************************************************************************************************/
void StateController::joypadPoseCallback(const geometry_msgs::Twist &twist)
{  
  rollJoy = twist.angular.x;;
  pitchJoy = -twist.angular.y; //Invert joystick
  yawJoy = twist.angular.z;
  xJoy = twist.linear.x;
  yJoy = twist.linear.y;
  zJoy = twist.linear.z;
}

/***********************************************************************************************************************
 * Joypad Start State Topic Callback
***********************************************************************************************************************/
void StateController::startCallback(const std_msgs::Bool &input)
{
  startFlag = input.data;
}

/***********************************************************************************************************************
 * Joypad Pose reset mode Topic Callback
***********************************************************************************************************************/
void StateController::poseResetCallback(const std_msgs::Int8 &input)
{
  poseResetMode = static_cast<PoseResetMode>(input.data);
}

/***********************************************************************************************************************
 * Toggle Leg State Callback
***********************************************************************************************************************/
void StateController::legStateCallback(const std_msgs::Bool &input)
{
  if (input.data && legStateDebounce)
  {
    toggleLegState = true;
    legStateDebounce = false;
  }
  else if (!input.data && !toggleLegState)
  {
    legStateDebounce = true;
  }
}

/***********************************************************************************************************************
 * Test Start Callback
***********************************************************************************************************************/
void StateController::startTestCallback(const std_msgs::Bool &input)
{
  if (input.data && startTestDebounce)
  {
    if (testState == TEST_RUNNING)
    {
      testState = TEST_ENDED;
    }
    else if (testState == TEST_ENDED)
    {
      testState = TEST_RUNNING;
    }
    startTestDebounce = false;
  }
  else if (!input.data)// && testState == TEST_ENDED)
  {
    startTestDebounce = true;
  }
}

/***********************************************************************************************************************
 * IMU data callback
***********************************************************************************************************************/
void StateController::imuCallback(const sensor_msgs::Imu &data)
{
  imuData = data;
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void StateController::jointStatesCallback(const sensor_msgs::JointState &jointStates)
{  
  bool getEffortValues = (jointStates.effort.size() != 0);
  bool getVelocityValues = (jointStates.velocity.size() != 0);
  
  for (uint i=0; i<jointStates.name.size(); i++)
  {
    const char* jointName = jointStates.name[i].c_str();
    if (!strcmp(jointName, "front_left_body_coxa") ||
        !strcmp(jointName, "AL_coxa_joint"))
    {
      jointPositions[0] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[0] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[0] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_coxa_femour") ||
              !strcmp(jointName, "AL_femur_joint"))
    {
      jointPositions[1] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[1] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[1] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_femour_tibia") ||
              !strcmp(jointName, "AL_tibia_joint"))
    {
      jointPositions[2] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[2] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[2] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_body_coxa") ||
              !strcmp(jointName, "AR_coxa_joint"))
    {
      jointPositions[3] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[3] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[3] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_coxa_femour") ||
              !strcmp(jointName, "AR_femur_joint"))
    {
      jointPositions[4] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[4] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[4] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_femour_tibia") ||
              !strcmp(jointName, "AR_tibia_joint"))
    {
      jointPositions[5] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[5] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[5] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_body_coxa") ||
              !strcmp(jointName, "BL_coxa_joint"))
    {
      jointPositions[6] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[6] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[6] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_coxa_femour") ||
              !strcmp(jointName, "BL_femur_joint"))
    {
      jointPositions[7] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[7] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[7] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_femour_tibia") ||
              !strcmp(jointName, "BL_tibia_joint"))
    {
      jointPositions[8] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[8] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[8] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_body_coxa") ||
              !strcmp(jointName, "BR_coxa_joint"))
    {
      jointPositions[9] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[9] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[9] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_coxa_femour") ||
              !strcmp(jointName, "BR_femur_joint"))
    {
      jointPositions[10] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[10] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[10] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_femour_tibia") ||
              !strcmp(jointName, "BR_tibia_joint"))
    {
      jointPositions[11] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[11] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[11] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_body_coxa") ||
              !strcmp(jointName, "CL_coxa_joint"))
    {
      jointPositions[12] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[12] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[12] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_coxa_femour") ||
              !strcmp(jointName, "CL_femur_joint"))
    {
      jointPositions[13] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[13] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[13] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_femour_tibia") ||
              !strcmp(jointName, "CL_tibia_joint"))
    {
      jointPositions[14] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[14] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[14] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_body_coxa") ||
              !strcmp(jointName, "CR_coxa_joint"))
    {
      jointPositions[15] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[15] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[15] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_coxa_femour") ||
              !strcmp(jointName, "CR_femur_joint"))
    {
      jointPositions[16] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[16] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[16] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_femour_tibia") ||
              !strcmp(jointName, "CR_tibia_joint"))
    {
      jointPositions[17] = jointStates.position[i];
      if (getVelocityValues) 
      {
	jointVelocities[17] = jointStates.velocity[i];
      }
      if (getEffortValues) 
      {
        jointEfforts[17] = jointStates.effort[i];
      }
    }
    
    //Check if all joint positions have been received from topic
    jointPosFlag = true;
    for (int i=0; i<18; i++)
    {  
      if (jointPositions[i] > 1e9)
      {
        jointPosFlag = false;
      }
    }
  }
}

/***********************************************************************************************************************
 * Gets tip forces
***********************************************************************************************************************/
void StateController::tipForceCallback(const sensor_msgs::JointState &jointStates)
{  
  tipForces[0] = jointStates.effort[0];
  tipForces[1] = jointStates.effort[2];
  tipForces[2] = jointStates.effort[4];
  tipForces[3] = jointStates.effort[6];
  tipForces[4] = jointStates.effort[8];
  tipForces[5] = jointStates.effort[10];  
}

/***********************************************************************************************************************
 * Gets hexapod parameters from rosparam server
***********************************************************************************************************************/
void StateController::getParameters()
{
  std::string baseParamString="/hexapod/parameters/";
  std::string paramString;
  std::string forceGait; 
  
  //Gait Parameters
  getGaitParameters(forceGait); //giving empty forceGait string gives default gait parameter
  
  // Hexapod Parameters
  if(!n.getParam(baseParamString+"hexapod_type", params.hexapodType))
  {
    ROS_ERROR("Error reading parameter/s (hexapod_type) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if(!n.getParam(baseParamString+"time_delta", params.timeDelta))
  {
    ROS_ERROR("Error reading parameter/s (time_delta) from rosparam. Check config file is loaded and type is correct\n");
  }
    
  if(!n.getParam(baseParamString+"imu_compensation", params.imuCompensation))
  {
    ROS_ERROR("Error reading parameter/s (imu_compensation) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(baseParamString+"auto_compensation", params.autoCompensation))
  {
    ROS_ERROR("Error reading parameter/s (auto_compensation) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(baseParamString+"manual_compensation", params.manualCompensation))
  {
    ROS_ERROR("Error reading parameter/s (manual_compensation) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(baseParamString+"inclination_compensation", params.inclinationCompensation))
  {
    ROS_ERROR("Error reading parameter/s (inclination_compensation) from rosparam. Check config file is loaded and type is correct\n");  
  }
   
  /********************************************************************************************************************/
  //Offset Parameters
  //Root Offset Parameters
  std::vector<double> rootOffsetAL(3);
  paramString=baseParamString+"/physical_leg_offsets/";
  if(!n.getParam(paramString+"root_offset_AL", rootOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_AL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[0][0] = Map<Vector3d>(&rootOffsetAL[0], 3);
  }
  
  std::vector<double> rootOffsetAR(3);
  if(!n.getParam(paramString+"root_offset_AR", rootOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_AR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[0][1] = Map<Vector3d>(&rootOffsetAR[0], 3);
  }
  
  std::vector<double> rootOffsetBL(3);
  if(!n.getParam(paramString+"root_offset_BL", rootOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_BL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[1][0] = Map<Vector3d>(&rootOffsetBL[0], 3);
  }
  
  std::vector<double> rootOffsetBR(3);
  if(!n.getParam(paramString+"root_offset_BR", rootOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_BR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[1][1] = Map<Vector3d>(&rootOffsetBR[0], 3);
  }
  
    std::vector<double> rootOffsetCL(3);
  if(!n.getParam(paramString+"root_offset_CL", rootOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_CL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[2][0] = Map<Vector3d>(&rootOffsetCL[0], 3);
  }
  
  std::vector<double> rootOffsetCR(3);
  if(!n.getParam(paramString+"root_offset_CR", rootOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (root_offset_CR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.rootOffset[2][1] = Map<Vector3d>(&rootOffsetCR[0], 3);
  }
  
  /********************************************************************************************************************/
  //Hip Offset Parameters
  std::vector<double> hipOffsetAL(3);
  if(!n.getParam(paramString+"hip_offset_AL", hipOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_AL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[0][0] = Map<Vector3d>(&hipOffsetAL[0], 3);
  }
  
  std::vector<double> hipOffsetAR(3);
  if(!n.getParam(paramString+"hip_offset_AR", hipOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_AR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[0][1] = Map<Vector3d>(&hipOffsetAR[0], 3);
  }
  
  std::vector<double> hipOffsetBL(3);
  if(!n.getParam(paramString+"hip_offset_BL", hipOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_BL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[1][0] = Map<Vector3d>(&hipOffsetBL[0], 3);
  }
  
  std::vector<double> hipOffsetBR(3);
  if(!n.getParam(paramString+"hip_offset_BR", hipOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_BR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[1][1] = Map<Vector3d>(&hipOffsetBR[0], 3);
  }
  
    std::vector<double> hipOffsetCL(3);
  if(!n.getParam(paramString+"hip_offset_CL", hipOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_CL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[2][0] = Map<Vector3d>(&hipOffsetCL[0], 3);
  }
  
  std::vector<double> hipOffsetCR(3);
  if(!n.getParam(paramString+"hip_offset_CR", hipOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (hip_offset_CR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipOffset[2][1] = Map<Vector3d>(&hipOffsetCR[0], 3);
  }
  
  /********************************************************************************************************************/
  //Knee Offset Parameters
  std::vector<double> kneeOffsetAL(3);
  if(!n.getParam(paramString+"knee_offset_AL", kneeOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_AL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[0][0] = Map<Vector3d>(&kneeOffsetAL[0], 3);
  }
  
  std::vector<double> kneeOffsetAR(3);
  if(!n.getParam(paramString+"knee_offset_AR", kneeOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_AR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[0][1] = Map<Vector3d>(&kneeOffsetAR[0], 3);
  }
  
  std::vector<double> kneeOffsetBL(3);
  if(!n.getParam(paramString+"knee_offset_BL", kneeOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_BL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[1][0] = Map<Vector3d>(&kneeOffsetBL[0], 3);
  }
  
  std::vector<double> kneeOffsetBR(3);
  if(!n.getParam(paramString+"knee_offset_BR", kneeOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_BR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[1][1] = Map<Vector3d>(&kneeOffsetBR[0], 3);
  }
  
    std::vector<double> kneeOffsetCL(3);
  if(!n.getParam(paramString+"knee_offset_CL", kneeOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_CL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[2][0] = Map<Vector3d>(&kneeOffsetCL[0], 3);
  }
  
  std::vector<double> kneeOffsetCR(3);
  if(!n.getParam(paramString+"knee_offset_CR", kneeOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (knee_offset_CR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeOffset[2][1] = Map<Vector3d>(&kneeOffsetCR[0], 3);
  }
  
  /********************************************************************************************************************/
  //Tip Offset Parameters
  std::vector<double> tipOffsetAL(3);
  if(!n.getParam(paramString+"tip_offset_AL", tipOffsetAL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_AL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[0][0] = Map<Vector3d>(&tipOffsetAL[0], 3);
  }
  
  std::vector<double> tipOffsetAR(3);
  if(!n.getParam(paramString+"tip_offset_AR", tipOffsetAR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_AR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[0][1] = Map<Vector3d>(&tipOffsetAR[0], 3);
  }
  
  std::vector<double> tipOffsetBL(3);
  if(!n.getParam(paramString+"tip_offset_BL", tipOffsetBL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_BL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[1][0] = Map<Vector3d>(&tipOffsetBL[0], 3);
  }
  
  std::vector<double> tipOffsetBR(3);
  if(!n.getParam(paramString+"tip_offset_BR", tipOffsetBR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_BR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[1][1] = Map<Vector3d>(&tipOffsetBR[0], 3);
  }
  
    std::vector<double> tipOffsetCL(3);
  if(!n.getParam(paramString+"tip_offset_CL", tipOffsetCL))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_CL) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[2][0] = Map<Vector3d>(&tipOffsetCL[0], 3);
  }
  
  std::vector<double> tipOffsetCR(3);
  if(!n.getParam(paramString+"tip_offset_CR", tipOffsetCR))
  {
    ROS_ERROR("Error reading parameter/s (tip_offset_CR) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.tipOffset[2][1] = Map<Vector3d>(&tipOffsetCR[0], 3);
  }
  
  /********************************************************************************************************************/
  //Yaw parameters 
  std::vector<double> stanceLegYaws(3);
  if(!n.getParam(baseParamString+"stance_leg_yaws", stanceLegYaws))
  {
    ROS_ERROR("Error reading parameter/s (stance_leg_yaws) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.stanceLegYaws = Map<Vector3d>(&stanceLegYaws[0], 3);
  }
  
  std::vector<double> physicalYawOffset(3);
  if(!n.getParam(baseParamString+"physical_yaw_offset", physicalYawOffset))
  {
    ROS_ERROR("Error reading parameter/s (physical_yaw_offset) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.physicalYawOffset = Map<Vector3d>(&physicalYawOffset[0], 3);
  }
  
  //Knee Offset parameter
  if(!n.getParam(baseParamString+"physical_knee_offset", params.physicalKneeOffset))
  {
    ROS_ERROR("Error reading parameter/s (physical_knee_offset) from rosparam. Check config file is loaded and type is correct\n");
  }

  /********************************************************************************************************************/
  //Joint Limit Parameters  
  paramString = baseParamString+"/joint_limits/";
  std::vector<double> yawLimits(3);
  if(!n.getParam(paramString+"yaw_limits", yawLimits))
  {
    ROS_ERROR("Error reading parameter/s (yaw_limits) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.yawLimits = Map<Vector3d>(&yawLimits[0], 3);
  }
    
  std::vector<double> kneeLimits(2);
  if(!n.getParam(paramString+"knee_limits", kneeLimits))
  {
    ROS_ERROR("Error reading parameter/s (knee_limits) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.kneeLimits = Map<Vector2d>(&kneeLimits[0], 2);
  }
  
  std::vector<double> hipLimits(2);    
  if(!n.getParam(paramString+"hip_limits", hipLimits))
  {
    ROS_ERROR("Error reading parameter/s (hip_limits) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.hipLimits = Map<Vector2d>(&hipLimits[0], 2);
  }
  
  std::vector<double> jointMaxAngularSpeeds(2); 
  if(!n.getParam(baseParamString+"joint_max_angular_speeds", jointMaxAngularSpeeds))
  {
    ROS_ERROR("Error reading parameter/s (joint_max_angular_speed) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.jointMaxAngularSpeeds = Map<Vector3d>(&jointMaxAngularSpeeds[0], 3);
  }
  
  if(!n.getParam(baseParamString+"dynamixel_interface", params.dynamixelInterface))
  {
    ROS_ERROR("Error reading parameter/s (dynamixel_interface) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  /********************************************************************************************************************/
  // Walk Controller Parameters
  paramString = baseParamString+"/walk_controller/";
  
  if (!n.getParam(paramString+"step_frequency", params.stepFrequency))
  {
    ROS_ERROR("Error reading parameter/s (step_frequency) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"step_clearance", params.stepClearance))
  {
    ROS_ERROR("Error reading parameter/s (step_clearance) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"step_depth", params.stepDepth))
  {
    ROS_ERROR("Error reading parameter/s (step_depth) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"body_clearance", params.bodyClearance))
  {
    ROS_ERROR("Error reading parameter/s (body_clearance) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"leg_span_scale", params.legSpanScale))
  {
    ROS_ERROR("Error reading parameter/s (leg_span_scale) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"max_linear_acceleration", params.maxLinearAcceleration))
  {
    ROS_ERROR("Error reading parameter/s (max_linear_acceleration) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"max_angular_acceleration", params.maxAngularAcceleration))
  {
    ROS_ERROR("Error reading parameter/s (max_angular_acceleration) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"footprint_downscale", params.footprintDownscale))
  {
    ROS_ERROR("Error reading parameter/s (footprint_downscale) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"interface_setup_speed", params.interfaceSetupSpeed))
  {
    ROS_ERROR("Error reading parameter/s (interface_setup_speed) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"velocity_input_mode", params.velocityInputMode))
  {
    ROS_ERROR("Error reading parameter/s (velocity_input_mode) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  /********************************************************************************************************************/
  // Pose Controller Parameters 
  paramString = baseParamString+"/pose_controller/";
  
  if(!n.getParam(paramString+"start_up_sequence", params.startUpSequence))
  {
    ROS_ERROR("Error reading parameter/s (start_up_sequence) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"move_legs_sequentially", params.moveLegsSequentially))
  {
    ROS_ERROR("Error reading parameter/s (move_legs_sequentially) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"time_to_start", params.timeToStart))
  {
    ROS_ERROR("Error reading parameter/s (time_to_start) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  //IMU compensation parameters
  //Translation compensation
  paramString = baseParamString+"/pose_controller/imu_pose_compensation/translation_compensation/";
  if(!n.getParam(paramString+"proportional_gain", params.translationCompensationProportionalGain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/proportional_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"integral_gain", params.translationCompensationIntegralGain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/integral_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"derivative_gain", params.translationCompensationDerivativeGain))
  {
    ROS_ERROR("Error reading parameter/s (translation_compensation/derivative_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }  
  //Rotation Compensation
  paramString = baseParamString+"/pose_controller/imu_pose_compensation/rotation_compensation/";
  if(!n.getParam(paramString+"proportional_gain", params.rotationCompensationProportionalGain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/proportional_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"integral_gain", params.rotationCompensationIntegralGain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/integral_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"derivative_gain", params.rotationCompensationDerivativeGain))
  {
    ROS_ERROR("Error reading parameter/s (rotation_compensation/derivative_gain) from rosparam. Check config file is loaded and type is correct\n");  
  }  
  
  //Auto compensation parameters
  paramString = baseParamString+"/pose_controller/auto_pose_compensation/";
  if(!n.getParam(paramString+"pitch_amplitude", params.pitchAmplitude))
  {
    ROS_ERROR("Error reading parameter/s (pitch_amplitude) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"roll_amplitude", params.rollAmplitude))
  {
    ROS_ERROR("Error reading parameter/s (roll_amplitude) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"z_trans_amplitude", params.zTransAmplitude))
  {
    ROS_ERROR("Error reading parameter/s (z_trans_amplitude) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  //Manual compensation parameters
  paramString = baseParamString+"/pose_controller/manual_pose_compensation/";
  std::vector<double> maxTranslation(3);
  if (!n.getParam(paramString+"max_translation", maxTranslation))
  {
    ROS_ERROR("Error reading parameter/s (max_translation) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.maxTranslation = Map<Vector3d>(&maxTranslation[0], 3);
  }
  
  std::vector<double> maxRotation(3);
  if (!n.getParam(paramString+"max_rotation", maxRotation))
  {
    ROS_ERROR("Error reading parameter/s (max_rotation) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.maxRotation = Map<Vector3d>(&maxRotation[0], 3);
  } 
  
  if(!n.getParam(paramString+"max_translation_velocity", params.maxTranslationVelocity))
  {
    ROS_ERROR("Error reading parameter/s (max_translation_velocity) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  if(!n.getParam(paramString+"max_rotation_velocity", params.maxRotationVelocity))
  {
    ROS_ERROR("Error reading parameter/s (max_rotation_velocity) from rosparam. Check config file is loaded and type is correct\n");  
  }
  
  /********************************************************************************************************************/
  
  paramString = baseParamString + "/pose_controller/packed_joint_positions/";  
  std::vector<double> packedJointPositionsAL(3);
  if (!n.getParam(paramString+"AL_packed_joint_positions", packedJointPositionsAL))
  {
    ROS_ERROR("Error reading parameter/s (AL_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsAL = Map<Vector3d>(&packedJointPositionsAL[0], 3);
  }
    
  std::vector<double> packedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_packed_joint_positions", packedJointPositionsAR))
  {
    ROS_ERROR("Error reading parameter/s (AR_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsAR = Map<Vector3d>(&packedJointPositionsAR[0], 3);
  }
  
  std::vector<double> packedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_packed_joint_positions", packedJointPositionsBL))
  {
    ROS_ERROR("Error reading parameter/s (BL_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsBL = Map<Vector3d>(&packedJointPositionsBL[0], 3);
  }
  
  std::vector<double> packedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_packed_joint_positions", packedJointPositionsBR))
  {
    ROS_ERROR("Error reading parameter/s (BR_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsBR = Map<Vector3d>(&packedJointPositionsBR[0], 3);
  }
  
  std::vector<double> packedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_packed_joint_positions", packedJointPositionsCL))
  {
    ROS_ERROR("Error reading parameter/s (CL_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsCL = Map<Vector3d>(&packedJointPositionsCL[0], 3);
  }
  
  std::vector<double> packedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_packed_joint_positions", packedJointPositionsCR))
  {
    ROS_ERROR("Error reading parameter/s (CR_packed_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.packedJointPositionsCR = Map<Vector3d>(&packedJointPositionsCR[0], 3);
  }
  
  /********************************************************************************************************************/
  paramString = baseParamString + "/pose_controller/unpacked_joint_positions/";  
  std::vector<double> unpackedJointPositionsAL(3);
  if (!n.getParam(paramString+"AL_unpacked_joint_positions", unpackedJointPositionsAL))
  {
    ROS_ERROR("Error reading parameter/s (AL_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsAL = Map<Vector3d>(&unpackedJointPositionsAL[0], 3);
  }
    
  std::vector<double> unpackedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_unpacked_joint_positions", unpackedJointPositionsAR))
  {
    ROS_ERROR("Error reading parameter/s (AR_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsAR = Map<Vector3d>(&unpackedJointPositionsAR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_unpacked_joint_positions", unpackedJointPositionsBL))
  {
    ROS_ERROR("Error reading parameter/s (BL_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsBL = Map<Vector3d>(&unpackedJointPositionsBL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_unpacked_joint_positions", unpackedJointPositionsBR))
  {
    ROS_ERROR("Error reading parameter/s (BR_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsBR = Map<Vector3d>(&unpackedJointPositionsBR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_unpacked_joint_positions", unpackedJointPositionsCL))
  {
    ROS_ERROR("Error reading parameter/s (CL_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsCL = Map<Vector3d>(&unpackedJointPositionsCL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_unpacked_joint_positions", unpackedJointPositionsCR))
  {
    ROS_ERROR("Error reading parameter/s (CR_unpacked_joint_positions) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.unpackedJointPositionsCR = Map<Vector3d>(&unpackedJointPositionsCR[0], 3);
  }
  
  /********************************************************************************************************************/
  // Impedance Control Parameters 
  
  paramString = baseParamString+"/impedance_controller/";

  if (!n.getParam(paramString+"impedance_control", params.impedanceControl))
  {
    ROS_ERROR("Error reading parameter/s (impedance_control) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"dynamic_stiffness", params.dynamicStiffness))
  {
    ROS_ERROR("Error reading parameter/s (dynamic_stiffness) from rosparam. Check config file is loaded and type is correct\n");
  }

  if (!n.getParam(paramString+"integrator_step_time", params.integratorStepTime))
  {
    ROS_ERROR("Error reading parameter/s (integrator_step_time) from rosparam. Check config file is loaded and type is correct\n");
  }

  if (!n.getParam(paramString+"virtual_mass", params.virtualMass))
  {
    ROS_ERROR("Error reading parameter/s (virtual_mass) from rosparam. Check config file is loaded and type is correct\n");
  }

  if (!n.getParam(paramString+"virtual_stiffness", params.virtualStiffness))
  {
    ROS_ERROR("Error reading parameter/s (virtual_stiffness) from rosparam. Check config file is loaded and type is correct\n");
  }
    
  if (!n.getParam(paramString+"/stiffness_multiplier", params.stiffnessMultiplier))
  {
    ROS_ERROR("Error reading parameter/s (stiffness_multiplier) from rosparam. Check config file is loaded and type is correct\n");
  } 

  if (!n.getParam(paramString+"virtual_damping_ratio", params.virtualDampingRatio))
  {
    ROS_ERROR("Error reading parameter/s (virtual_damping_ratio) from rosparam. Check config file is loaded and type is correct\n");
  }

  if (!n.getParam(paramString+"force_gain", params.forceGain))
  {
    ROS_ERROR("Error reading parameter/s (force_gain) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString+"impedance_input", params.impedanceInput))
  {
    ROS_ERROR("Error reading parameter/s (impedance_input) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  /********************************************************************************************************************/
  // Debug Parameters 
  
  paramString="/hexapod/debug_parameters/";
  
  if (!n.getParam(paramString + "testing", params.testing))
  {
    ROS_ERROR("Error reading debug parameter/s (testing) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "test_time_length", params.testTimeLength))
  {
    ROS_ERROR("Error reading debug parameter/s (test_time_length) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  std::vector<double> testLinearVelocity(2);
  if (!n.getParam(paramString + "test_linear_velocity", testLinearVelocity))
  {
    ROS_ERROR("Error reading debug parameter/s (test_linear_velocity) from rosparam. Check config file is loaded and type is correct\n");
  }
  else
  {
    params.testLinearVelocity = Map<Vector2d>(&testLinearVelocity[0], 2);
  }
  
  if (!n.getParam(paramString + "test_angular_velocity", params.testAngularVelocity))
  {
    ROS_ERROR("Error reading debug parameter/s (test_angular_velocity) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "console_verbosity", params.consoleVerbosity))
  {
    ROS_ERROR("Error reading debug parameter/s (console_verbosity) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_move_to_joint_position", params.debugMoveToJointPosition))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_move_to_joint_position) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_step_to_position", params.debugStepToPosition))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_step_to_position) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_swing_trajectory", params.debugSwingTrajectory))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_swing_trajectory) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_stance_trajectory", params.debugStanceTrajectory))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_stance_trajectory) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_manual_compensation_rotation", params.debugManualCompensationRotation))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_manual_compensation_rotation) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_manual_compensation_translation", params.debugManualCompensationTranslation))
  {
    ROS_ERROR("Error reading debug parameter/s (debug_manual_compensation_translation) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  if (!n.getParam(paramString + "debug_rviz", params.debug_rviz))
  {
    ROS_ERROR("Error reading debug parameter/s (rviz) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  /********************************************************************************************************************/
  //Hexapod remote parameters (set via hexapod_remote launch file instead of by config files)
  paramString = "/hexapod_remote/param_adjust_sensitivity";
  if (!n.getParam(paramString, paramAdjustSensitivity))
  {
    ROS_ERROR("Error reading parameter/s (param_adjust_sensitivity) from rosparam. Check hexapod_remote is running and launch file properly set param\n");
    paramAdjustSensitivity = 10.0; //Default to 10.0 (as per default in hexapod_remote)
  }
}

/***********************************************************************************************************************
 * Gets gait parameters from rosparam server
***********************************************************************************************************************/
void StateController::getGaitParameters(std::string forceGait)
{
  std::string baseParamString="/hexapod/parameters/";
  std::string paramString;
  
  if (forceGait.empty())
  {
    if (!n.getParam(baseParamString+"gait_type", params.gaitType))
    {
      ROS_ERROR("Error reading parameter/s (gaitType) from rosparam. Check config file is loaded and type is correct\n");
    }
  }
  else
  {
    params.gaitType = forceGait;
  } 
  
  baseParamString = "/hexapod/gait_parameters/";

  paramString = baseParamString+params.gaitType+"/stance_phase";
  if (!n.getParam(paramString, params.stancePhase))
  {
    ROS_ERROR("Error reading parameter/s (stance_phase) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  paramString = baseParamString+params.gaitType+"/swing_phase";
  if (!n.getParam(paramString, params.swingPhase))
  {
    ROS_ERROR("Error reading parameter/s (swing_phase) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  paramString = baseParamString+params.gaitType+"/phase_offset";
  if (!n.getParam(paramString, params.phaseOffset))
  {
    ROS_ERROR("Error reading parameter/s (phase_offset) from rosparam. Check config file is loaded and type is correct\n");
  }
  
  paramString = baseParamString+params.gaitType+"/offset_multiplier";
  if (!n.getParam(paramString, params.offsetMultiplier))
  {
    ROS_ERROR("Error reading parameter/s (offset_multiplier) from rosparam. Check config file is loaded and type is correct\n");
  } 
}

/***********************************************************************************************************************
***********************************************************************************************************************/


