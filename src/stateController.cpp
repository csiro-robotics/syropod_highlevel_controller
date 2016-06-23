#include "../include/simple_hexapod_controller/stateController.h"

/***********************************************************************************************************************
 * State controller contructor
***********************************************************************************************************************/
StateController::StateController(ros::NodeHandle nodeHandle): n(nodeHandle)
{  
  //Get parameters from parameter server
  getParameters();
  defaultParams = params;
    
  //Initiate model
  hexapod = new Model(params);  
  
  //Populate joint position array with excessive value
  for (int i=0; i<18; i++)
  {
    jointPositions[i] = 1e10;
  }
}

/***********************************************************************************************************************
 * State controller destructor
***********************************************************************************************************************/
StateController::~StateController()
{
  delete hexapod;
  delete interface;
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
  else if (params.gaitType == "balance_gait")
  {
    gait = BALANCE_GAIT;
  }
  
  // Create controller objects
  walker = new WalkController(hexapod, params);
  poser = new PoseController(hexapod, walker, params);
  impedance = new ImpedanceController(params);
        
  setCompensationDebug(debug);
  
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
  
  localVelocity = Vector2d(0.0,0.0);
  tipForce = vector<vector<double> > (3, vector<double>(2));
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
          cout << "Leg: " << leg << ":" << side << " body-coxa joint set to: " << jointPositions[index] << endl;
        }
        if (jointPositions[index+1] == 1e10)
        {
          jointPositions[index+1] = dir*max(0.0,hexapod->minMaxHipLift[0]);
          cout << "Leg: " << leg << ":" << side << " coxa-femour joint set to: " << jointPositions[index+1] << endl;
        }
        if (jointPositions[index+2] == 1e10)
        {
          
          jointPositions[index+2] = dir*max(0.0,hexapod->minMaxKneeBend[0]);
          cout << "Leg: " << leg << ":" << side << " femour-tibia joint set to: " << jointPositions[index+2] << endl;
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
                                                        jointPositions[index+2]));
    }
  }  
}

/***********************************************************************************************************************
 * State machine loop
***********************************************************************************************************************/
void StateController::loop()
{
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
  if (poser->moveToJointPosition(unpackedJointPositions))
  {          
    state = STARTUP;
    cout << "Hexapod unpacked. Running startup sequence . . .\n" << endl;
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
    cout << "Startup sequence complete.\nReady to walk.\n" << endl;
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
    cout << "Shutdown sequence complete. Packing hexapod . . .\n" << endl;
  } 
}

/***********************************************************************************************************************
 * Pack state
***********************************************************************************************************************/
void StateController::packState()
{
  //Move joints directly to packed positions and then transition to next state
  if (poser->moveToJointPosition(packedJointPositions))
  {
    state = PACKED;
    cout << "Hexapod packing complete.\n" << endl;  
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
    cout << "Unpacking hexapod . . .\n" << endl;
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
    
    if (poser->stepToPosition(walker->identityTipPositions, deltaZ, mode, 0, params.timeToStart))
    {
      state = RUNNING;
      cout << "Startup sequence complete. \nReady to walk.\n" << endl;
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
      cout << "WARNING! Hexapod currently in packed state and cannot run direct startup sequence." << endl;
      cout << "Either manually unpack hexapod or set start_up_sequence to true in config file\n" << endl;
      ASSERT(false);
    }
    else
    {
      state = PACKED;
      cout << "Hexapod currently packed.\n" << endl;  
    }
  }
  else if (!params.startUpSequence)
  {    
    state = DIRECT;
    cout << "WARNING! Running direct startup sequence - assuming hexapod is not on the ground" << endl;
    cout << "Running startup sequence (Complete in " << params.timeToStart << " seconds) . . .\n" << endl;       
  }
  else if (startFlag)
  {
    state = STARTUP;
    cout << "Hexapod unpacked. Running startup sequence . . .\n" << endl;
  } 
}

/***********************************************************************************************************************
 * Running state
***********************************************************************************************************************/
void StateController::runningState()
{
  //Compensation
  compensation();
    
  //Impedance control
  if (params.impedanceControl)
  { 
    impedanceControl();
  }
    
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
    //Update walking stance based on desired pose
    poser->updateStance(walker->identityTipPositions, params.autoCompensation);
    
    //Update Walker 
    walker->updateWalk(localVelocity, turnRate, deltaZ, velocityMultiplier); 
  }
  
  //Check for shutdown cue
  if (!startFlag && params.startUpSequence)
  {
    state = SHUTDOWN;
    cout << "Running shutdown sequence . . .\n" << endl;
  }
}

/***********************************************************************************************************************
 * Compensation
***********************************************************************************************************************/
void StateController::compensation()
{
  //Auto Compensation using IMU feedback
  if (params.imuCompensation)
  {  
    //NEEDS REFACTORING
    /*
    Vector2d acc = walker->localCentreAcceleration;
    Vector3d deltaAngle = Vector3d(0,0,0);
    Vector3d deltaPos = Vector3d(0,0,0);

    compensation(Vector3d(acc[0], acc[1], 0),
                walker->angularVelocity, 
                &deltaAngle, &deltaPos, 
                pIncrement, params.timeDelta);
    */
  }        
  //Automatic & Manual compensation running concurrently
  else if (params.autoCompensation && params.manualCompensation)
  {
    if (walker->state == MOVING)
    {
      poser->autoCompensation();
    }
    else if (walker->state == STOPPED)
    {
      Pose targetPose = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitchJoy,rollJoy,yawJoy));
      poser->manualCompensation(targetPose, poseTimeJoy/sqrt(params.stepFrequency));
    }
  }
  //Automatic (non-feedback) compensation
  else if (params.autoCompensation)    
  {   
    poser->autoCompensation();
  }    
  //Manual (joystick controlled) body compensation 
  else if (params.manualCompensation)
  {          
    Pose targetPose = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitchJoy,rollJoy,yawJoy));
    poser->manualCompensation(targetPose, poseTimeJoy/sqrt(params.stepFrequency));
  }
}

/***********************************************************************************************************************
 * Impedance Control
***********************************************************************************************************************/
void StateController::impedanceControl()
{     
  //If all legs are in stance state then update forces on tips
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
  
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      if (walker->legSteppers[l][s].state == SWING || legsInStance == 6)
      {
        double maxForce = 0;
        double minForce = 0;
        if (useTipForce)
        {
          double offset = 1255.0;   
          tipForce[l][s] = tipForces[2*l+s] - offset;
          maxForce = 1000.0;
          minForce = 0.0;
        }
        else if (useJointEffort)
        {                
          int index = 6*l+3*s+1;
          int dir = (s==0) ? -1:1;
          tipForce[l][s] = dir*jointEfforts[index];
          maxForce = 1e9;
          minForce = -1e9;
        }  
        //Ensure force is within limits
        tipForce[l][s] = min(tipForce[l][s], maxForce);
        tipForce[l][s] = max(tipForce[l][s], minForce);
      }
    }
  }     
  
  vector<vector<double> > dZ(3, vector<double >(2));
  dZ = impedance->updateImpedance(tipForce);
  //Transform to Vector3d
  for (int l = 0; l<3; l++)
  {
    for (int s = 0; s<2; s++)
    {
      deltaZ[l][s] = dZ[l][s]; 
    }
  }
}

/***********************************************************************************************************************
 * Dynamic Parameter adjustment
***********************************************************************************************************************/
void StateController::paramAdjust()
{ 
  //Force hexapod to stop walking
  localVelocity = Vector2d(0.0,0.0);
  turnRate = 0.0;
  
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
      default:
      {
        cout << "Attempting to adjust unknown parameter.\n" << endl;
        break;
      }
    }              
    //Update tip Positions for new parameter value
    double stepHeight = walker->maximumBodyHeight*walker->stepClearance;
    if (poser->stepToPosition(walker->identityTipPositions, deltaZ, TRIPOD_MODE, stepHeight, 1.0/(2.0*walker->stepFrequency)))
    {    
      cout << "Parameter '" + paramString + "' set to " << roundToInt(paramScaler*100) << "% of default (" << paramVal << ").\n" << endl;
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
  localVelocity = Vector2d(0.0,0.0);
  turnRate = 0.0;
  
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
      default:
        cout << "Attempting to change to unknown gait." << endl;
        break;
    }   
    walker->setGaitParams(params);
    poser->params = params;                 
    cout << "Now using " << params.gaitType << " mode.\n" << endl;
    changeGait = false;
  }
}

/***********************************************************************************************************************
 * Leg State Toggle
***********************************************************************************************************************/
void StateController::legStateToggle()
{ 
  int l = legSelection/2;
  int s = legSelection%2;
  if (hexapod->legs[l][s].state == WALKING)
  {
    hexapod->legs[l][s].state = OFF;
    cout << "Leg: " << legSelection/2 << ":" << legSelection%2 << " set to state: OFF.\n" << endl; 
  }
  else if (hexapod->legs[l][s].state == OFF)
  {
    hexapod->legs[l][s].state = WALKING;
    cout << "Leg: " << legSelection/2 << ":" << legSelection%2 << " set to state: WALKING.\n" << endl;
  }
  toggleLegState = false;
}

/***********************************************************************************************************************
 * Publisheds tip positions for debugging
***********************************************************************************************************************/
void StateController::publishTipPositions()
{
  std_msgs::Float32MultiArray msg;
  for (int s = 0; s<2; s++)
  {
    for (int l = 0; l<3; l++)
    {        
      msg.data.clear();
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[0]);
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[1]);
      msg.data.push_back(hexapod->legs[l][s].localTipPosition[2]);
      tipPositionPublishers[l][s].publish(msg);
    }
  }
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
  
  //cout << "Tip Positions Array Size: " << debug.tipPositions.size() << endl;
  //cout << "Static Tip Positions Array Size: " << debug.staticTipPositions.size() << endl;
  
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
      double knee = dir*hexapod->legs[l][s].kneeAngle;
      
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
          cout << "Leg: " << l << ":" << s << " body_coxa joint velocity (" << yawVel << ") exceeds maximum ("; 
          cout << sign(yawVel)*hexapod->jointMaxAngularSpeeds[0] << ") - CLAMPING TO MAXIMUM!" << endl; 
          yawVel = sign(yawVel)*hexapod->jointMaxAngularSpeeds[0];
          yaw = hexapod->legs[l][s].oldYaw + yawVel*params.timeDelta;
        }
        if (abs(liftVel) > hexapod->jointMaxAngularSpeeds[1])
        {
          cout << "Leg: " << l << ":" << s << " coxa_femour joint velocity (" << liftVel << ") exceeds maximum (";
          cout << sign(liftVel)*hexapod->jointMaxAngularSpeeds[1] << ") - CLAMPING TO MAXIMUM!" << endl;                 
          liftVel = sign(liftVel)*hexapod->jointMaxAngularSpeeds[1];
          lift = hexapod->legs[l][s].oldLiftAngle + liftVel*params.timeDelta;
        }
        if (abs(kneeVel) > hexapod->jointMaxAngularSpeeds[2])
        {
          cout << "Leg: " << l << ":" << s << " femour_tibia joint velocity (" << kneeVel << ") exceeds maximum (";
          cout << sign(kneeVel)*hexapod->jointMaxAngularSpeeds[2] << ") - CLAMPING TO MAXIMUM!" << endl; 
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
              
      //interface->setVelocity(l, s, 0, yawVel); //Removed due to issues with Large Hexapod
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
        cout << "No parameters currently selected.\n" << endl;
        break;
      }
      case(STEP_FREQUENCY):
      {
        paramSelection = STEP_FREQUENCY;
        cout << "step_frequency parameter selected.\n" << endl;
        break;
      }
      case(STEP_CLEARANCE):
      {
        paramSelection = STEP_CLEARANCE;
        cout << "step_clearance parameter selected.\n" << endl;
        break;
      }
      case(BODY_CLEARANCE):
      {
        paramSelection = BODY_CLEARANCE;
        cout << "body_clearance parameter selected.\n" << endl;
        break;
      }      
      case(LEG_SPAN_SCALE):
      {
        paramSelection = LEG_SPAN_SCALE;
        cout << "leg_span_scale parameter selected.\n" << endl;
        break;
      }
      default:
      {
        paramSelection = NO_PARAM_SELECTION;
        cout << "Unknown parameter selection requested from control input. No parameters currently selected.\n" << endl;
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
      
      cout << "Adjusting selected parameter . . . (WARNING: Changing parameters may crash controller)\n" << endl;
    }
    else
    {
      cout << "Checking selected parameter value . . .\n" << endl;
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
        cout << "Transitioning to tripod_gait mode . . .\n" << endl;
        gait = TRIPOD_GAIT;
        break;
      case(RIPPLE_GAIT):
        cout << "Transitioning to ripple_gait mode . . .\n" << endl;
        gait = RIPPLE_GAIT;
        break;
      case(WAVE_GAIT):
        cout << "Transitioning to wave_gait mode . . .\n" << endl;
        gait = WAVE_GAIT;
        break;
      default:
        cout << "Unknown gait requested from control input.\n" << endl;
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
  if (state == RUNNING)
  {
    switch (input.data)
    {
      case(-1):
        break;
      case(0):
        if (legSelection != FRONT_LEFT)
        {
          legSelection = FRONT_LEFT;
          cout << "Front left leg selected.\n" << endl;
        }
        break;
      case(1):
        if (legSelection != FRONT_RIGHT)
        {
          legSelection = FRONT_RIGHT;
          cout << "Front right leg selected.\n" << endl;
        }
        break;
      case(2):
        if (legSelection != MIDDLE_LEFT)
        {
          legSelection = MIDDLE_LEFT;
          cout << "Middle left leg selected.\n" << endl;
        }
        break;
      case(3):
        if (legSelection != MIDDLE_RIGHT)
        {
          legSelection = MIDDLE_RIGHT;
          cout << "Middle right leg selected.\n" << endl;
        }
        break;
      case(4):
        if (legSelection != REAR_LEFT)
        {
          legSelection = REAR_LEFT;
          cout << "Rear left leg selected.\n" << endl;
        }
        break;
      case(5):
        if (legSelection != REAR_RIGHT)
        {
          legSelection = REAR_RIGHT;
          cout << "Rear right leg selected.\n" << endl;
        }
        break;
      default:
        cout << "Unknown leg selection requested from control input.\n" << endl;
    }
  }
}

/***********************************************************************************************************************
 * Joypad Velocity Topic Callback
***********************************************************************************************************************/
void StateController::joypadVelocityCallback(const geometry_msgs::Twist &twist)
{
  velocityMultiplier = (-0.5*twist.linear.z+1.5); //Range: 1.0->2.0
  localVelocity = Vector2d(twist.linear.x, twist.linear.y);
  //localVelocity = clamped(localVelocity, 1.0);

  turnRate = twist.angular.x;
  turnRate = turnRate*turnRate*turnRate; // the cube just lets the thumbstick give small turns easier
    
  //Allows rotation without velocity input
  if (localVelocity.norm() == 0.0)
  {
    localVelocity = Vector2d(0, twist.angular.x);
    turnRate = sign(twist.angular.x);
  }
  
  poseTimeJoy = params.maxPoseTime*(0.5*twist.angular.z+0.5); //Range: maxPoseTime->0.0
}

/***********************************************************************************************************************
 * Joypad Pose Topic Callback
***********************************************************************************************************************/
void StateController::joypadPoseCallback(const geometry_msgs::Twist &twist)
{
  double deadband = 0.1; //%10 deadband
  
  double newRollJoy = twist.angular.x*params.maxRoll;
  double newPitchJoy = twist.angular.y*params.maxPitch;
  double newYawJoy = twist.angular.z*params.maxYaw;
  double newXJoy = twist.linear.x*params.maxX;
  double newYJoy = twist.linear.y*params.maxY;
  double newZJoy = twist.linear.z >= 0 ? twist.linear.z*params.maxZ : twist.linear.z*params.minZ;
  
  //Percentage change is above deadband value
  if (abs(newRollJoy-rollJoy) > deadband*abs(rollJoy))
  {
    rollJoy = newRollJoy;
  }
  if (abs(newPitchJoy-pitchJoy) > deadband*abs(pitchJoy))
  {
    pitchJoy = newPitchJoy;
  }
  if (abs(newYawJoy-yawJoy) > deadband*abs(yawJoy))
  {
    yawJoy = newYawJoy;
  }
  if (abs(newXJoy-xJoy) > deadband*abs(xJoy))
  {
    xJoy = newXJoy;
  }
  if (abs(newYJoy-yJoy) > deadband*abs(yJoy))
  {
    yJoy = newYJoy;
  }
  if (abs(newZJoy-zJoy) > deadband*abs(zJoy))
  {
    zJoy = newZJoy; 
  }
}

/***********************************************************************************************************************
 * Joypad Start State Topic Callback
***********************************************************************************************************************/
void StateController::startCallback(const std_msgs::Bool &input)
{
  startFlag = input.data;
}

/***********************************************************************************************************************
 * Toggle Leg State Callback
***********************************************************************************************************************/
void StateController::legStateCallback(const std_msgs::Bool &input)
{
  if (input.data && debounce)
  {
    toggleLegState = true;
    debounce = false;
  }
  else if (!input.data && !toggleLegState)
  {
    debounce = true;
  }
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void StateController::jointStatesCallback(const sensor_msgs::JointState &jointStates)
{  
  bool getEffortValues;
  if (jointStates.effort.size() == 0)
  {
    getEffortValues = false;
    cout << "Warning! Effort values not being published! Impedance controller may not function.\n" << endl;
  }
  else
  {
    getEffortValues = true;
  }
  
  for (uint i=0; i<jointStates.name.size(); i++)
  {
    const char* jointName = jointStates.name[i].c_str();
    if (!strcmp(jointName, "front_left_body_coxa") ||
        !strcmp(jointName, "AL_coxa_joint"))
    {
      jointPositions[0] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[0] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_coxa_femour") ||
              !strcmp(jointName, "AL_femur_joint"))
    {
      jointPositions[1] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[1] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_left_femour_tibia") ||
              !strcmp(jointName, "AL_tibia_joint"))
    {
      jointPositions[2] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[2] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_body_coxa") ||
              !strcmp(jointName, "AR_coxa_joint"))
    {
      jointPositions[3] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[3] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_coxa_femour") ||
              !strcmp(jointName, "AR_femur_joint"))
    {
      jointPositions[4] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[4] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "front_right_femour_tibia") ||
              !strcmp(jointName, "AR_tibia_joint"))
    {
      jointPositions[5] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[5] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_body_coxa") ||
              !strcmp(jointName, "BL_coxa_joint"))
    {
      jointPositions[6] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[6] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_coxa_femour") ||
              !strcmp(jointName, "BL_femur_joint"))
    {
      jointPositions[7] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[7] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_left_femour_tibia") ||
              !strcmp(jointName, "BL_tibia_joint"))
    {
      jointPositions[8] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[8] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_body_coxa") ||
              !strcmp(jointName, "BR_coxa_joint"))
    {
      jointPositions[9] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[9] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_coxa_femour") ||
              !strcmp(jointName, "BR_femur_joint"))
    {
      jointPositions[10] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[10] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "middle_right_femour_tibia") ||
              !strcmp(jointName, "BR_tibia_joint"))
    {
      jointPositions[11] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[11] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_body_coxa") ||
              !strcmp(jointName, "CL_coxa_joint"))
    {
      jointPositions[12] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[12] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_coxa_femour") ||
              !strcmp(jointName, "CL_femur_joint"))
    {
      jointPositions[13] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[13] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_left_femour_tibia") ||
              !strcmp(jointName, "CL_tibia_joint"))
    {
      jointPositions[14] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[14] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_body_coxa") ||
              !strcmp(jointName, "CR_coxa_joint"))
    {
      jointPositions[15] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[15] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_coxa_femour") ||
              !strcmp(jointName, "CR_femur_joint"))
    {
      jointPositions[16] = jointStates.position[i];
      if (getEffortValues) 
      {
        jointEfforts[16] = jointStates.effort[i];
      }
    }
    else if (!strcmp(jointName, "rear_right_femour_tibia") ||
              !strcmp(jointName, "CR_tibia_joint"))
    {
      jointPositions[17] = jointStates.position[i];
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
        jointPosFlag = false;
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
    cout << "Error reading parameter/s (hexapod_type) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam(baseParamString+"time_delta", params.timeDelta))
  {
    cout << "Error reading parameter/s (time_delta) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
    
  if(!n.getParam(baseParamString+"imu_compensation", params.imuCompensation))
  {
    cout << "Error reading parameter/s (imu_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(baseParamString+"auto_compensation", params.autoCompensation))
  {
    cout << "Error reading parameter/s (auto_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(baseParamString+"manual_compensation", params.manualCompensation))
  {
    cout << "Error reading parameter/s (manual_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
   
  /********************************************************************************************************************/
  //Offset Parameters
  //Root Offset Parameters
  std::vector<double> rootOffsetAL(3);
  paramString=baseParamString+"/physical_leg_offsets/";
  if(!n.getParam(paramString+"root_offset_AL", rootOffsetAL))
  {
    cout << "Error reading parameter/s (root_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.rootOffset[0][0] = Map<Vector3d>(&rootOffsetAL[0], 3);
  }
  
  std::vector<double> rootOffsetAR(3);
  if(!n.getParam(paramString+"root_offset_AR", rootOffsetAR))
  {
    cout << "Error reading parameter/s (root_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.rootOffset[0][1] = Map<Vector3d>(&rootOffsetAR[0], 3);
  }
  
  std::vector<double> rootOffsetBL(3);
  if(!n.getParam(paramString+"root_offset_BL", rootOffsetBL))
  {
    cout << "Error reading parameter/s (root_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.rootOffset[1][0] = Map<Vector3d>(&rootOffsetBL[0], 3);
  }
  
  std::vector<double> rootOffsetBR(3);
  if(!n.getParam(paramString+"root_offset_BR", rootOffsetBR))
  {
    cout << "Error reading parameter/s (root_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.rootOffset[1][1] = Map<Vector3d>(&rootOffsetBR[0], 3);
  }
  
    std::vector<double> rootOffsetCL(3);
  if(!n.getParam(paramString+"root_offset_CL", rootOffsetCL))
  {
    cout << "Error reading parameter/s (root_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.rootOffset[2][0] = Map<Vector3d>(&rootOffsetCL[0], 3);
  }
  
  std::vector<double> rootOffsetCR(3);
  if(!n.getParam(paramString+"root_offset_CR", rootOffsetCR))
  {
    cout << "Error reading parameter/s (root_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (hip_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipOffset[0][0] = Map<Vector3d>(&hipOffsetAL[0], 3);
  }
  
  std::vector<double> hipOffsetAR(3);
  if(!n.getParam(paramString+"hip_offset_AR", hipOffsetAR))
  {
    cout << "Error reading parameter/s (hip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipOffset[0][1] = Map<Vector3d>(&hipOffsetAR[0], 3);
  }
  
  std::vector<double> hipOffsetBL(3);
  if(!n.getParam(paramString+"hip_offset_BL", hipOffsetBL))
  {
    cout << "Error reading parameter/s (hip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipOffset[1][0] = Map<Vector3d>(&hipOffsetBL[0], 3);
  }
  
  std::vector<double> hipOffsetBR(3);
  if(!n.getParam(paramString+"hip_offset_BR", hipOffsetBR))
  {
    cout << "Error reading parameter/s (hip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipOffset[1][1] = Map<Vector3d>(&hipOffsetBR[0], 3);
  }
  
    std::vector<double> hipOffsetCL(3);
  if(!n.getParam(paramString+"hip_offset_CL", hipOffsetCL))
  {
    cout << "Error reading parameter/s (hip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipOffset[2][0] = Map<Vector3d>(&hipOffsetCL[0], 3);
  }
  
  std::vector<double> hipOffsetCR(3);
  if(!n.getParam(paramString+"hip_offset_CR", hipOffsetCR))
  {
    cout << "Error reading parameter/s (hip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (knee_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeOffset[0][0] = Map<Vector3d>(&kneeOffsetAL[0], 3);
  }
  
  std::vector<double> kneeOffsetAR(3);
  if(!n.getParam(paramString+"knee_offset_AR", kneeOffsetAR))
  {
    cout << "Error reading parameter/s (knee_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeOffset[0][1] = Map<Vector3d>(&kneeOffsetAR[0], 3);
  }
  
  std::vector<double> kneeOffsetBL(3);
  if(!n.getParam(paramString+"knee_offset_BL", kneeOffsetBL))
  {
    cout << "Error reading parameter/s (knee_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeOffset[1][0] = Map<Vector3d>(&kneeOffsetBL[0], 3);
  }
  
  std::vector<double> kneeOffsetBR(3);
  if(!n.getParam(paramString+"knee_offset_BR", kneeOffsetBR))
  {
    cout << "Error reading parameter/s (knee_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeOffset[1][1] = Map<Vector3d>(&kneeOffsetBR[0], 3);
  }
  
    std::vector<double> kneeOffsetCL(3);
  if(!n.getParam(paramString+"knee_offset_CL", kneeOffsetCL))
  {
    cout << "Error reading parameter/s (knee_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeOffset[2][0] = Map<Vector3d>(&kneeOffsetCL[0], 3);
  }
  
  std::vector<double> kneeOffsetCR(3);
  if(!n.getParam(paramString+"knee_offset_CR", kneeOffsetCR))
  {
    cout << "Error reading parameter/s (knee_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (tip_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.tipOffset[0][0] = Map<Vector3d>(&tipOffsetAL[0], 3);
  }
  
  std::vector<double> tipOffsetAR(3);
  if(!n.getParam(paramString+"tip_offset_AR", tipOffsetAR))
  {
    cout << "Error reading parameter/s (tip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.tipOffset[0][1] = Map<Vector3d>(&tipOffsetAR[0], 3);
  }
  
  std::vector<double> tipOffsetBL(3);
  if(!n.getParam(paramString+"tip_offset_BL", tipOffsetBL))
  {
    cout << "Error reading parameter/s (tip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.tipOffset[1][0] = Map<Vector3d>(&tipOffsetBL[0], 3);
  }
  
  std::vector<double> tipOffsetBR(3);
  if(!n.getParam(paramString+"tip_offset_BR", tipOffsetBR))
  {
    cout << "Error reading parameter/s (tip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.tipOffset[1][1] = Map<Vector3d>(&tipOffsetBR[0], 3);
  }
  
    std::vector<double> tipOffsetCL(3);
  if(!n.getParam(paramString+"tip_offset_CL", tipOffsetCL))
  {
    cout << "Error reading parameter/s (tip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.tipOffset[2][0] = Map<Vector3d>(&tipOffsetCL[0], 3);
  }
  
  std::vector<double> tipOffsetCR(3);
  if(!n.getParam(paramString+"tip_offset_CR", tipOffsetCR))
  {
    cout << "Error reading parameter/s (tip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (stance_leg_yaws) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.stanceLegYaws = Map<Vector3d>(&stanceLegYaws[0], 3);
  }
  
  std::vector<double> physicalYawOffset(3);
  if(!n.getParam(baseParamString+"physical_yaw_offset", physicalYawOffset))
  {
    cout << "Error reading parameter/s (stance_leg_yaws) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.physicalYawOffset = Map<Vector3d>(&physicalYawOffset[0], 3);
  }
  /********************************************************************************************************************/
  //Joint Limit Parameters  
  paramString = baseParamString+"/joint_limits/";
  std::vector<double> yawLimits(3);
  if(!n.getParam(paramString+"yaw_limits", yawLimits))
  {
    cout << "Error reading parameter/s (yaw_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.yawLimits = Map<Vector3d>(&yawLimits[0], 3);
  }
    
  std::vector<double> kneeLimits(2);
  if(!n.getParam(paramString+"knee_limits", kneeLimits))
  {
    cout << "Error reading parameter/s (knee_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.kneeLimits = Map<Vector2d>(&kneeLimits[0], 2);
  }
  
  std::vector<double> hipLimits(2);    
  if(!n.getParam(paramString+"hip_limits", hipLimits))
  {
    cout << "Error reading parameter/s (hip_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.hipLimits = Map<Vector2d>(&hipLimits[0], 2);
  }
  
  std::vector<double> jointMaxAngularSpeeds(2); 
  if(!n.getParam(baseParamString+"joint_max_angular_speeds", jointMaxAngularSpeeds))
  {
    cout << "Error reading parameter/s (joint_max_angular_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.jointMaxAngularSpeeds = Map<Vector3d>(&jointMaxAngularSpeeds[0], 3);
  }
  
  if(!n.getParam(baseParamString+"dynamixel_interface", params.dynamixelInterface))
  {
    cout << "Error reading parameter/s (dynamixel_interface) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /********************************************************************************************************************/
  // Walk Controller Parameters
  paramString = baseParamString+"/walk_controller/";
  
  if (!n.getParam(paramString+"step_frequency", params.stepFrequency))
  {
    cout << "Error reading parameter/s (step_frequency) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"step_clearance", params.stepClearance))
  {
    cout << "Error reading parameter/s (step_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"body_clearance", params.bodyClearance))
  {
    cout << "Error reading parameter/s (body_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"leg_span_scale", params.legSpanScale))
  {
    cout << "Error reading parameter/s (leg_span_scale) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam(paramString+"leg_state_correction", params.legStateCorrection))
  {
    cout << "Error reading parameter/s (leg_state_correction) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if (!n.getParam(paramString+"max_acceleration", params.maxAcceleration))
  {
    cout << "Error reading parameter/s (max_acceleration) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_curvature_speed", params.maxCurvatureSpeed))
  {
    cout << "Error reading parameter/s (max_curvature_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"step_curvature_allowance", params.stepCurvatureAllowance))
  {
    cout << "Error reading parameter/s (step_curvature_allowance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"interface_setup_speed", params.interfaceSetupSpeed))
  {
    cout << "Error reading parameter/s (interface_setup_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /********************************************************************************************************************/
  // Pose Controller Parameters 
  paramString = baseParamString+"/pose_controller/";
  
  if(!n.getParam(paramString+"start_up_sequence", params.startUpSequence))
  {
    cout << "Error reading parameter/s (start_up_sequence) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"move_legs_sequentially", params.moveLegsSequentially))
  {
    cout << "Error reading parameter/s (move_legs_sequentially) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"time_to_start", params.timeToStart))
  {
    cout << "Error reading parameter/s (time_to_start) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  paramString = baseParamString+"/pose_controller/auto_pose_compensation/";
  if(!n.getParam(paramString+"pitch_amplitude", params.pitchAmplitude))
  {
    cout << "Error reading parameter/s (pitch_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"roll_amplitude", params.rollAmplitude))
  {
    cout << "Error reading parameter/s (roll_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  paramString = baseParamString+"/pose_controller/manual_pose_compensation/";
  if (!n.getParam(paramString+"max_pose_time", params.maxPoseTime))
  {
    cout << "Error reading parameter/s (max_pose_time) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_roll", params.maxRoll))
  {
    cout << "Error reading parameter/s (max_roll) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_pitch", params.maxPitch))
  {
    cout << "Error reading parameter/s (max_pitch) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_yaw", params.maxYaw))
  {
    cout << "Error reading parameter/s (max_yaw) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_x", params.maxX))
  {
    cout << "Error reading parameter/s (max_x) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_y", params.maxY))
  {
    cout << "Error reading parameter/s (max_y) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_z", params.maxZ))
  {
    cout << "Error reading parameter/s (max_z) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  } 
  
  if (!n.getParam(paramString+"min_z", params.minZ))
  {
    cout << "Error reading parameter/s (min_z) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  } 
  
  /********************************************************************************************************************/
  
  paramString = baseParamString + "/pose_controller/packed_joint_positions/";  
  std::vector<double> packedJointPositionsAL(3);
  if (!n.getParam(paramString+"AL_packed_joint_positions", packedJointPositionsAL))
  {
    cout << "Error reading parameter/s (AL_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.packedJointPositionsAL = Map<Vector3d>(&packedJointPositionsAL[0], 3);
  }
    
  std::vector<double> packedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_packed_joint_positions", packedJointPositionsAR))
  {
    cout << "Error reading parameter/s (AR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.packedJointPositionsAR = Map<Vector3d>(&packedJointPositionsAR[0], 3);
  }
  
  std::vector<double> packedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_packed_joint_positions", packedJointPositionsBL))
  {
    cout << "Error reading parameter/s (BL_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.packedJointPositionsBL = Map<Vector3d>(&packedJointPositionsBL[0], 3);
  }
  
  std::vector<double> packedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_packed_joint_positions", packedJointPositionsBR))
  {
    cout << "Error reading parameter/s (BR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.packedJointPositionsBR = Map<Vector3d>(&packedJointPositionsBR[0], 3);
  }
  
  std::vector<double> packedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_packed_joint_positions", packedJointPositionsCL))
  {
    cout << "Error reading parameter/s (CL_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.packedJointPositionsCL = Map<Vector3d>(&packedJointPositionsCL[0], 3);
  }
  
  std::vector<double> packedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_packed_joint_positions", packedJointPositionsCR))
  {
    cout << "Error reading parameter/s (CR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (AL_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.unpackedJointPositionsAL = Map<Vector3d>(&unpackedJointPositionsAL[0], 3);
  }
    
  std::vector<double> unpackedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_unpacked_joint_positions", unpackedJointPositionsAR))
  {
    cout << "Error reading parameter/s (AR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.unpackedJointPositionsAR = Map<Vector3d>(&unpackedJointPositionsAR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_unpacked_joint_positions", unpackedJointPositionsBL))
  {
    cout << "Error reading parameter/s (BL_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.unpackedJointPositionsBL = Map<Vector3d>(&unpackedJointPositionsBL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_unpacked_joint_positions", unpackedJointPositionsBR))
  {
    cout << "Error reading parameter/s (BR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.unpackedJointPositionsBR = Map<Vector3d>(&unpackedJointPositionsBR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_unpacked_joint_positions", unpackedJointPositionsCL))
  {
    cout << "Error reading parameter/s (CL_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params.unpackedJointPositionsCL = Map<Vector3d>(&unpackedJointPositionsCL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_unpacked_joint_positions", unpackedJointPositionsCR))
  {
    cout << "Error reading parameter/s (CR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (impedance_control) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }

  if (!n.getParam(paramString+"integrator_step_time", params.integratorStepTime))
  {
    cout << "Error reading parameter/s (integrator_step_time) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }

  if (!n.getParam(paramString+"virtual_mass", params.virtualMass))
  {
    cout << "Error reading parameter/s (virtual_mass) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }

  if (!n.getParam(paramString+"virtual_stiffness", params.virtualStiffness))
  {
    cout << "Error reading parameter/s (virtual_stiffness) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }

  if (!n.getParam(paramString+"virtual_damping_ratio", params.virtualDampingRatio))
  {
    cout << "Error reading parameter/s (virtual_damping_ratio) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }

  if (!n.getParam(paramString+"force_gain", params.forceGain))
  {
    cout << "Error reading parameter/s (force_gain) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
   
  /********************************************************************************************************************/
  //Debug Parameters (set via launch file instead of by config files)
  
  paramString = "/hexapod/debug_parameters/rviz";
  if (!n.getParam(paramString, params.debug_rviz))
  {
    params.debug_rviz = false;
  }
  
  /********************************************************************************************************************/
  //Hexapod remote parameters (set via hexapod_remote launch file instead of by config files)
  paramString = "/hexapod_remote/param_adjust_sensitivity";
  if (!n.getParam(paramString, paramAdjustSensitivity))
  {
    cout << "Error reading parameter/s (param_adjust_sensitivity) from rosparam" <<endl; 
    cout << "Check hexapod_remote is running and launch file properly set param" << endl;
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
      cout << "Error reading parameter/s (gaitType) from rosparam" <<endl; 
      cout << "Check config file is loaded and type is correct" << endl;
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
    cout << "Error reading parameter/s (stance_phase) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = baseParamString+params.gaitType+"/swing_phase";
  if (!n.getParam(paramString, params.swingPhase))
  {
    cout << "Error reading parameter/s (swing_phase) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = baseParamString+params.gaitType+"/phase_offset";
  if (!n.getParam(paramString, params.phaseOffset))
  {
    cout << "Error reading parameter/s (phase_offset) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = baseParamString+params.gaitType+"/offset_multiplier";
  if (!n.getParam(paramString, params.offsetMultiplier))
  {
    cout << "Error reading parameter/s (offset_multiplier) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = baseParamString+params.gaitType+"/transition_period";
  if (!n.getParam(paramString, params.transitionPeriod))
  {
    cout << "Error reading parameter/s (transition_period) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/


