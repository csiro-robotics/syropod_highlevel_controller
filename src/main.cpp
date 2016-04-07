/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/walkController.h"
#include "../include/simple_hexapod_controller/poseController.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include "../include/simple_hexapod_controller/imuCompensation.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/Joy.h"

enum State
{
  UNPACK,
  STARTUP,
  RUNNING,
  GAIT_TRANSITION,
  SHUTDOWN,
  PACK,
  PACKED,
  DIRECT,
  UNKNOWN,  
};

enum Gait
{
  TRIPOD_GAIT,
  RIPPLE_GAIT,
  WAVE_GAIT,
  DEFAULT,
};

enum LegSelection
{
  FRONT_LEFT,
  FRONT_RIGHT,
  MIDDLE_LEFT,
  MIDDLE_RIGHT,
  REAR_LEFT,
  REAR_RIGHT,
};

//Globals for joypad callbacks
static Parameters *pParams;

State state = UNKNOWN;

Gait gait = DEFAULT;
LegSelection legSelection = FRONT_LEFT;

static Vector2d localVelocity(0,0);
static double turnRate = 0;

static double pitchJoy = 0;
static double rollJoy = 0;
static double yawJoy = 0;
static double xJoy = 0;
static double yJoy = 0;
static double zJoy = 0;

static double poseTimeJoy = 0.2;
static double stepFrequencyMultiplier = 1.0;

double pIncrement=0;
bool toggleLegState = false;
bool debounce = false;

//Globals for joint states callback
sensor_msgs::JointState jointStates;
double jointPositions[18];
bool jointPosFlag = false;
bool startFlag = false;

void jointStatesCallback(const sensor_msgs::JointState &joint_States);
void joypadVelocityCallback(const geometry_msgs::Twist &twist);
void joypadPoseCallback(const geometry_msgs::Twist &twist);
void imuControllerCallback(const sensor_msgs::Joy &input);
void gaitSelectionCallback(const std_msgs::Int8 &input);
void legSelectionCallback(const std_msgs::Int8 &input);
void legStateCallback(const std_msgs::Bool &input);
void startCallback(const std_msgs::Bool &input);

void getParameters(ros::NodeHandle n, Parameters *params, std::string forceGait);

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");  
  
  ros::Publisher controlPub = n.advertise<geometry_msgs::Vector3>("controlsignal", 1000);
  
  ros::Subscriber velocitySubscriber = n.subscribe("hexapod_remote/desired_velocity", 1, joypadVelocityCallback);
  ros::Subscriber poseSubscriber = n.subscribe("hexapod_remote/desired_pose", 1, joypadPoseCallback);
  ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data", 1, imuCallback);
  ros::Subscriber imuControlSubscriber = n.subscribe("/joy", 1, imuControllerCallback);
  ros::Subscriber legSelectSubscriber = n.subscribe("hexapod_remote/leg_selection", 1, legSelectionCallback);
  ros::Subscriber legStateSubscriber = n.subscribe("hexapod_remote/leg_state_toggle", 1, legStateCallback);
  
  ros::Subscriber startSubscriber = n.subscribe("hexapod_remote/start_state", 1, startCallback);
  ros::Subscriber gaitSelectSubscriber = n.subscribe("hexapod_remote/gait_mode", 1, gaitSelectionCallback);
  ros::Subscriber jointStatesSubscriber;  
    
  //Get parameters from rosparam via loaded config file
  Parameters params;
  pParams = &params;
  std::string forceGait; //Feed empty string for unforced gait choice
  getParameters(n, &params, forceGait);
  
  ros::Rate r(roundToInt(1.0/params.timeDelta));         //frequency of the loop. 
  
  //Start User Message
  cout << "Press 'Start' to run controller" << endl;
  
  if (params.debug_rviz)
    cout << "WARNING: DEBUGGING USING RVIZ - CODE IS CPU INTENSIVE." << endl;
  
  //Loop waiting for start button press
  while(!startFlag)
  {
    ros::spinOnce();
    r.sleep();
  }  
  
  //Create hexapod model    
  Model hexapod(params);  
  
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
  
  //Get current joint positions
  jointStatesSubscriber = n.subscribe("/hexapod/joint_states", 1, jointStatesCallback);    
  if(!jointStatesSubscriber)
  {
    cout << "Failed to subscribe to joint_states topic - check to see if topic is being published." << endl;
    params.startUpSequence = false;
  }
  else
  {
    for (int i=0; i<18; i++)
      jointPositions[i] = 1e10;
  
    //Wait for joint positions via callback
    int spin = 1/params.timeDelta; //Max ros spin cycles to find joint positions
    while(spin--)
    {
      ros::spinOnce();
      r.sleep();
    }
  }     
  if (jointPosFlag)
  {
    // set initial leg angles
    for (int leg = 0; leg<3; leg++)
    {
      for (int side = 0; side<2; side++)
      {
        double dir = side==0 ? -1 : 1;
        int index = leg*6+(side == 0 ? 0 : 3);
        hexapod.setLegStartAngles(side, leg, dir*Vector3d(jointPositions[index+0]+dir*params.physicalYawOffset[leg],
                                                          -jointPositions[index+1], 
                                                          jointPositions[index+2]));
      }
    }
  }
  else
  {
    cout << "Failed to acquire all joint position values." << endl;
    cout << "WARNING: ASSUMING STARTING JOINT POSITIONS ARE AT ZERO!" << endl;
    params.startUpSequence = false;
  }
  
  // Create walk and pose controller objects
  WalkController walker(&hexapod, params);
  PoseController poser(&hexapod, params);
    
  DebugOutput debug;
  setCompensationDebug(debug);

  //Setup motor interaface
  MotorInterface *interface;
  
  if (params.dynamixelInterface)
    interface = new DynamixelMotorInterface();  
  else
    interface = new DynamixelProMotorInterface();
  
  interface->setupSpeed(params.interfaceSetupSpeed);   
  
  //Startup/Shutdown variables
  double heightRatio = -1;
  double stepHeight = -1;
  
  //Packed/Unpacked joint position arrays
  Vector3d packedJointPositions[3][2];
  Vector3d unpackedJointPositions[3][2];
  
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

  //Position update loop
  while (ros::ok())
  {
    Pose adjust = Pose::identity();
    
    //Auto Compensation using IMU feedback
    if (params.imuCompensation)
    {      
      Vector2d acc = walker.localCentreAcceleration;
      Vector3d deltaAngle = Vector3d(0,0,0);
      Vector3d deltaPos = Vector3d(0,0,0);

      compensation(Vector3d(acc[0], acc[1], 0),
                   walker.angularVelocity, 
                   &deltaAngle, &deltaPos, 
                   pIncrement, params.timeDelta);
    }
    //Automatic (non-feedback) compensation
    else if (params.autoCompensation)    
    {      
      double pitch = poser.getPitchCompensation(walker.legSteppers[0][0].phase);
      double roll = poser.getRollCompensation(walker.legSteppers[0][0].phase); 
      if (gait == WAVE_GAIT)
        adjust = Pose(Vector3d(0,0,0), Quat(1,pitch,roll,0));
      else if (gait == TRIPOD_GAIT)
        adjust = Pose(Vector3d(0,0,0), Quat(1,0,roll,0));
    }    
    //Manual (joystick controlled) body compensation 
    else if (params.manualCompensation)
    {          
      adjust = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitchJoy,rollJoy,yawJoy));
    }         
    
    //Robot state machine
    switch (state)
    {
      //Robot is in a state of unpacking itself into neutral 'unpacked' position
      case(UNPACK):
      {
        if (poser.moveToJointPosition(unpackedJointPositions))
        {
          state = STARTUP;
          heightRatio = poser.createSequence(walker);
          stepHeight = walker.stepClearance*walker.maximumBodyHeight;
        }
        break;
      }
        
      //Robot steps safely from unpacked position into the operational 'walking' position
      case(STARTUP):
      {
        if (heightRatio == -1 || stepHeight == -1)
        {
          heightRatio = poser.createSequence(walker);
          stepHeight = walker.stepClearance*walker.maximumBodyHeight;
        }
        
        if (poser.startUpSequence(heightRatio, stepHeight, params.moveLegsSequentially))  
        {            
          state = RUNNING;
          cout << "Startup sequence complete. \nReady to walk." << endl;
        }
        break;
      }
      
      //Robot is in operational state (walking/posing/actuating etc.)
      case(RUNNING):
      {
        if (!startFlag && params.startUpSequence)
        {
          state = SHUTDOWN;
          cout << "Running shutdown sequence . . ." << endl;
          heightRatio = poser.createSequence(walker);
          stepHeight = walker.stepClearance*walker.maximumBodyHeight;
        }
        else 
        {   
          //Update Walker and Poser
          double poseTime = params.manualCompensation ? poseTimeJoy : 0.0;
          poser.updateStance(walker.identityTipPositions, adjust, poseTime);
          walker.updateWalk(localVelocity, turnRate, stepFrequencyMultiplier);
          
          //Switch gait parameters and create new walker instance
          if ((gait == TRIPOD_GAIT && params.gaitType != "tripod_gait") ||
              (gait == RIPPLE_GAIT && params.gaitType != "ripple_gait") ||
              (gait == WAVE_GAIT && params.gaitType != "wave_gait"))
          {
            state = GAIT_TRANSITION;
          }          
          
          //Leg Selection for toggling state          
          if (toggleLegState)
          {
            int l = legSelection/2;
            int s = legSelection%2;
            if (hexapod.legs[l][s].state == WALKING)
            {
              hexapod.legs[l][s].state = OFF;
              cout << "Leg: " << legSelection/2 << ":" << legSelection%2 << " set to state: OFF." << endl; 
            }
            else if (hexapod.legs[l][s].state == OFF)
            {
              hexapod.legs[l][s].state = WALKING;
              cout << "Leg: " << legSelection/2 << ":" << legSelection%2 << " set to state: WALKING." << endl;
            }
            toggleLegState = false;
          } 
        }
        break;
      } 
      
      case(GAIT_TRANSITION):
      {
        if (poser.stepToPosition(walker.identityTipPositions, TRIPOD_MODE, stepHeight, 2.0))
        {
          state = RUNNING;
          getParameters(n, &params, params.gaitType);
          switch (gait)
          {
            case (TRIPOD_GAIT):
              getParameters(n, &params, "tripod_gait");
              break;
            case(RIPPLE_GAIT):
              getParameters(n, &params, "ripple_gait");
              params.stepFrequency*=0.5;
              break;
            case(WAVE_GAIT):
              getParameters(n, &params, "wave_gait");
              params.stepFrequency*=0.1;
              break;
          }          
          walker = WalkController(&hexapod, params);
          cout << params.gaitType << " Selected." << endl;
        }
        break;
      }
      
      //Robot steps from operational position into a safe position where it can then begin packing procedure
      case(SHUTDOWN):
      {
        if (poser.shutDownSequence(heightRatio, stepHeight, params.moveLegsSequentially))
        {
          state = PACK;
        } 
        break;
      }

      //Robot transitions to packed state
      case(PACK):
      {
        if (poser.moveToJointPosition(packedJointPositions))
        {
          state = PACKED;
          cout << "Shutdown sequence complete." << endl;  
        }
        break;
      }   
      
      //Robot in packed state
      case(PACKED):
      {           
        if (startFlag)
        {
          state = UNPACK;
          cout << "Running startup sequence . . ." << endl;
        }
        break;
      }        
      
      //State unknown due to first iteration of controller
      case(UNKNOWN):
      {
        int checkPacked = 0;
        for (int l=0; l<3; l++)
        {
          for (int s=0; s<2; s++)
          {
            //Check all current joint positions are 'close' to packed joint positions
            double tolerance = 0.1;
            checkPacked += abs(hexapod.legs[l][s].yaw - packedJointPositions[l][s][0]) < tolerance &&
                           abs(hexapod.legs[l][s].liftAngle - packedJointPositions[l][s][1]) < tolerance &&
                           abs(hexapod.legs[l][s].kneeAngle - packedJointPositions[l][s][2]) < tolerance;
          }
        }
        if (checkPacked == 6) //All joints in each leg are approximately in the packed position
        {
          if (!params.startUpSequence)
          {
            cout << "WARNING! Robot currently in packed state and cannot run direct startup sequence." << endl;
            cout << "Either manually unpack robot or set start_up_sequence to true in config file" << endl;
            ASSERT(false);
          }
          else
          {
            state = PACKED;
          }
        }
        else if (!params.startUpSequence)
        {    
          state = DIRECT;
          cout << "WARNING! Running direct startup sequence - assuming hexapod is not on the ground" << endl;
          cout << "Running startup sequence (Complete in " << params.timeToStart << " seconds) . . ." << endl;       
        }
        else if (startFlag)
        {
          state = STARTUP;
          cout << "Running startup sequence . . ." << endl;
        }         
        break;
      }
      
      //No startup/shutdown ability requested - move legs directly to walk position  
      case(DIRECT):
      {
        if (startFlag)
        {
          int mode = params.moveLegsSequentially ? SEQUENTIAL_MODE:NO_STEP_MODE;
          double stepSpeed = 6.0/params.timeToStart;
          
          if (poser.stepToPosition(walker.identityTipPositions, mode, 0, stepSpeed))
          {
            state = RUNNING;
            cout << "Startup sequence complete. \nReady to walk." << endl;
          }
        }
        break;
      }
    } 
    
    //RVIZ
    if (params.debug_rviz)
    {
      for (int s = 0; s<2; s++)
      {
        for (int l = 0; l<3; l++)
        {
          debug.tipPositions.insert(debug.tipPositions.begin(), walker.pose.transformVector(hexapod.legs[l][s].localTipPosition));
          debug.staticTipPositions.insert(debug.staticTipPositions.begin(), hexapod.legs[l][s].localTipPosition);
        }
      }
      debug.drawRobot(hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose), Vector4d(1,1,1,1));    
      
      debug.drawPoints(debug.tipPositions, Vector4d(1,0,0,1)); //Actual Tip Trajectory Paths
      //debug.drawPoints(debug.staticTipPositions, Vector4d(1,0,0,1)); //Static Single Tip Trajectory command
      
      //cout << "Tip Positions Array Size: " << debug.tipPositions.size() << endl;
      //cout << "Static Tip Positions Array Size: " << debug.staticTipPositions.size() << endl;
      
      if (debug.tipPositions.size() > 2000)
        debug.tipPositions.erase(debug.tipPositions.end()-6,debug.tipPositions.end());
      if (debug.staticTipPositions.size() >= 6*(1/params.timeDelta))
        debug.staticTipPositions.clear();
    }
    //RVIZ 
    
    //Publish desired joint angles
    for (int s = 0; s<2; s++)
    {
      double dir = s==0 ? -1 : 1;
      for (int l = 0; l<3; l++)
      {            
        double yaw = dir*(hexapod.legs[l][s].yaw - params.physicalYawOffset[l]);
        double lift = dir*hexapod.legs[l][s].liftAngle;
        double knee = dir*hexapod.legs[l][s].kneeAngle;
        
        if (false) // !firstFrame)
        {
          double yawVel = (yaw - hexapod.legs[l][s].debugOldYaw)/params.timeDelta;
          double liftVel = (lift - hexapod.legs[l][s].debugOldLiftAngle)/params.timeDelta;
          double kneeVel = (knee - hexapod.legs[l][s].debugOldKneeAngle)/params.timeDelta;
          
          if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[0])
          {
            yaw = hexapod.legs[l][s].debugOldYaw + 
            sign(yawVel)*hexapod.jointMaxAngularSpeeds[0]*params.timeDelta;
          }
          if (abs(liftVel) > hexapod.jointMaxAngularSpeeds[1])
          {
            lift = hexapod.legs[l][s].debugOldLiftAngle + 
            sign(liftVel)*hexapod.jointMaxAngularSpeeds[1]*params.timeDelta;
          }
          if (abs(kneeVel) > hexapod.jointMaxAngularSpeeds[2])
          {
            knee = hexapod.legs[l][s].debugOldKneeAngle + 
            sign(kneeVel)*hexapod.jointMaxAngularSpeeds[2]*params.timeDelta;
          }
          if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[0] || 
              abs(liftVel) > hexapod.jointMaxAngularSpeeds[1] || 
              abs(kneeVel) > hexapod.jointMaxAngularSpeeds[2])
          {
            cout << "WARNING: MAXIMUM SPEED EXCEEDED!" << endl;
            cout << "Clamping to maximum angular speed for leg: " << l << " side: " << s << endl;
          }
        }
        
        interface->setTargetAngle(l, s, 0, yaw);
        interface->setTargetAngle(l, s, 1, -lift);
        interface->setTargetAngle(l, s, 2, knee);
        
        hexapod.legs[l][s].debugOldYaw = yaw;
        hexapod.legs[l][s].debugOldLiftAngle = lift;
        hexapod.legs[l][s].debugOldKneeAngle = knee;
      }
    }
    interface->publish();
    
    ros::spinOnce();
    r.sleep();

    debug.reset();
  }
}

/***********************************************************************************************************************
 * This callback increments the gains in the controller
***********************************************************************************************************************/
void imuControllerCallback(const sensor_msgs::Joy &input)
{  
  if(input.axes[7]==1)
  {
    pIncrement += 0.1;
  } 
  if(input.axes[7]==-1)
  {
    pIncrement -= 0.1;
  }    
}

/***********************************************************************************************************************
 * Gait Selection Callback
***********************************************************************************************************************/
void gaitSelectionCallback(const std_msgs::Int8 &input)
{
  switch (input.data)
  {
    case(-1):
      gait = DEFAULT;
      break;
    case(0):
      gait = TRIPOD_GAIT;
      break;
    case(1):
      gait = RIPPLE_GAIT;
      break;
    case(2):
      gait = WAVE_GAIT;
      break;
    default:
      cout << "Unknown gait requested from control input." << endl;
  }
}


/***********************************************************************************************************************
 * Actuating Leg Selection Callback
***********************************************************************************************************************/
void legSelectionCallback(const std_msgs::Int8 &input)
{
  switch (input.data)
  {
    case(-1):
      break;
    case(0):
      if (legSelection != FRONT_LEFT)
      {
        legSelection = FRONT_LEFT;
        cout << "Front left leg selected." << endl;
      }
      break;
    case(1):
      if (legSelection != FRONT_RIGHT)
      {
        legSelection = FRONT_RIGHT;
        cout << "Front right leg selected." << endl;
      }
      break;
    case(2):
      if (legSelection != MIDDLE_LEFT)
      {
        legSelection = MIDDLE_LEFT;
        cout << "Middle left leg selected." << endl;
      }
      break;
    case(3):
      if (legSelection != MIDDLE_RIGHT)
      {
        legSelection = MIDDLE_RIGHT;
        cout << "Middle right leg selected." << endl;
      }
      break;
    case(4):
      if (legSelection != REAR_LEFT)
      {
        legSelection = REAR_LEFT;
        cout << "Rear left leg selected." << endl;
      }
      break;
    case(5):
      if (legSelection != REAR_RIGHT)
      {
        legSelection = REAR_RIGHT;
        cout << "Rear right leg selected." << endl;
      }
      break;
    default:
      cout << "Unknown leg selection requested from control input." << endl;
  }
}

/***********************************************************************************************************************
 * Joypad Velocity Topic Callback
***********************************************************************************************************************/
void joypadVelocityCallback(const geometry_msgs::Twist &twist)
{
  stepFrequencyMultiplier = (-0.5*twist.linear.z+1.5); //Range: 1.0->2.0
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
  
  poseTimeJoy = pParams->maxPoseTime*(0.5*twist.angular.z+0.5); //Range: maxPoseTime->0.0
}

/***********************************************************************************************************************
 * Joypad Pose Topic Callback
***********************************************************************************************************************/
void joypadPoseCallback(const geometry_msgs::Twist &twist)
{
  double deadband = 0.1; //%10 deadband
  
  double newRollJoy = twist.angular.x*pParams->maxRoll;
  double newPitchJoy = twist.angular.y*pParams->maxPitch;
  double newYawJoy = twist.angular.z*pParams->maxYaw;
  double newXJoy = twist.linear.x*pParams->maxX;
  double newYJoy = twist.linear.y*pParams->maxY;
  double newZJoy = twist.linear.z >= 0 ? twist.linear.z*pParams->maxZ : twist.linear.z*pParams->minZ;
  
  //Percentage change is above deadband value
  if (abs(newRollJoy-rollJoy) > deadband*abs(rollJoy))
    rollJoy = newRollJoy;
  if (abs(newPitchJoy-pitchJoy) > deadband*abs(pitchJoy))
    pitchJoy = newPitchJoy;
  if (abs(newYawJoy-yawJoy) > deadband*abs(yawJoy))
    yawJoy = newYawJoy;
  if (abs(newXJoy-xJoy) > deadband*abs(xJoy))
    xJoy = newXJoy;
  if (abs(newYJoy-yJoy) > deadband*abs(yJoy))
    yJoy = newYJoy;
  if (abs(newZJoy-zJoy) > deadband*abs(zJoy))
    zJoy = newZJoy; 
}

/***********************************************************************************************************************
 * Joypad Start State Topic Callback
***********************************************************************************************************************/
void startCallback(const std_msgs::Bool &input)
{
  startFlag = input.data;
}

/***********************************************************************************************************************
 * Toggle Leg State Callback
***********************************************************************************************************************/
void legStateCallback(const std_msgs::Bool &input)
{
  if (input.data && debounce)
  {
    toggleLegState = true;
    debounce = false;
  }
  else if (!input.data && !toggleLegState)
    debounce = true;
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void jointStatesCallback(const sensor_msgs::JointState &joint_States)
{  
  if (!jointPosFlag)
  {
    for (uint i=0; i<joint_States.name.size(); i++)
    {
      const char* jointName = joint_States.name[i].c_str();
      if (!strcmp(jointName, "front_left_body_coxa") ||
          !strcmp(jointName, "AL_coxa_joint"))
        jointPositions[0] = joint_States.position[i];
      else if (!strcmp(jointName, "front_left_coxa_femour") ||
                !strcmp(jointName, "AL_femur_joint"))
        jointPositions[1] = joint_States.position[i];
      else if (!strcmp(jointName, "front_left_femour_tibia") ||
                !strcmp(jointName, "AL_tibia_joint"))
        jointPositions[2] = joint_States.position[i];
      else if (!strcmp(jointName, "front_right_body_coxa") ||
                !strcmp(jointName, "AR_coxa_joint"))
        jointPositions[3] = joint_States.position[i];
      else if (!strcmp(jointName, "front_right_coxa_femour") ||
                !strcmp(jointName, "AR_femur_joint"))
        jointPositions[4] = joint_States.position[i];
      else if (!strcmp(jointName, "front_right_femour_tibia") ||
                !strcmp(jointName, "AR_tibia_joint"))
        jointPositions[5] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_left_body_coxa") ||
                !strcmp(jointName, "BL_coxa_joint"))
        jointPositions[6] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_left_coxa_femour") ||
                !strcmp(jointName, "BL_femur_joint"))
        jointPositions[7] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_left_femour_tibia") ||
                !strcmp(jointName, "BL_tibia_joint"))
        jointPositions[8] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_right_body_coxa") ||
                !strcmp(jointName, "BR_coxa_joint"))
        jointPositions[9] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_right_coxa_femour") ||
                !strcmp(jointName, "BR_femur_joint"))
        jointPositions[10] = joint_States.position[i];
      else if (!strcmp(jointName, "middle_right_femour_tibia") ||
                !strcmp(jointName, "BR_tibia_joint"))
        jointPositions[11] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_left_body_coxa") ||
                !strcmp(jointName, "CL_coxa_joint"))
        jointPositions[12] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_left_coxa_femour") ||
                !strcmp(jointName, "CL_femur_joint"))
        jointPositions[13] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_left_femour_tibia") ||
                !strcmp(jointName, "CL_tibia_joint"))
        jointPositions[14] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_right_body_coxa") ||
                !strcmp(jointName, "CR_coxa_joint"))
        jointPositions[15] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_right_coxa_femour") ||
                !strcmp(jointName, "CR_femur_joint"))
        jointPositions[16] = joint_States.position[i];
      else if (!strcmp(jointName, "rear_right_femour_tibia") ||
                !strcmp(jointName, "CR_tibia_joint"))
        jointPositions[17] = joint_States.position[i];
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
 * Gets hexapod parameters from rosparam server
***********************************************************************************************************************/
void getParameters(ros::NodeHandle n, Parameters *params, std::string forceGait)
{
  std::string baseParamString="/hexapod/parameters/";
  std::string paramString;
  
  // Hexapod Parameters
  if(!n.getParam(baseParamString+"hexapod_type", params->hexapodType))
  {
    cout << "Error reading parameter/s (hexapod_type) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam(baseParamString+"time_delta", params->timeDelta))
  {
    cout << "Error reading parameter/s (time_delta) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
    
  if(!n.getParam(baseParamString+"imu_compensation", params->imuCompensation))
  {
    cout << "Error reading parameter/s (imu_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(baseParamString+"auto_compensation", params->autoCompensation))
  {
    cout << "Error reading parameter/s (auto_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(baseParamString+"manual_compensation", params->manualCompensation))
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
    params->rootOffset[0][0] = Map<Vector3d>(&rootOffsetAL[0], 3);
  }
  
  std::vector<double> rootOffsetAR(3);
  if(!n.getParam(paramString+"root_offset_AR", rootOffsetAR))
  {
    cout << "Error reading parameter/s (root_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[0][1] = Map<Vector3d>(&rootOffsetAR[0], 3);
  }
  
  std::vector<double> rootOffsetBL(3);
  if(!n.getParam(paramString+"root_offset_BL", rootOffsetBL))
  {
    cout << "Error reading parameter/s (root_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[1][0] = Map<Vector3d>(&rootOffsetBL[0], 3);
  }
  
  std::vector<double> rootOffsetBR(3);
  if(!n.getParam(paramString+"root_offset_BR", rootOffsetBR))
  {
    cout << "Error reading parameter/s (root_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[1][1] = Map<Vector3d>(&rootOffsetBR[0], 3);
  }
  
    std::vector<double> rootOffsetCL(3);
  if(!n.getParam(paramString+"root_offset_CL", rootOffsetCL))
  {
    cout << "Error reading parameter/s (root_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[2][0] = Map<Vector3d>(&rootOffsetCL[0], 3);
  }
  
  std::vector<double> rootOffsetCR(3);
  if(!n.getParam(paramString+"root_offset_CR", rootOffsetCR))
  {
    cout << "Error reading parameter/s (root_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[2][1] = Map<Vector3d>(&rootOffsetCR[0], 3);
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
    params->hipOffset[0][0] = Map<Vector3d>(&hipOffsetAL[0], 3);
  }
  
  std::vector<double> hipOffsetAR(3);
  if(!n.getParam(paramString+"hip_offset_AR", hipOffsetAR))
  {
    cout << "Error reading parameter/s (hip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[0][1] = Map<Vector3d>(&hipOffsetAR[0], 3);
  }
  
  std::vector<double> hipOffsetBL(3);
  if(!n.getParam(paramString+"hip_offset_BL", hipOffsetBL))
  {
    cout << "Error reading parameter/s (hip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[1][0] = Map<Vector3d>(&hipOffsetBL[0], 3);
  }
  
  std::vector<double> hipOffsetBR(3);
  if(!n.getParam(paramString+"hip_offset_BR", hipOffsetBR))
  {
    cout << "Error reading parameter/s (hip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[1][1] = Map<Vector3d>(&hipOffsetBR[0], 3);
  }
  
    std::vector<double> hipOffsetCL(3);
  if(!n.getParam(paramString+"hip_offset_CL", hipOffsetCL))
  {
    cout << "Error reading parameter/s (hip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[2][0] = Map<Vector3d>(&hipOffsetCL[0], 3);
  }
  
  std::vector<double> hipOffsetCR(3);
  if(!n.getParam(paramString+"hip_offset_CR", hipOffsetCR))
  {
    cout << "Error reading parameter/s (hip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[2][1] = Map<Vector3d>(&hipOffsetCR[0], 3);
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
    params->kneeOffset[0][0] = Map<Vector3d>(&kneeOffsetAL[0], 3);
  }
  
  std::vector<double> kneeOffsetAR(3);
  if(!n.getParam(paramString+"knee_offset_AR", kneeOffsetAR))
  {
    cout << "Error reading parameter/s (knee_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[0][1] = Map<Vector3d>(&kneeOffsetAR[0], 3);
  }
  
  std::vector<double> kneeOffsetBL(3);
  if(!n.getParam(paramString+"knee_offset_BL", kneeOffsetBL))
  {
    cout << "Error reading parameter/s (knee_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[1][0] = Map<Vector3d>(&kneeOffsetBL[0], 3);
  }
  
  std::vector<double> kneeOffsetBR(3);
  if(!n.getParam(paramString+"knee_offset_BR", kneeOffsetBR))
  {
    cout << "Error reading parameter/s (knee_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[1][1] = Map<Vector3d>(&kneeOffsetBR[0], 3);
  }
  
    std::vector<double> kneeOffsetCL(3);
  if(!n.getParam(paramString+"knee_offset_CL", kneeOffsetCL))
  {
    cout << "Error reading parameter/s (knee_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[2][0] = Map<Vector3d>(&kneeOffsetCL[0], 3);
  }
  
  std::vector<double> kneeOffsetCR(3);
  if(!n.getParam(paramString+"knee_offset_CR", kneeOffsetCR))
  {
    cout << "Error reading parameter/s (knee_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[2][1] = Map<Vector3d>(&kneeOffsetCR[0], 3);
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
    params->tipOffset[0][0] = Map<Vector3d>(&tipOffsetAL[0], 3);
  }
  
  std::vector<double> tipOffsetAR(3);
  if(!n.getParam(paramString+"tip_offset_AR", tipOffsetAR))
  {
    cout << "Error reading parameter/s (tip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[0][1] = Map<Vector3d>(&tipOffsetAR[0], 3);
  }
  
  std::vector<double> tipOffsetBL(3);
  if(!n.getParam(paramString+"tip_offset_BL", tipOffsetBL))
  {
    cout << "Error reading parameter/s (tip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[1][0] = Map<Vector3d>(&tipOffsetBL[0], 3);
  }
  
  std::vector<double> tipOffsetBR(3);
  if(!n.getParam(paramString+"tip_offset_BR", tipOffsetBR))
  {
    cout << "Error reading parameter/s (tip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[1][1] = Map<Vector3d>(&tipOffsetBR[0], 3);
  }
  
    std::vector<double> tipOffsetCL(3);
  if(!n.getParam(paramString+"tip_offset_CL", tipOffsetCL))
  {
    cout << "Error reading parameter/s (tip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[2][0] = Map<Vector3d>(&tipOffsetCL[0], 3);
  }
  
  std::vector<double> tipOffsetCR(3);
  if(!n.getParam(paramString+"tip_offset_CR", tipOffsetCR))
  {
    cout << "Error reading parameter/s (tip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[2][1] = Map<Vector3d>(&tipOffsetCR[0], 3);
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
    params->stanceLegYaws = Map<Vector3d>(&stanceLegYaws[0], 3);
  }
  
  std::vector<double> physicalYawOffset(3);
  if(!n.getParam(baseParamString+"physical_yaw_offset", physicalYawOffset))
  {
    cout << "Error reading parameter/s (stance_leg_yaws) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->physicalYawOffset = Map<Vector3d>(&physicalYawOffset[0], 3);
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
    params->yawLimits = Map<Vector3d>(&yawLimits[0], 3);
  }
    
  std::vector<double> kneeLimits(2);
  if(!n.getParam(paramString+"knee_limits", kneeLimits))
  {
    cout << "Error reading parameter/s (knee_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeLimits = Map<Vector2d>(&kneeLimits[0], 2);
  }
  
  std::vector<double> hipLimits(2);    
  if(!n.getParam(paramString+"hip_limits", hipLimits))
  {
    cout << "Error reading parameter/s (hip_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipLimits = Map<Vector2d>(&hipLimits[0], 2);
  }
  
  std::vector<double> jointMaxAngularSpeeds(2); 
  if(!n.getParam(baseParamString+"joint_max_angular_speeds", jointMaxAngularSpeeds))
  {
    cout << "Error reading parameter/s (joint_max_angular_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->jointMaxAngularSpeeds = Map<Vector3d>(&jointMaxAngularSpeeds[0], 3);
  }
  
  if(!n.getParam(baseParamString+"dynamixel_interface", params->dynamixelInterface))
  {
    cout << "Error reading parameter/s (dynamixel_interface) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /********************************************************************************************************************/
  // Walk Controller Parameters
  paramString = baseParamString+"/walk_controller/";
  
  if (!n.getParam(paramString+"step_frequency", params->stepFrequency))
  {
    cout << "Error reading parameter/s (step_frequency) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"step_clearance", params->stepClearance))
  {
    cout << "Error reading parameter/s (step_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"body_clearance", params->bodyClearance))
  {
    cout << "Error reading parameter/s (body_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"leg_span_scale", params->legSpanScale))
  {
    cout << "Error reading parameter/s (leg_span_scale) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam(paramString+"leg_state_correction", params->legStateCorrection))
  {
    cout << "Error reading parameter/s (leg_state_correction) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if (!n.getParam(paramString+"max_acceleration", params->maxAcceleration))
  {
    cout << "Error reading parameter/s (max_acceleration) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_curvature_speed", params->maxCurvatureSpeed))
  {
    cout << "Error reading parameter/s (max_curvature_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"step_curvature_allowance", params->stepCurvatureAllowance))
  {
    cout << "Error reading parameter/s (step_curvature_allowance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"interface_setup_speed", params->interfaceSetupSpeed))
  {
    cout << "Error reading parameter/s (interface_setup_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /********************************************************************************************************************/
  // Pose Controller Parameters 
  paramString = baseParamString+"/pose_controller/";
  
  if(!n.getParam(paramString+"start_up_sequence", params->startUpSequence))
  {
    cout << "Error reading parameter/s (start_up_sequence) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"move_legs_sequentially", params->moveLegsSequentially))
  {
    cout << "Error reading parameter/s (move_legs_sequentially) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"time_to_start", params->timeToStart))
  {
    cout << "Error reading parameter/s (time_to_start) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  paramString = baseParamString+"/pose_controller/auto_pose_compensation/";
  if(!n.getParam(paramString+"pitch_amplitude", params->pitchAmplitude))
  {
    cout << "Error reading parameter/s (pitch_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam(paramString+"roll_amplitude", params->rollAmplitude))
  {
    cout << "Error reading parameter/s (roll_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  paramString = baseParamString+"/pose_controller/manual_pose_compensation/";
  if (!n.getParam(paramString+"max_pose_time", params->maxPoseTime))
  {
    cout << "Error reading parameter/s (max_pose_time) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_roll", params->maxRoll))
  {
    cout << "Error reading parameter/s (max_roll) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_pitch", params->maxPitch))
  {
    cout << "Error reading parameter/s (max_pitch) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_yaw", params->maxYaw))
  {
    cout << "Error reading parameter/s (max_yaw) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_x", params->maxX))
  {
    cout << "Error reading parameter/s (max_x) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_y", params->maxY))
  {
    cout << "Error reading parameter/s (max_y) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam(paramString+"max_z", params->maxZ))
  {
    cout << "Error reading parameter/s (max_z) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  } 
  
  if (!n.getParam(paramString+"min_z", params->minZ))
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
    params->packedJointPositionsAL = Map<Vector3d>(&packedJointPositionsAL[0], 3);
  }
    
  std::vector<double> packedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_packed_joint_positions", packedJointPositionsAR))
  {
    cout << "Error reading parameter/s (AR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->packedJointPositionsAR = Map<Vector3d>(&packedJointPositionsAR[0], 3);
  }
  
  std::vector<double> packedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_packed_joint_positions", packedJointPositionsBL))
  {
    cout << "Error reading parameter/s (BL_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->packedJointPositionsBL = Map<Vector3d>(&packedJointPositionsBL[0], 3);
  }
  
  std::vector<double> packedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_packed_joint_positions", packedJointPositionsBR))
  {
    cout << "Error reading parameter/s (BR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->packedJointPositionsBR = Map<Vector3d>(&packedJointPositionsBR[0], 3);
  }
  
  std::vector<double> packedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_packed_joint_positions", packedJointPositionsCL))
  {
    cout << "Error reading parameter/s (CL_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->packedJointPositionsCL = Map<Vector3d>(&packedJointPositionsCL[0], 3);
  }
  
  std::vector<double> packedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_packed_joint_positions", packedJointPositionsCR))
  {
    cout << "Error reading parameter/s (CR_packed_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->packedJointPositionsCR = Map<Vector3d>(&packedJointPositionsCR[0], 3);
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
    params->unpackedJointPositionsAL = Map<Vector3d>(&unpackedJointPositionsAL[0], 3);
  }
    
  std::vector<double> unpackedJointPositionsAR(3);
  if (!n.getParam(paramString+"AR_unpacked_joint_positions", unpackedJointPositionsAR))
  {
    cout << "Error reading parameter/s (AR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->unpackedJointPositionsAR = Map<Vector3d>(&unpackedJointPositionsAR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBL(3);
  if (!n.getParam(paramString+"BL_unpacked_joint_positions", unpackedJointPositionsBL))
  {
    cout << "Error reading parameter/s (BL_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->unpackedJointPositionsBL = Map<Vector3d>(&unpackedJointPositionsBL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsBR(3);
  if (!n.getParam(paramString+"BR_unpacked_joint_positions", unpackedJointPositionsBR))
  {
    cout << "Error reading parameter/s (BR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->unpackedJointPositionsBR = Map<Vector3d>(&unpackedJointPositionsBR[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCL(3);
  if (!n.getParam(paramString+"CL_unpacked_joint_positions", unpackedJointPositionsCL))
  {
    cout << "Error reading parameter/s (CL_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->unpackedJointPositionsCL = Map<Vector3d>(&unpackedJointPositionsCL[0], 3);
  }
  
  std::vector<double> unpackedJointPositionsCR(3);
  if (!n.getParam(paramString+"CR_unpacked_joint_positions", unpackedJointPositionsCR))
  {
    cout << "Error reading parameter/s (CR_unpacked_joint_positions) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->unpackedJointPositionsCR = Map<Vector3d>(&unpackedJointPositionsCR[0], 3);
  }
  
  /********************************************************************************************************************/
  // Gait Parameters 
  if (forceGait.empty())
  {
    if (!n.getParam(baseParamString+"gait_type", params->gaitType))
    {
      cout << "Error reading parameter/s (gaitType) from rosparam" <<endl; 
      cout << "Check config file is loaded and type is correct" << endl;
    }
  }
  else
  {
    params->gaitType = forceGait;
  } 

  paramString = params->gaitType+"_parameters/stance_phase";
  if (!n.getParam(paramString, params->stancePhase))
  {
    cout << "Error reading parameter/s (stance_phase) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = params->gaitType+"_parameters/swing_phase";
  if (!n.getParam(paramString, params->swingPhase))
  {
    cout << "Error reading parameter/s (swing_phase) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = params->gaitType+"_parameters/phase_offset";
  if (!n.getParam(paramString, params->phaseOffset))
  {
    cout << "Error reading parameter/s (phase_offset) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = params->gaitType+"_parameters/leg_selection_pattern";
  if (!n.getParam(paramString, params->legSelectionPattern))
  {
    cout << "Error reading parameter/s (leg_selection_pattern) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = params->gaitType+"_parameters/side_selection_pattern";
  if (!n.getParam(paramString, params->sideSelectionPattern))
  {
    cout << "Error reading parameter/s (side_selection_pattern) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  paramString = params->gaitType+"_parameters/transition_period";
  if (!n.getParam(paramString, params->transitionPeriod))
  {
    cout << "Error reading parameter/s (transition_period) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /********************************************************************************************************************/
  //Debug Parameters (set via launch file instead of by config files)
  
  paramString = "/hexapod/debug_parameters/rviz";
  if (!n.getParam(paramString, params->debug_rviz))
  {
    params->debug_rviz = false;
  }
}

/***********************************************************************************************************************
***********************************************************************************************************************/


