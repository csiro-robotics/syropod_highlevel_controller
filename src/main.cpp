/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/controller.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include "../include/simple_hexapod_controller/imuCompensation.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include "std_msgs/Bool.h"
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/Joy.h"

//Globals for joypad callback
static Vector2d localVelocity(0,0);
static double turnRate = 0;
static double pitchJoy = 0;
static double rollJoy = 0;
static double yawJoy = 0;
static double xJoy = 0;
static double yJoy = 0;
static double zJoy = 0;
double pIncrement=0;

//Globals for joint states callback
sensor_msgs::JointState jointStates;
double jointPositions[18];
bool jointPosFlag = false;
bool startFlag = false;


void joypadVelocityCallback(const geometry_msgs::Twist &twist);
void joypadPoseCallback(const geometry_msgs::Twist &twist);
void imuControllerIncrement(const sensor_msgs::Joy &bButton);

void startCallback(const std_msgs::Bool &startBool);

double getRollCompensation(WalkController walker);
double getPitchCompensation(WalkController walker);

void jointStatesCallback(const sensor_msgs::JointState &joint_States);
void getParameters(ros::NodeHandle n, Parameters *params);

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  
  
  ros::Publisher controlPub = n.advertise<geometry_msgs::Vector3>("controlsignal", 1000);
  
  ros::Subscriber velocitySubscriber = n.subscribe("/desired_velocity", 1, joypadVelocityCallback);
  ros::Subscriber poseSubscriber = n.subscribe("/desired_pose", 1, joypadPoseCallback);
  ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data", 1, imuCallback);
  ros::Subscriber bButton = n.subscribe("/joy", 1, imuControllerIncrement);
    
  ros::Subscriber startSubscriber = n.subscribe("/start_state", 1, startCallback);
  ros::Subscriber jointStatesSubscriber;  
    
  //Get parameters from rosparam via loaded config file
  Parameters params;
  getParameters(n, &params);
  
  ros::Rate r(roundToInt(1.0/params.timeDelta));         //frequency of the loop. 
  
  //Start User Message
  cout << "Press 'Start' to run controller" << endl;
  
  //Loop waiting for start button press
  while(!startFlag)
  {
    ros::spinOnce();
    r.sleep();
  }  
  
  //MOVE_TO_START
  if (params.moveToStart)
  {
    jointStatesSubscriber = n.subscribe("/hexapod/joint_states", 1, jointStatesCallback);
    
    if(!jointStatesSubscriber)
    {
      cout << "Failed to subscribe to joint_states topic - check to see if topic is being published." << endl;
      params.moveToStart = false;
    }
    else
    {
      for (int i=0; i<18; i++)
        jointPositions[i] = 1e10;
    
      int spin = 1/params.timeDelta; //Max ros spin cycles to find joint positions
      while(spin--)
      {
        ros::spinOnce();
        r.sleep();
      }
    }
  }
  
  //Create hexapod model    
  Model hexapod(params);
  
  hexapod.jointMaxAngularSpeeds = params.jointMaxAngularSpeeds;
  
  //MOVE_TO_START
  if (params.moveToStart)
  {
    if (jointPosFlag)
    {
      // set initial leg angles
      for (int leg = 0; leg<3; leg++)
      {
        for (int side = 0; side<2; side++)
        {
          double dir = side==0 ? -1 : 1;
          int index = leg*6+(side == 0 ? 0 : 3);
          hexapod.setLegStartAngles(side, leg, dir*Vector3d(jointPositions[index+0]+dir*params.stanceLegYaws[leg],
                                                            -jointPositions[index+1], jointPositions[index+2]));          
        }
      }
    }
    else
    {
      cout << "Failed to acquire all joint position values" << endl;
      params.moveToStart = false;
    }
  }
  
  // Create walk controller object
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
  
  if (params.moveToStart)
    cout << "Attempting to move to starting stance . . ." << endl;
  
    
  //Position update loop
  bool firstFrame = true;
  bool started = false;
  while (ros::ok())
  {
    Pose adjust = Pose::identity();
    
    Vector3d *deltaAngle = NULL;
    Vector3d *deltaPos = NULL;
    
    if (params.imuCompensation)
    {
      //Auto Compensation using IMU feedback
      Vector2d acc = walker.localCentreAcceleration;

      compensation(Vector3d(acc[0], acc[1], 0), walker.angularVelocity, deltaAngle, deltaPos,pIncrement, params.timeDelta);
      //geometry_msgs::Vector3 controlMeanAcc;
      //controlMeanAcc.x = (*deltaPos)[0];
      //controlMeanAcc.y = (*deltaPos)[1];
      //controlMeanAcc.z = (*deltaPos)[2];
      //controlPub.publish(controlMeanAcc); 
    }
    else if (params.autoCompensation)
    {
      //Automatic (non-feedback) compensation
      double pitch = getPitchCompensation(walker);
      double roll = getRollCompensation(walker);  
      if (params.gaitType == "wave_gait")
        adjust = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitch,roll,yawJoy));
      else if (params.gaitType == "tripod_gait")
        adjust = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,0,roll,yawJoy));
    }    
    else if (params.manualCompensation)
    {    
      //Manual body compensation 
      adjust = Pose(Vector3d(xJoy,yJoy,zJoy), Quat(1,pitchJoy,rollJoy,yawJoy));
    }   
    
    //Manual velocity control
    //localVelocity = Vector2d(1e-10, 1e-10);
    
    //Starting tip positions and pose for walking
    Pose startPose = Pose(Vector3d(0,0,0), Quat(1,0,0,0)); 
    //Pose startPose = Pose::identity();
    Vector3d startTipPositions[3][2];
    for (int l = 0; l<3; l++)
      for (int s = 0; s<2; s++) 
        startTipPositions[l][s] = walker.legSteppers[l][s].defaultTipPosition;
        //startTipPositions[l][s] = hexapod.legs[l][s].identityTipPosition;
    
    //Update walker or move to starting stance
    if (!started && params.moveToStart)
      started = poser.moveToPose(startTipPositions, startPose, params.timeToStart, params.moveLegsSequentially);
    else
    {
      walker.update(localVelocity, turnRate*turnRate*turnRate, &adjust, deltaPos, deltaAngle); // the cube just lets the thumbstick give small turns easier
      Vector3d tipPositions[3][2];
      for (int l = 0; l<3; l++)
        for (int s = 0; s<2; s++) 
          tipPositions[l][s] = hexapod.legs[l][s].localTipPosition;
      poser.moveToPose(tipPositions, adjust, 5.0);
    }
    
    debug.drawRobot(hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose * adjust), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));
    
    if (true)
    {
      for (int s = 0; s<2; s++)
      {
        double dir = s==0 ? -1 : 1;
        for (int l = 0; l<3; l++)
        {
          double angle;  
          double yaw = dir*(walker.model->legs[l][s].yaw - params.stanceLegYaws[l]);
          double lift = dir*walker.model->legs[l][s].liftAngle;
          double knee = dir*walker.model->legs[l][s].kneeAngle;
                   
          if (false) // !firstFrame)
          {
            double yawVel = (yaw - walker.model->legs[l][s].debugOldYaw)/params.timeDelta;
            double liftVel = (lift - walker.model->legs[l][s].debugOldLiftAngle)/params.timeDelta;
            double kneeVel = (knee - walker.model->legs[l][s].debugOldKneeAngle)/params.timeDelta;
            
            if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[0])
              yaw = walker.model->legs[l][s].debugOldYaw + sign(yawVel)*hexapod.jointMaxAngularSpeeds[0]*params.timeDelta;
            if (abs(liftVel) > hexapod.jointMaxAngularSpeeds[1])
              lift = walker.model->legs[l][s].debugOldLiftAngle + sign(liftVel)*hexapod.jointMaxAngularSpeeds[1]*params.timeDelta;
            if (abs(kneeVel) > hexapod.jointMaxAngularSpeeds[2])
              knee = walker.model->legs[l][s].debugOldKneeAngle + sign(kneeVel)*hexapod.jointMaxAngularSpeeds[2]*params.timeDelta;
              cout << "WARNING: MAXIMUM SPEED EXCEEDED! Clamping to maximum angular speed for leg " << l << " side " << s << endl;
          }
          
          interface->setTargetAngle(l, s, 0, yaw);
          interface->setTargetAngle(l, s, 1, -lift);
          interface->setTargetAngle(l, s, 2, knee);
          
          walker.model->legs[l][s].debugOldYaw = yaw;
          walker.model->legs[l][s].debugOldLiftAngle = lift;
          walker.model->legs[l][s].debugOldKneeAngle = knee;
        }
      }
      interface->publish();
    }
    
    firstFrame = false;
    ros::spinOnce();
    r.sleep();

    debug.reset();
  }
}

/***********************************************************************************************************************
This callback increments the gains in the controller
***********************************************************************************************************************/
void imuControllerIncrement(const sensor_msgs::Joy &gainAdjust)
{  
  if(gainAdjust.axes[7]==1)
  {
    pIncrement += 0.1;
  } 
  if(gainAdjust.axes[7]==-1)
  {
    pIncrement -= 0.1;
  }    
}

/***********************************************************************************************************************
 * Joypad Velocity Topic Callback
***********************************************************************************************************************/
void joypadVelocityCallback(const geometry_msgs::Twist &twist)
{
  localVelocity = Vector2d(twist.linear.x, twist.linear.y);
  localVelocity = clamped(localVelocity, 1.0);
  turnRate = twist.angular.x; //RS ROTATION CONTROL SCHEME
  //turnRate = (twist.linear.z-twist.angular.z)/2; //TRIGGER ROTATION CONTROL SCHEME
}

/***********************************************************************************************************************
 * Joypad Pose Topic Callback
***********************************************************************************************************************/
void joypadPoseCallback(const geometry_msgs::Twist &twist)
{
   //ADJUSTED FOR SENSITIVITY OF JOYSTICK
  rollJoy = twist.angular.x*0.075;
  pitchJoy = twist.angular.y*-0.075;
  yawJoy = twist.angular.z*0.2;  
  xJoy = twist.linear.x*0.05;
  yJoy = twist.linear.y*0.05; 
  zJoy = twist.linear.z*0.05; 
}

/***********************************************************************************************************************
 * Joypad Start State Topic Callback
***********************************************************************************************************************/
void startCallback(const std_msgs::Bool &startBool)
{
  if (startBool.data == true)
    startFlag = true;
  else
    startFlag = false;
}

/***********************************************************************************************************************
 * Calculates pitch for body compensation
***********************************************************************************************************************/
double getPitchCompensation(WalkController walker)
{
  double pitch;
  double amplitude = walker.params.pitchAmplitude;
  double phase = walker.legSteppers[0][0].phase;
  double phaseLength = walker.params.stancePhase + walker.params.swingPhase;
  double buffer = walker.params.phaseOffset/2;
  double phaseOffset = walker.params.phaseOffset;
  double p0[2] = {0*phaseOffset, -amplitude};
  double p1[2] = {1*phaseOffset + buffer, -amplitude};
  double p2[2] = {2*phaseOffset + buffer, amplitude};
  double p3[2] = {4*phaseOffset + buffer, amplitude};
  double p4[2] = {5*phaseOffset + buffer, -amplitude};
  double p5[2] = {6*phaseOffset, -amplitude};
    
  if (phase >= p0[0] && phase < p1[0])
    pitch = p0[1];
  else if (phase >= p1[0] && phase < p2[0])
  {
    double gradient = (p2[1]-p1[1])/(p2[0]-p1[0]);
    double offset = ((p2[0]-p1[0])/2 + p1[0]);
    pitch = gradient*phase - gradient*offset;   //-2*phase/3 + 4;
  }
  else if (phase >= p2[0] && phase < p3[0])
    pitch = p2[1];
  else if (phase >= p3[0] && phase < p4[0])
  {
    double gradient = (p4[1]-p3[1])/(p4[0]-p3[0]);
    double offset = ((p4[0]-p3[0])/2 + p3[0]);
    pitch = gradient*phase - gradient*offset;   //2*phase/3 - 10;
  }
  else if (phase >= p4[0] && phase < p5[0])
    pitch = p4[1];    
  
  return pitch;    
}

/***********************************************************************************************************************
 * Calculates roll for body compensation
***********************************************************************************************************************/
double getRollCompensation(WalkController walker)
{ 
  double roll;
  double amplitude = walker.params.rollAmplitude;
  double phase = walker.legSteppers[0][0].phase;
  double phaseLength = walker.params.stancePhase + walker.params.swingPhase;
  double buffer = walker.params.swingPhase/2.25;
  double phaseOffset = walker.params.phaseOffset;
  double p0[2] = {0, -amplitude};           
  double p1[2] = {0, -amplitude}; 
  double p2[2] = {0, amplitude};
  double p3[2] = {0, amplitude};
  double p4[2] = {0, -amplitude};
  double p5[2] = {0, -amplitude};
  
  if (walker.params.gaitType == "tripod_gait")
  {
    p0[0] = 0*phaseOffset;           
    p1[0] = 0*phaseOffset + buffer; 
    p2[0] = 1*phaseOffset - buffer;
    p3[0] = 1*phaseOffset + buffer;
    p4[0] = 2*phaseOffset - buffer;
    p5[0] = 2*phaseOffset;
  }
  else if (walker.params.gaitType == "wave_gait")
  {
    p0[0] = 0*phaseOffset;           
    p1[0] = 0*phaseOffset + buffer; 
    p2[0] = 1*phaseOffset - buffer;
    p3[0] = 3*phaseOffset + buffer;
    p4[0] = 4*phaseOffset - buffer;
    p5[0] = 6*phaseOffset;
  }
  else
    return 0.0;
    
  if (phase >= p0[0] && phase < p1[0])
    roll = p0[1];
  else if (phase >= p1[0] && phase < p2[0])
  {
    double gradient = (p2[1]-p1[1])/(p2[0]-p1[0]);
    double offset = ((p2[0]-p1[0])/2 + p1[0]);
    roll = gradient*phase - gradient*offset; //-2*phase + 3;
  }     
  else if (phase >= p2[0] && phase < p3[0])
    roll = p2[1];
  else if (phase >= p3[0] && phase < p4[0])
  {
    double gradient = (p4[1]-p3[1])/(p4[0]-p3[0]);
    double offset = ((p4[0]-p3[0])/2 + p3[0]);
    roll = gradient*phase - gradient*offset; //2*phase - 21;      
  }
  else if (phase >= p4[0] && phase < p5[0])
    roll = p4[1];
  
  return roll;
}

/***********************************************************************************************************************
 * Gets hexapod parameters from rosparam server
***********************************************************************************************************************/
void getParameters(ros::NodeHandle n, Parameters *params)
{
  std::string paramString;
  
  // Hexapod Parameters
  if(!n.getParam("hexapod_type", params->hexapodType))
  {
    cout << "Error reading parameter/s (hexapod_type) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam("time_delta", params->timeDelta))
  {
    cout << "Error reading parameter/s (time_delta) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if(!n.getParam("move_to_start", params->moveToStart))
  {
    cout << "Error reading parameter/s (move_to_start) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("move_legs_sequentially", params->moveLegsSequentially))
  {
    cout << "Error reading parameter/s (move_legs_sequentially) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("time_to_start", params->timeToStart))
  {
    cout << "Error reading parameter/s (time_to_start) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("imu_compensation", params->imuCompensation))
  {
    cout << "Error reading parameter/s (imu_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("auto_compensation", params->autoCompensation))
  {
    cout << "Error reading parameter/s (auto_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("pitch_amplitude", params->pitchAmplitude))
  {
    cout << "Error reading parameter/s (pitch_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("roll_amplitude", params->rollAmplitude))
  {
    cout << "Error reading parameter/s (roll_amplitude) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  if(!n.getParam("manual_compensation", params->manualCompensation))
  {
    cout << "Error reading parameter/s (manual_compensation) from rosparam" << endl;
    cout << "Check config file is loaded and type is correct" << endl;  
  }
  
  /**********************************************************************************************************************/
  //Offset Parameters
  //Root Offset Parameters
  std::vector<double> rootOffsetAL(3);
  if(!n.getParam("root_offset_AL", rootOffsetAL))
  {
    cout << "Error reading parameter/s (root_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[0][0] = Map<Vector3d>(&rootOffsetAL[0], 3);
  }
  
  std::vector<double> rootOffsetAR(3);
  if(!n.getParam("root_offset_AR", rootOffsetAR))
  {
    cout << "Error reading parameter/s (root_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[0][1] = Map<Vector3d>(&rootOffsetAR[0], 3);
  }
  
  std::vector<double> rootOffsetBL(3);
  if(!n.getParam("root_offset_BL", rootOffsetBL))
  {
    cout << "Error reading parameter/s (root_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[1][0] = Map<Vector3d>(&rootOffsetBL[0], 3);
  }
  
  std::vector<double> rootOffsetBR(3);
  if(!n.getParam("root_offset_BR", rootOffsetBR))
  {
    cout << "Error reading parameter/s (root_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[1][1] = Map<Vector3d>(&rootOffsetBR[0], 3);
  }
  
    std::vector<double> rootOffsetCL(3);
  if(!n.getParam("root_offset_CL", rootOffsetCL))
  {
    cout << "Error reading parameter/s (root_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[2][0] = Map<Vector3d>(&rootOffsetCL[0], 3);
  }
  
  std::vector<double> rootOffsetCR(3);
  if(!n.getParam("root_offset_CR", rootOffsetCR))
  {
    cout << "Error reading parameter/s (root_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->rootOffset[2][1] = Map<Vector3d>(&rootOffsetCR[0], 3);
  }
  
  /**********************************************************************************************************************/
  //Hip Offset Parameters
  std::vector<double> hipOffsetAL(3);
  if(!n.getParam("hip_offset_AL", hipOffsetAL))
  {
    cout << "Error reading parameter/s (hip_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[0][0] = Map<Vector3d>(&hipOffsetAL[0], 3);
  }
  
  std::vector<double> hipOffsetAR(3);
  if(!n.getParam("hip_offset_AR", hipOffsetAR))
  {
    cout << "Error reading parameter/s (hip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[0][1] = Map<Vector3d>(&hipOffsetAR[0], 3);
  }
  
  std::vector<double> hipOffsetBL(3);
  if(!n.getParam("hip_offset_BL", hipOffsetBL))
  {
    cout << "Error reading parameter/s (hip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[1][0] = Map<Vector3d>(&hipOffsetBL[0], 3);
  }
  
  std::vector<double> hipOffsetBR(3);
  if(!n.getParam("hip_offset_BR", hipOffsetBR))
  {
    cout << "Error reading parameter/s (hip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[1][1] = Map<Vector3d>(&hipOffsetBR[0], 3);
  }
  
    std::vector<double> hipOffsetCL(3);
  if(!n.getParam("hip_offset_CL", hipOffsetCL))
  {
    cout << "Error reading parameter/s (hip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[2][0] = Map<Vector3d>(&hipOffsetCL[0], 3);
  }
  
  std::vector<double> hipOffsetCR(3);
  if(!n.getParam("hip_offset_CR", hipOffsetCR))
  {
    cout << "Error reading parameter/s (hip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipOffset[2][1] = Map<Vector3d>(&hipOffsetCR[0], 3);
  }
  
  /**********************************************************************************************************************/
  //Knee Offset Parameters
  std::vector<double> kneeOffsetAL(3);
  if(!n.getParam("knee_offset_AL", kneeOffsetAL))
  {
    cout << "Error reading parameter/s (knee_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[0][0] = Map<Vector3d>(&kneeOffsetAL[0], 3);
  }
  
  std::vector<double> kneeOffsetAR(3);
  if(!n.getParam("knee_offset_AR", kneeOffsetAR))
  {
    cout << "Error reading parameter/s (knee_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[0][1] = Map<Vector3d>(&kneeOffsetAR[0], 3);
  }
  
  std::vector<double> kneeOffsetBL(3);
  if(!n.getParam("knee_offset_BL", kneeOffsetBL))
  {
    cout << "Error reading parameter/s (knee_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[1][0] = Map<Vector3d>(&kneeOffsetBL[0], 3);
  }
  
  std::vector<double> kneeOffsetBR(3);
  if(!n.getParam("knee_offset_BR", kneeOffsetBR))
  {
    cout << "Error reading parameter/s (knee_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[1][1] = Map<Vector3d>(&kneeOffsetBR[0], 3);
  }
  
    std::vector<double> kneeOffsetCL(3);
  if(!n.getParam("knee_offset_CL", kneeOffsetCL))
  {
    cout << "Error reading parameter/s (knee_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[2][0] = Map<Vector3d>(&kneeOffsetCL[0], 3);
  }
  
  std::vector<double> kneeOffsetCR(3);
  if(!n.getParam("knee_offset_CR", kneeOffsetCR))
  {
    cout << "Error reading parameter/s (knee_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeOffset[2][1] = Map<Vector3d>(&kneeOffsetCR[0], 3);
  }
  
  /**********************************************************************************************************************/
  //Tip Offset Parameters
  std::vector<double> tipOffsetAL(3);
  if(!n.getParam("tip_offset_AL", tipOffsetAL))
  {
    cout << "Error reading parameter/s (tip_offset_AL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[0][0] = Map<Vector3d>(&tipOffsetAL[0], 3);
  }
  
  std::vector<double> tipOffsetAR(3);
  if(!n.getParam("tip_offset_AR", tipOffsetAR))
  {
    cout << "Error reading parameter/s (tip_offset_AR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[0][1] = Map<Vector3d>(&tipOffsetAR[0], 3);
  }
  
  std::vector<double> tipOffsetBL(3);
  if(!n.getParam("tip_offset_BL", tipOffsetBL))
  {
    cout << "Error reading parameter/s (tip_offset_BL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[1][0] = Map<Vector3d>(&tipOffsetBL[0], 3);
  }
  
  std::vector<double> tipOffsetBR(3);
  if(!n.getParam("tip_offset_BR", tipOffsetBR))
  {
    cout << "Error reading parameter/s (tip_offset_BR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[1][1] = Map<Vector3d>(&tipOffsetBR[0], 3);
  }
  
    std::vector<double> tipOffsetCL(3);
  if(!n.getParam("tip_offset_CL", tipOffsetCL))
  {
    cout << "Error reading parameter/s (tip_offset_CL) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[2][0] = Map<Vector3d>(&tipOffsetCL[0], 3);
  }
  
  std::vector<double> tipOffsetCR(3);
  if(!n.getParam("tip_offset_CR", tipOffsetCR))
  {
    cout << "Error reading parameter/s (tip_offset_CR) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->tipOffset[2][1] = Map<Vector3d>(&tipOffsetCR[0], 3);
  }
  
  
  std::vector<double> stanceLegYaws(3);
  if(!n.getParam("stance_leg_yaws", stanceLegYaws))
  {
    cout << "Error reading parameter/s (stance_leg_yaws) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->stanceLegYaws = Map<Vector3d>(&stanceLegYaws[0], 3);
  }
  /**********************************************************************************************************************/
  //Joint Limit Parameters  
  std::vector<double> yawLimits(3);
  if(!n.getParam("yaw_limits", yawLimits))
  {
    cout << "Error reading parameter/s (yaw_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->yawLimits = Map<Vector3d>(&yawLimits[0], 3);
  }
    
  std::vector<double> kneeLimits(2);
  if(!n.getParam("knee_limits", kneeLimits))
  {
    cout << "Error reading parameter/s (knee_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->kneeLimits = Map<Vector2d>(&kneeLimits[0], 2);
  }
  
  std::vector<double> hipLimits(2);    
  if(!n.getParam("hip_limits", hipLimits))
  {
    cout << "Error reading parameter/s (hip_limits) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->hipLimits = Map<Vector2d>(&hipLimits[0], 2);
  }
  
  std::vector<double> jointMaxAngularSpeeds(2); 
  if(!n.getParam("joint_max_angular_speeds", jointMaxAngularSpeeds))
  {
    cout << "Error reading parameter/s (joint_max_angular_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  else
  {
    params->jointMaxAngularSpeeds = Map<Vector3d>(&jointMaxAngularSpeeds[0], 3);
  }
  
  if(!n.getParam("dynamixel_interface", params->dynamixelInterface))
  {
    cout << "Error reading parameter/s (dynamixel_interface) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /**********************************************************************************************************************/
  // Walk Controller Parameters
  if (!n.getParam("step_frequency", params->stepFrequency))
  {
    cout << "Error reading parameter/s (step_frequency) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("step_clearance", params->stepClearance))
  {
    cout << "Error reading parameter/s (step_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("body_clearance", params->bodyClearance))
  {
    cout << "Error reading parameter/s (body_clearance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("leg_span_scale", params->legSpanScale))
  {
    cout << "Error reading parameter/s (leg_span_scale) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("max_acceleration", params->maxAcceleration))
  {
    cout << "Error reading parameter/s (max_acceleration) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("max_curvature_speed", params->maxCurvatureSpeed))
  {
    cout << "Error reading parameter/s (max_curvature_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("step_curvature_allowance", params->stepCurvatureAllowance))
  {
    cout << "Error reading parameter/s (step_curvature_allowance) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  if (!n.getParam("interface_setup_speed", params->interfaceSetupSpeed))
  {
    cout << "Error reading parameter/s (interface_setup_speed) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
  }
  
  /**********************************************************************************************************************/
  // Gait Parameters  
  if (!n.getParam("gait_type", params->gaitType))
  {
    cout << "Error reading parameter/s (gaitType) from rosparam" <<endl; 
    cout << "Check config file is loaded and type is correct" << endl;
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
}

/***********************************************************************************************************************
 * Gets ALL joint positions from joint state messages
***********************************************************************************************************************/
void jointStatesCallback(const sensor_msgs::JointState &joint_States)
{  
  if (!jointPosFlag)
  {
    for (int i=0; i<joint_States.name.size(); i++)
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