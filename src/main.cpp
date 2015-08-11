/* (c) Copyright CSIRO 2013. Author: Thomas Lowe
   This software is provided under the terms of Schedule 1 of the license agreement between CSIRO, 3DLM and GeoSLAM.
*/
#include "../include/simple_hexapod_controller/standardIncludes.h"
#include "../include/simple_hexapod_controller/model.h"
#include "../include/simple_hexapod_controller/walkController.h"
#include "../include/simple_hexapod_controller/debugOutput.h"
#include "../include/simple_hexapod_controller/motorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelMotorInterface.h"
#include "../include/simple_hexapod_controller/dynamixelProMotorInterface.h"
#include "../include/simple_hexapod_controller/imuCompensation.h"
#include <boost/concept_check.hpp>
#include <iostream>
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <boost/circular_buffer.hpp> 

static Vector2d localVelocity(0,0);
static double turnRate = 0;
sensor_msgs::JointState jointStates;
double jointPositions[18];
bool jointPosFlag = false;

// target rather than measured data
static Vector3d offsetPos(0.0,0.0,0.0);
static Vector3d offsetVel(0,0,0);

void jointStatesCallback(const sensor_msgs::JointState &joint_States);

void joypadChangeCallback(const geometry_msgs::Twist &twist)
{
  //ASSERT(twist.angular.z < 0.51);
 // ASSERT(twist.linear.y < 0.51);
  // these are 0 to 5 for some reason, so multiply by 0.2
  localVelocity = Vector2d(twist.linear.x, twist.linear.y);
  localVelocity = clamped(localVelocity, 1.0);
  turnRate = -twist.angular.z;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Hexapod");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  ros::Subscriber subscriber = n.subscribe("/desired_body_velocity", 1, joypadChangeCallback);
  ros::Subscriber imuSubscriber = n.subscribe("/ig/imu/data_ned", 1, imuCallback);
  
  ros::Rate r(roundToInt(1.0/timeDelta));         //frequency of the loop. 
  double t = 0;

//#define MOVE_TO_START    
#if defined(MOVE_TO_START)
#if defined(FLEXIPOD)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/hexapod/joint_states", 1, jointStatesCallback);
#elif defined(LOBSANG)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/joint_states", 1, jointStatesCallback);
#elif defined(LARGE_HEXAPOD)
  //Check if the order is the same in the large hexapod!!(front left, front right, middle left, middle right...)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/hexapod_joint_state", 1, jointStatesCallback);
#endif  

  for (int i=0; i<18; i++)
    jointPositions[i] = 1e10;
  
  int spin = 10; //Max ros spin cycles to find joint positions
  while(spin--)//If working with Rviz, (Not with an actual robot or gazebo), comment this two lines and the for loops
  {
    ros::spinOnce();
    r.sleep();
  }
#endif
 
  bool dynamixel_interface = true;
  n_priv.param<bool>("dynamixel_interface", dynamixel_interface, false);
  Vector3d yawOffsets(0,0,0);  

#if defined(FLEXIPOD)
  yawOffsets = Vector3d(0.77,0,-0.77);  
  Model hexapod(yawOffsets, Vector3d(1.4,1.4,1.4), Vector2d(0,1.9));
#elif defined(LOBSANG)  
  yawOffsets = Vector3d(0.77,0,-0.77);   
  Model hexapod(yawOffsets, Vector3d(1.4,1.4,1.4), Vector2d(0,1.9));  
#elif defined(LARGE_HEXAPOD)
  double yawLimit = 30;
  Vector3d yawLimits = Vector3d(yawLimit, yawLimit, yawLimit)*pi/180.0;
  Vector2d kneeLimit = Vector2d(50, 160)*pi/180.0;
  Vector2d hipLimit = Vector2d(-25, 80)*pi/180.0;
  Model hexapod(Vector3d(45,0,-45)*pi/180.0, yawLimits, kneeLimit, hipLimit);
  hexapod.jointMaxAngularSpeeds = Vector3d(1.0, 0.4, 0.4);
#endif

  
#if defined(MOVE_TO_START)  
  if (jointPosFlag)
  {
    // set initial leg angles
    for (int leg = 0; leg<3; leg++)
    {
      for (int side = 0; side<2; side++)
      {
        double dir = side==0 ? -1 : 1;
        int index = leg*6+(side == 0 ? 0 : 3);
        hexapod.setLegStartAngles(side, leg, dir*Vector3d(jointPositions[index+0]+dir*yawOffsets[leg], -jointPositions[index+1], jointPositions[index+2]));
        cout << "leg << " << leg << ", side: " << side << 
        " values: " << hexapod.legs[leg][side].yaw << ", " << hexapod.legs[leg][side].liftAngle << ", " << hexapod.legs[leg][side].kneeAngle << endl;
      }
    }
  }
  else
    cout << "Failed to acquire all joint position values" << endl;
#endif

#if defined(FLEXIPOD)
  WalkController walker(&hexapod, 1, 0.5, 0.12);
#elif defined(LOBSANG)
  WalkController walker(&hexapod, 1, 0.5, 0.1; 
#elif defined(LARGE_HEXAPOD)
  WalkController walker(&hexapod, 1, 0.18, 0.06, 0.4, 0.6);
#endif
    
  DebugOutput debug;
  setCompensationDebug(debug);

  MotorInterface *interface;

  if (dynamixel_interface)
    interface = new DynamixelMotorInterface();
  else
    interface = new DynamixelProMotorInterface();

  interface->setupSpeed(0.5);
   

  
  Vector3d maxVel(0,0,0);
  bool firstFrame = true;
  bool started = false;
  double time = 0;
  while (ros::ok())
  {
    time += timeDelta;
    Pose adjust = Pose::identity(); // offset pose for body. Use this to close loop with the IMU    
    Vector2d acc = walker.localCentreAcceleration;
    //adjust = compensation(Vector3d(acc[0], acc[1], 0), walker.angularVelocity);

    localVelocity[1] = time < 3*pi ? 0.2 : 0.0;
    if (time > 3*pi+0.5)
      localVelocity[1] = 0.2;
#if defined(MOVE_TO_START)
    if (!started)
      started = walker.moveToStart();
    else
#endif
      walker.update(localVelocity, turnRate*turnRate*turnRate, &adjust); // the cube just lets the thumbstick give small turns easier
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
          double yaw = dir*(walker.model->legs[l][s].yaw - yawOffsets[l]);
          double lift = -dir*walker.model->legs[l][s].liftAngle;
          double knee = dir*walker.model->legs[l][s].kneeAngle;
          if (!firstFrame)
          {
            double yawVel = (yaw - walker.model->legs[l][s].debugOldYaw)/timeDelta;
            double liftVel = (lift - walker.model->legs[l][s].debugOldLiftAngle)/timeDelta;
            double kneeVel = (knee - walker.model->legs[l][s].debugOldKneeAngle)/timeDelta;
            if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[0])
              yaw = walker.model->legs[l][s].debugOldYaw + sign(yawVel)*hexapod.jointMaxAngularSpeeds[0]*timeDelta;
            if (abs(liftVel) > hexapod.jointMaxAngularSpeeds[1])
              lift = walker.model->legs[l][s].debugOldLiftAngle + sign(liftVel)*hexapod.jointMaxAngularSpeeds[1]*timeDelta;
            if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[2])
              knee = walker.model->legs[l][s].debugOldKneeAngle + sign(kneeVel)*hexapod.jointMaxAngularSpeeds[2]*timeDelta;
            if (abs(yawVel) > hexapod.jointMaxAngularSpeeds[0] || abs(liftVel) > hexapod.jointMaxAngularSpeeds[1] || abs(yawVel) > hexapod.jointMaxAngularSpeeds[2])
              cout << "WARNING: MAXIMUM SPEED EXCEEDED! Clamping to maximum angular speed for leg " << l << " side " << s << endl;
            maxVel[0] = max(maxVel[0], abs(yawVel));
            maxVel[1] = max(maxVel[1], abs(liftVel));
            maxVel[2] = max(maxVel[2], abs(kneeVel));
          }
          interface->setTargetAngle(l, s, 0, yaw);
          interface->setTargetAngle(l, s, 1, lift);
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
    t += timeDelta;
  }
}

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
