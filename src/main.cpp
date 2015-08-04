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
#include <boost/concept_check.hpp>
#include <iostream>
#include <sys/select.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <boost/circular_buffer.hpp> 

static Vector2d localVelocity(0,0);
static double turnRate = 0;
sensor_msgs::Imu imu;
sensor_msgs::JointState jointStates;
double jointPositions [18];
bool jointPosFlag = false;

// target rather than measured data
static Vector3d offsetPos(0.0,0.0,0.0);
static Vector3d offsetVel(0,0,0);

// source catkin_ws/devel/setup.bash
// roslaunch hexapod_teleop hexapod_controllers.launch

void imuCallback(const sensor_msgs::Imu &imudata)
{  
  imu = imudata;
}


void jointStatesCallback(const sensor_msgs::JointState &joint_States)
{  
  if (!jointPosFlag)
  {
    for (int i=0; i<joint_States.name.size(); i++)
    {
      if (!strcmp(joint_States.name[i].c_str(), "AL_coxa_joint"))
        jointPositions[0] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "AL_femur_joint"))
        jointPositions[1] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "AL_tibia_joint"))
        jointPositions[2] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "AR_coxa_joint"))
        jointPositions[3] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "AR_femur_joint"))
        jointPositions[4] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "AR_tibia_joint"))
        jointPositions[5] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BL_coxa_joint"))
        jointPositions[6] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BL_femur_joint"))
        jointPositions[7] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BL_tibia_joint"))
        jointPositions[8] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BR_coxa_joint"))
        jointPositions[9] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BR_femur_joint"))
        jointPositions[10] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "BR_tibia_joint"))
        jointPositions[11] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CL_coxa_joint"))
        jointPositions[12] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CL_femur_joint"))
        jointPositions[13] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CL_tibia_joint"))
        jointPositions[14] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CR_coxa_joint"))
        jointPositions[15] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CR_femur_joint"))
        jointPositions[16] = joint_States.position[i];
      else if (!strcmp(joint_States.name[i].c_str(), "CR_tibia_joint"))
        jointPositions[17] = joint_States.position[i];
      cout << "Joint: " << joint_States.name[i].c_str() << " set as: " << joint_States.position[i] << endl;
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

Pose compensation(const Vector3d &targetAccel, double targetAngularVel)
{
  Pose adjust;
  Quat orient;            //Orientation from IMU in quat
  Vector3d accel;         //Accelerations with respect to the IMU
  Vector3d rotation;
  Vector3d angularAcc;
  static Vector3d angularVel(0,0,0);
  adjust.rotation=Quat(Vector3d(0,0,0));
  static boost::circular_buffer<float> cbx(4,0);
  static boost::circular_buffer<float> cby(4,1);
  static boost::circular_buffer<float> cbz(4,2);
 
  orient.w = imu.orientation.w;
  orient.x = imu.orientation.x;
  orient.y = imu.orientation.y;
  orient.z = imu.orientation.z;
  accel(1) = -imu.linear_acceleration.x;
  accel(0) = -imu.linear_acceleration.y;
  accel(2) = -imu.linear_acceleration.z;
  
  /*//Compensation for gravity
  Vector3d accelcomp;
  RowVector3d gravityrot; 
  gravityrot=orient.toRotationMatrix()*Vector3d(0,0,-9.81);
  accelcomp(0)=accel(0)-gravityrot(0);
  accelcomp(1)=accel(1)-gravityrot(1);  
  accelcomp(2)=accel(2)-gravityrot(2);  
  ROS_ERROR("ACCEL= %f %f %f", accel(0), accel(1),accel(2));0
  ROS_ERROR("GRAVITYROT= %f %f %f", gravityrot(0), gravityrot(1),gravityrot(2)); 
  ROS_ERROR("ACCELCOMP= %f %f %f", accelcomp(0), accelcomp(1),accelcomp(2));*/
  
//Postion compensation

//#define ZERO_ORDER_FEEDBACK
//#define FIRST_ORDER_FEEDBACK
#define SECOND_ORDER_FEEDBACK

#if defined(SECOND_ORDER_FEEDBACK)
  double imuStrength = 1;
  double stiffness = 13; // how strongly/quickly we return to the neutral pose
  Vector3d offsetAcc = imuStrength*(targetAccel-accel+Vector3d(0,0,9.8)) - sqr(stiffness)*offsetPos - 2.0*stiffness*offsetVel;
  offsetVel += offsetAcc*timeDelta;
  offsetPos += offsetVel*timeDelta;
  
#elif defined(FIRST_ORDER_FEEDBACK)
  double imuStrength = 0.5;
  double stiffness = 10; // how strongly/quickly we return to the neutral pose
  offsetVel = imuStrength*(targetAccel-accel+Vector3d(0,0,9.8)) - sqr(stiffness)*offsetPos;
  offsetPos += offsetVel*timeDelta;
  
#elif defined(ZERO_ORDER_FEEDBACK)
  double imuStrength = 0.001;
  cbx.push_back(-imu.linear_acceleration.y); 
  cby.push_back(-imu.linear_acceleration.x); 
  cbz.push_back(-imu.linear_acceleration.z);
  offsetPos(0) = imuStrength * (targetAccel(0)-(cbx[0]+cbx[1]+cbx[2]+cbx[3]+cbx[3])/4);
  offsetPos(1) = imuStrength * (targetAccel(1)-(cby[0]+cby[1]+cby[2]+cby[3]+cbx[3])/4);
  offsetPos(2) = imuStrength * (targetAccel(2)-(cbz[0]+cbz[1]+cbz[2]+cbz[3]+cbx[3])/4 + 9.8);
  
  //offsetPos = imuStrength * (targetAccel-accel+Vector3d(0,0,9.8));
 
#endif
  
  //Angular body velocity compensation.
  double stiffnessangular=5;
  Vector3d angleDelta = adjust.rotation.toRotationVector();  
  //angularAcc(0)=-sqr(stiffnessangular)*angleDelta(0) + 2.0*stiffnessangular*(imu.angular_velocity.y - angularVel(0));
  //angularAcc(1)=-sqr(stiffnessangular)*angleDelta(1) + 2.0*stiffnessangular*(imu.angular_velocity.x - angularVel(1));
  angularAcc= -sqr(stiffnessangular)*angleDelta + 2.0*stiffnessangular*(Vector3d(0,0,targetAngularVel) - Vector3d(-imu.angular_velocity.y, -imu.angular_velocity.x, -imu.angular_velocity.z) - angularVel);
  angularVel += angularAcc*timeDelta;
  rotation(0)=angularVel(0)*timeDelta;
  rotation(1)=angularVel(1)*timeDelta;
  rotation(2)=angularVel(2)*timeDelta;
  
  /*// control towards imu's orientation
  double stiffnessangular=15;
  Quat targetAngle = ~orient;
  Vector3d angleDelta = (targetAngle*(~adjust.rotation)).toRotationVector(); // diff=target*current^-1
  angleDelta[2] = 0;  // this may not be quite right  
  angularAcc = sqr(stiffnessangular)*angleDelta -2.0*stiffnessangular*angularVel;
  angularVel += angularAcc*timeDelta;  
  rotation(0)=angularVel(0)*timeDelta;
  rotation(1)=angularVel(1)*timeDelta;
  rotation(2)=angularVel(2)*timeDelta;*/
  
  //adjust.rotation*= Quat(rotation);  
  adjust.position = offsetPos;
  return adjust;
}

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

//#define MOVE_TO_START    
#if defined(MOVE_TO_START)
#if defined(FLEXIPOD)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/joint_states", 1, jointStatesCallback);
#elif defined(LOBSANG)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/joint_states", 1, jointStatesCallback);
#elif defined(LARGE_HEXAPOD)
  //Check if the order is the same in the large hexapod!!(front left, front right, middle left, middle right...)
  ros::Subscriber jointStatesSubscriber = n.subscribe("/hexapod_joint_states", 1, jointStatesCallback);
#endif  

  for (int i=0; i<18; i++)
    jointPositions[i] = 1e10;
  
  while(!jointPosFlag)//If working with Rviz, (Not with an actual robot or gazebo), comment this two lines and the for loops
    ros::spinOnce();                      

  #endif
  
  bool dynamixel_interface = true;
  n_priv.param<bool>("dynamixel_interface", dynamixel_interface, true);
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
#endif

  
#if defined(MOVE_TO_START)     
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
#endif

#if defined(FLEXIPOD)
  WalkController walker(&hexapod, 1, 0.5, 0.12);
#elif defined(LOBSANG)
  WalkController walker(&hexapod, 1, 0.5, 0.1; 
#elif defined(LARGE_HEXAPOD)
  WalkController walker(&hexapod, 1, 0.18, 0.08);
#endif
    
  DebugOutput debug;

  double angle;  
  MotorInterface *interface;

  if (dynamixel_interface)
    interface = new DynamixelMotorInterface();
  else
    interface = new DynamixelProMotorInterface();

  interface->setupSpeed(0.5);
   
  ros::Rate r(roundToInt(1.0/timeDelta));         //frequency of the loop. 
  double t = 0;
  
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

    localVelocity[1] = time < 20.0 ? 0.18 : 0.0;
#if defined(MOVE_TO_START)
    if (!started)
      started = walker.moveToStart();
    else
#endif
      walker.update(localVelocity, turnRate*turnRate*turnRate, &adjust); // the cube just lets the thumbstick give small turns easier
    debug.drawRobot(hexapod.legs[0][0].rootOffset, hexapod.getJointPositions(walker.pose * adjust), Vector4d(1,1,1,1));
    debug.drawPoints(walker.targets, Vector4d(1,0,0,1));


    Vector3d oldMaxVel = maxVel;
    if (true)
    {
      for (int s = 0; s<2; s++)
      {
        double dir = s==0 ? -1 : 1;
        for (int l = 0; l<3; l++)
        {
#if defined(FLEXIPOD)
          angle = dir*(walker.model->legs[l][s].yaw - yawOffsets[l]);
          interface->setTargetAngle(l, s, 0, angle);
          angle = -dir*walker.model->legs[l][s].liftAngle;
          interface->setTargetAngle(l, s, 1, angle);
          angle = dir*walker.model->legs[l][s].kneeAngle;
          interface->setTargetAngle(l, s, 2, angle);
#elif defined(LOBSANG) // currently the same as flexipod above
          angle = dir*(walker.model->legs[l][s].yaw - yawOffsets[l]);
          interface->setTargetAngle(l, s, 0, angle);
          angle = -dir*walker.model->legs[l][s].liftAngle;
          interface->setTargetAngle(l, s, 1, angle);
          angle = dir*walker.model->legs[l][s].kneeAngle;
          interface->setTargetAngle(l, s, 2, angle);
#elif defined(LARGE_HEXAPOD) // currently the same as flexipod above - yaw offsets?
          angle = dir*(walker.model->legs[l][s].yaw);
          interface->setTargetAngle(l, s, 0, angle);
          angle = -dir*walker.model->legs[l][s].liftAngle;
          interface->setTargetAngle(l, s, 1, angle);
          angle = dir*walker.model->legs[l][s].kneeAngle;
          interface->setTargetAngle(l, s, 2, angle);
#endif
          if (!firstFrame)
          {
            maxVel[0] = max(maxVel[0], abs(walker.model->legs[l][s].yaw - walker.model->legs[l][s].debugOldYaw)/timeDelta);
            maxVel[1] = max(maxVel[1], abs(walker.model->legs[l][s].liftAngle - walker.model->legs[l][s].debugOldLiftAngle)/timeDelta);
            maxVel[2] = max(maxVel[2], abs(walker.model->legs[l][s].kneeAngle - walker.model->legs[l][s].debugOldKneeAngle)/timeDelta);
          }
          
          walker.model->legs[l][s].debugOldYaw = walker.model->legs[l][s].yaw;
          walker.model->legs[l][s].debugOldLiftAngle = walker.model->legs[l][s].liftAngle;
          walker.model->legs[l][s].debugOldKneeAngle = walker.model->legs[l][s].kneeAngle;
        }
      }
      interface->publish();
    }
    firstFrame = false;
    if (maxVel != oldMaxVel)
      cout << "max yaw vel: " << maxVel[0] << ", hip vel: " << maxVel[1] << ", knee vel: " << maxVel[2] << endl;
    ros::spinOnce();
    r.sleep();

    debug.reset();
    t += timeDelta;
  }
}

