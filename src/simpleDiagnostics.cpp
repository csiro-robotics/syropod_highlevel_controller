#pragma once
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <map>
#include "std_msgs/Float64.h"

static sensor_msgs::JointState jointStatesActual[6];
static sensor_msgs::JointState jointStatesDesired[6];
static sensor_msgs::JointState jointStatesMotorActual[6];
static sensor_msgs::JointState jointStatesMotorDesired[6];

static std::map<std::string, int> jointMap;

ros::Publisher jointStatesActualPublishers[6];
ros::Publisher jointStatesDesiredPublishers[6];
ros::Publisher jointStatesMotorActualPublishers[6];
ros::Publisher jointStatesMotorDesiredPublishers[6];

enum DataType
{
  JOINT_POSITION,
  JOINT_VELOCITY,
  JOINT_EFFORT,
};

/***********************************************************************************************************************
 * Generates map of all joints including name and index number
***********************************************************************************************************************/
void populateJointMap(std::map<std::string, int> *jointMap)
{    
  jointMap->insert(std::map<std::string, int>::value_type("front_left_body_coxa", 0));
  jointMap->insert(std::map<std::string, int>::value_type("front_left_coxa_femour", 1));
  jointMap->insert(std::map<std::string, int>::value_type("front_left_femour_tibia", 2));
  jointMap->insert(std::map<std::string, int>::value_type("front_right_body_coxa", 3));
  jointMap->insert(std::map<std::string, int>::value_type("front_right_coxa_femour", 4));
  jointMap->insert(std::map<std::string, int>::value_type("front_right_femour_tibia", 5));
  jointMap->insert(std::map<std::string, int>::value_type("middle_left_body_coxa", 6));
  jointMap->insert(std::map<std::string, int>::value_type("middle_left_coxa_femour", 7));
  jointMap->insert(std::map<std::string, int>::value_type("middle_left_femour_tibia", 8));
  jointMap->insert(std::map<std::string, int>::value_type("middle_right_body_coxa", 9));
  jointMap->insert(std::map<std::string, int>::value_type("middle_right_coxa_femour", 10));
  jointMap->insert(std::map<std::string, int>::value_type("middle_right_femour_tibia", 11));
  jointMap->insert(std::map<std::string, int>::value_type("rear_left_body_coxa", 12));
  jointMap->insert(std::map<std::string, int>::value_type("rear_left_coxa_femour", 13));
  jointMap->insert(std::map<std::string, int>::value_type("rear_left_femour_tibia", 14));
  jointMap->insert(std::map<std::string, int>::value_type("rear_right_body_coxa", 15));
  jointMap->insert(std::map<std::string, int>::value_type("rear_right_coxa_femour", 16));
  jointMap->insert(std::map<std::string, int>::value_type("rear_right_femour_tibia", 17));
    
  jointMap->insert(std::map<std::string, int>::value_type("AL_coxa_joint", 0));
  jointMap->insert(std::map<std::string, int>::value_type("AL_femur_joint", 1));
  jointMap->insert(std::map<std::string, int>::value_type("AL_tibia_joint", 2));
  jointMap->insert(std::map<std::string, int>::value_type("AR_coxa_joint", 3));
  jointMap->insert(std::map<std::string, int>::value_type("AR_femur_joint", 4));
  jointMap->insert(std::map<std::string, int>::value_type("AR_tibia_joint", 5));
  jointMap->insert(std::map<std::string, int>::value_type("BL_coxa_joint", 6));
  jointMap->insert(std::map<std::string, int>::value_type("BL_femur_joint", 7));
  jointMap->insert(std::map<std::string, int>::value_type("BL_tibia_joint", 8));
  jointMap->insert(std::map<std::string, int>::value_type("BR_coxa_joint", 9));
  jointMap->insert(std::map<std::string, int>::value_type("BR_femur_joint", 10));
  jointMap->insert(std::map<std::string, int>::value_type("BR_tibia_joint", 11));
  jointMap->insert(std::map<std::string, int>::value_type("CL_coxa_joint", 12));
  jointMap->insert(std::map<std::string, int>::value_type("CL_femur_joint", 13));
  jointMap->insert(std::map<std::string, int>::value_type("CL_tibia_joint", 14));
  jointMap->insert(std::map<std::string, int>::value_type("CR_coxa_joint", 15));
  jointMap->insert(std::map<std::string, int>::value_type("CR_femur_joint", 16));
  jointMap->insert(std::map<std::string, int>::value_type("CR_tibia_joint", 17));
}

/***********************************************************************************************************************
 * Takes joint state data from source message and organises if correctly in destination message format
***********************************************************************************************************************/
void populateJointStates(sensor_msgs::JointState jointStatesSource, sensor_msgs::JointState *jointStatesDestination)
{  
  for (uint i=0; i<jointStatesSource.name.size(); i++)
  {
    const char* jointName = jointStatesSource.name[i].c_str();
    int index = jointMap[jointName];
    
    jointStatesDestination[index/3].header.stamp = ros::Time::now();
    
    if (jointStatesSource.name.size() != 0)
    {
      jointStatesDestination[index/3].name[index%3] = jointStatesSource.name[i];
    }
    
    if (jointStatesSource.position.size() != 0)
    {
      jointStatesDestination[index/3].position[index%3] = jointStatesSource.position[i];
    }
    
    if (jointStatesSource.velocity.size() != 0)
    {
      jointStatesDestination[index/3].velocity[index%3] = jointStatesSource.velocity[i];
    }

    if (jointStatesSource.effort.size() != 0)
    {
      jointStatesDestination[index/3].effort[index%3] = jointStatesSource.effort[i];
    }
  }
}

/***********************************************************************************************************************
 * Takes joint state data from source message and organises if correctly in destination message format
***********************************************************************************************************************/
void populateJointStates(std::string jointName, DataType dataType, double dataValue, sensor_msgs::JointState *jointStatesDestination)
{  
  int index = jointMap[jointName];
  
  jointStatesDestination[index/3].header.stamp = ros::Time::now();
  
  jointStatesDestination[index/3].name[index%3] = jointName;
  
  if (dataType == JOINT_POSITION)
  {
    jointStatesDestination[index/3].position[index%3] = dataValue;
  }  
  else if (dataType == JOINT_VELOCITY)
  {
    jointStatesDestination[index/3].velocity[index%3] = dataValue;
  }
  else if (dataType == JOINT_EFFORT)
  {
    jointStatesDestination[index/3].effort[index%3] = dataValue;
  }
}

/***********************************************************************************************************************
 * Callback for actual joint state messages
***********************************************************************************************************************/
void jointStatesActualCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesActual);
}

/***********************************************************************************************************************
 * Callback for desired joint state messages
***********************************************************************************************************************/
void jointStatesDesiredCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesDesired);
}

/***********************************************************************************************************************
 * Callback for desired motor joint state messages
***********************************************************************************************************************/
void jointStatesMotorDesiredCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesMotorDesired);
}

/***********************************************************************************************************************
 * Callback for actual motor joint state messages
***********************************************************************************************************************/
void jointStatesMotorActualCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesMotorActual);
}

/***********************************************************************************************************************
 * Callback for desired position subscriber
***********************************************************************************************************************/
void jointPositionDesiredCallback(const std_msgs::Float64::ConstPtr &position, const std::string &jointName)
{
  populateJointStates(jointName, JOINT_POSITION, position->data, jointStatesDesired);
}

/***********************************************************************************************************************
 * Callback for desired velocity subscriber
***********************************************************************************************************************/
void jointVelocityDesiredCallback(const std_msgs::Float64::ConstPtr &velocity, const std::string &jointName)
{
  populateJointStates(jointName, JOINT_VELOCITY, velocity->data, jointStatesDesired);
}

/***********************************************************************************************************************
 * Callback for desired effort subscriber
***********************************************************************************************************************/
void jointEffortDesiredCallback(const std_msgs::Float64::ConstPtr &effort, const std::string &jointName)
{
  populateJointStates(jointName, JOINT_EFFORT, effort->data, jointStatesDesired);
}

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_diagnostics");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  
  ros::Rate r(100);

  //Attempt to subscribe to one of two possible joint state topics
  ros::Subscriber jointStatesActualSubscriber1 = n.subscribe("/hexapod_joint_state", 1000, &jointStatesActualCallback);
  ros::Subscriber jointStatesActualSubscriber2 = n.subscribe("/hexapod/joint_states", 1000, &jointStatesActualCallback);

  ros::Subscriber jointStatesDesiredSubscriber = n.subscribe("/desired_hexapod_joint_state", 1000, &jointStatesDesiredCallback);

  ros::Subscriber jointPositionDesiredSubscriber1 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_left_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber2 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_left_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber3 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_left_femour_tibia"));
  ros::Subscriber jointPositionDesiredSubscriber4 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_right_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber5 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_right_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber6 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "front_right_femour_tibia"));
  ros::Subscriber jointPositionDesiredSubscriber7 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_left_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber8 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_left_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber9 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_left_femour_tibia"));
  ros::Subscriber jointPositionDesiredSubscriber10 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_right_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber11 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_right_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber12 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "middle_right_femour_tibia"));
  ros::Subscriber jointPositionDesiredSubscriber13 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_left_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber14 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_left_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber15 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_left_femour_tibia"));
  ros::Subscriber jointPositionDesiredSubscriber16 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_body_coxa/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_right_body_coxa"));
  ros::Subscriber jointPositionDesiredSubscriber17 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_right_coxa_femour"));
  ros::Subscriber jointPositionDesiredSubscriber18 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/command", 1000, boost::bind(jointPositionDesiredCallback, _1, "rear_right_femour_tibia"));
  
  ros::Subscriber jointVelocityDesiredSubscriber1 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_left_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber2 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_left_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber3 = n.subscribe<std_msgs::Float64>("/hexapod/front_left_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_left_femour_tibia"));
  ros::Subscriber jointVelocityDesiredSubscriber4 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_right_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber5 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_right_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber6 = n.subscribe<std_msgs::Float64>("/hexapod/front_right_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "front_right_femour_tibia"));
  ros::Subscriber jointVelocityDesiredSubscriber7 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_left_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber8 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_left_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber9 = n.subscribe<std_msgs::Float64>("/hexapod/middle_left_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_left_femour_tibia"));
  ros::Subscriber jointVelocityDesiredSubscriber10 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_right_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber11 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_right_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber12 = n.subscribe<std_msgs::Float64>("/hexapod/middle_right_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "middle_right_femour_tibia"));
  ros::Subscriber jointVelocityDesiredSubscriber13 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_left_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber14 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_left_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber15 = n.subscribe<std_msgs::Float64>("/hexapod/rear_left_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_left_femour_tibia"));
  ros::Subscriber jointVelocityDesiredSubscriber16 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_body_coxa/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_right_body_coxa"));
  ros::Subscriber jointVelocityDesiredSubscriber17 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_coxa_femour/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_right_coxa_femour"));
  ros::Subscriber jointVelocityDesiredSubscriber18 = n.subscribe<std_msgs::Float64>("/hexapod/rear_right_femour_tibia/velocity", 1000, boost::bind(jointVelocityDesiredCallback, _1, "rear_right_femour_tibia"));

  ros::Subscriber jointStatesMotorDesiredSubscriber = n.subscribe("/desired_motor_state", 1000, &jointStatesMotorDesiredCallback);
  ros::Subscriber jointStatesMotorActualSubscriber = n.subscribe("/motor_encoders", 1000, &jointStatesMotorActualCallback);

  populateJointMap(&jointMap);

  for (int i=0; i<6; i++)
  {
    jointStatesActual[i].name.resize(3);
    jointStatesActual[i].position.resize(3);
    jointStatesActual[i].velocity.resize(3);
    jointStatesActual[i].effort.resize(3);
    jointStatesDesired[i].name.resize(3);
    jointStatesDesired[i].position.resize(3);
    jointStatesDesired[i].velocity.resize(3);
    jointStatesDesired[i].effort.resize(3);
    jointStatesMotorActual[i].name.resize(3);
    jointStatesMotorActual[i].position.resize(3);
    jointStatesMotorActual[i].velocity.resize(3);
    jointStatesMotorActual[i].effort.resize(3);
    jointStatesMotorDesired[i].name.resize(3);
    jointStatesMotorDesired[i].position.resize(3);
    jointStatesMotorDesired[i].velocity.resize(3);
    jointStatesMotorDesired[i].effort.resize(3);
  }

  jointStatesActualPublishers[0] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_left_joint_state/actual", 1000);
  jointStatesActualPublishers[1] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_right_joint_state/actual", 1000);
  jointStatesActualPublishers[2] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_left_joint_state/actual", 1000);
  jointStatesActualPublishers[3] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_right_joint_state/actual", 1000);
  jointStatesActualPublishers[4] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_left_joint_state/actual", 1000);
  jointStatesActualPublishers[5] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_right_joint_state/actual", 1000);

  jointStatesDesiredPublishers[0] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_left_joint_state/desired", 1000);
  jointStatesDesiredPublishers[1] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_right_joint_state/desired", 1000);
  jointStatesDesiredPublishers[2] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_left_joint_state/desired", 1000);
  jointStatesDesiredPublishers[3] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_right_joint_state/desired", 1000);
  jointStatesDesiredPublishers[4] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_left_joint_state/desired", 1000);
  jointStatesDesiredPublishers[5] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_right_joint_state/desired", 1000);

  jointStatesMotorActualPublishers[0] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_left_joint_state/motor_actual", 1000);
  jointStatesMotorActualPublishers[1] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_right_joint_state/motor_actual", 1000);
  jointStatesMotorActualPublishers[2] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_left_joint_state/motor_actual", 1000);
  jointStatesMotorActualPublishers[3] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_right_joint_state/motor_actual", 1000);
  jointStatesMotorActualPublishers[4] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_left_joint_state/motor_actual", 1000);
  jointStatesMotorActualPublishers[5] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_right_joint_state/motor_actual", 1000);
  
  jointStatesMotorDesiredPublishers[0] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_left_joint_state/motor_desired", 1000);
  jointStatesMotorDesiredPublishers[1] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_right_joint_state/motor_desired", 1000);
  jointStatesMotorDesiredPublishers[2] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_left_joint_state/motor_desired", 1000);
  jointStatesMotorDesiredPublishers[3] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_right_joint_state/motor_desired", 1000);
  jointStatesMotorDesiredPublishers[4] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_left_joint_state/motor_desired", 1000);
  jointStatesMotorDesiredPublishers[5] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_right_joint_state/motor_desired", 1000);

  while (ros::ok())
  {
    for (int i=0; i<6; i++)
    {
      jointStatesActualPublishers[i].publish(jointStatesActual[i]);
      jointStatesDesiredPublishers[i].publish(jointStatesDesired[i]);
      jointStatesMotorActualPublishers[i].publish(jointStatesMotorActual[i]);
      jointStatesMotorDesiredPublishers[i].publish(jointStatesMotorDesired[i]);
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
