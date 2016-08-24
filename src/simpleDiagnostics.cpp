#pragma once
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <map>

static sensor_msgs::JointState jointStatesActual[6];
static sensor_msgs::JointState jointStatesDesired[6];
static sensor_msgs::JointState jointStatesMotor[6];

static std::map<std::string, int> jointMap;

ros::Publisher jointStatesActualPublishers[6];
ros::Publisher jointStatesDesiredPublishers[6];
ros::Publisher jointStatesMotorPublishers[6];

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
 * Callback for actual joint state messages
***********************************************************************************************************************/
void jointStatesActualCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesActual);
  
  //Publish each leg actual state
  for (int i=0; i<6; i++)
  {
    jointStatesActualPublishers[i].publish(jointStatesActual[i]);
  }
}

/***********************************************************************************************************************
 * Callback for desired joint state messages
***********************************************************************************************************************/
void jointStatesDesiredCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesDesired);
  
  //Publish each leg actual state
  for (int i=0; i<6; i++)
  {
    jointStatesDesiredPublishers[i].publish(jointStatesDesired[i]);
  }
}

/***********************************************************************************************************************
 * Callback for motor joint state messages
***********************************************************************************************************************/
void jointStatesMotorCallback(const sensor_msgs::JointState &jointStates)
{  
  populateJointStates(jointStates, jointStatesMotor);
  
  //Publish each leg actual state
  for (int i=0; i<6; i++)
  {
    jointStatesMotorPublishers[i].publish(jointStatesMotor[i]);
  }
}

/***********************************************************************************************************************
 * Main
***********************************************************************************************************************/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_diagnostics");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	
	//Attempt to subscribe to one of two possible joint state topics
	ros::Subscriber jointStatesActualSubscriber1 = n.subscribe("/hexapod_joint_state", 1000, &jointStatesActualCallback);
	ros::Subscriber jointStatesActualSubscriber2 = n.subscribe("/hexapod/joint_states", 1000, &jointStatesActualCallback);
	
	ros::Subscriber jointStatesDesiredSubscriber = n.subscribe("/desired_hexapod_joint_state", 1000, &jointStatesDesiredCallback);
	
	ros::Subscriber jointStatesMotorSubscriber = n.subscribe("/desired_motor_state", 1000, &jointStatesMotorCallback);
	
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
	  jointStatesMotor[i].name.resize(3);
	  jointStatesMotor[i].position.resize(3);
	  jointStatesMotor[i].velocity.resize(3);
	  jointStatesMotor[i].effort.resize(3);
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

	jointStatesMotorPublishers[0] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_left_joint_state/motor", 1000);
	jointStatesMotorPublishers[1] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/front_right_joint_state/motor", 1000);
	jointStatesMotorPublishers[2] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_left_joint_state/motor", 1000);
	jointStatesMotorPublishers[3] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/middle_right_joint_state/motor", 1000);
	jointStatesMotorPublishers[4] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_left_joint_state/motor", 1000);
	jointStatesMotorPublishers[5] = n.advertise<sensor_msgs::JointState>("/simple_diagnostics/rear_right_joint_state/motor", 1000);

	ros::spin();
	return 0;
}

/***********************************************************************************************************************
***********************************************************************************************************************/
