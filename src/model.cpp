
#include "../include/simple_hexapod_controller/model.h"

/***********************************************************************************************************************
 * Defines hexapod model
***********************************************************************************************************************/
Model::Model(Parameters* params)
  : leg_count_(params->leg_id.data.size())
{
  for (int i=0; i < leg_count_; ++i)
  {   
    Leg* leg = new Leg(this, i, params);
    leg_container_.insert(std::map<int, Leg*>::value_type(i, leg));
  }
}

/***********************************************************************************************************************
 * Initialise all legs in model
***********************************************************************************************************************/
void Model::initLegs(bool use_default_joint_positions)
{
  // Set initial leg angles
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;
    leg->init(use_default_joint_positions);
  } 
}

/***********************************************************************************************************************
 * Restrict joint angles to limits
***********************************************************************************************************************/
void Model::clampToLimits(void)
{
  // Clamp desired joint positions and alert if a limit has been hit
  std::map<int, Leg*>::iterator leg_it;
  for (leg_it = leg_container_.begin(); leg_it != leg_container_.end(); ++leg_it)
  {
    Leg* leg = leg_it->second;    
    std::map<int, Joint*>::iterator joint_it;
    for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it)
    {
      Joint* joint = joint_it->second;
      double message_tolerance = 0.01;
      if (joint->desired_position < joint->min_position)
      {
	joint->desired_position = joint->min_position;
	double diff = abs(joint->desired_position - joint->min_position);
	ROS_WARN_COND(diff > message_tolerance, 
		      "%s leg has tried to exceed %s min joint limit: %f by %f. Clamping joint to limit.\n", 
		      leg->getIDName().c_str(), joint->name.c_str(), joint->min_position, diff);
      }      
      else if (joint->desired_position > joint->max_position)
      {
	joint->desired_position = joint->max_position;
	double diff = abs(joint->desired_position - joint->max_position);
	ROS_WARN_COND(diff > message_tolerance, 
		      "%s leg has tried to exceed %s max joint limit: %f by %f. Clamping joint to limit.\n", 
		      leg->getIDName().c_str(), joint->name.c_str(), joint->max_position, diff);
      }      
    }
  }
}

/***********************************************************************************************************************
 * Generic leg data object
***********************************************************************************************************************/
Leg::Leg(Model* model, int id_number, Parameters* params) 
  : model_(model)
  , id_number_(id_number)
  , id_name_(params->leg_id.data[id_number])
  , num_joints_(params->leg_DOF.data[id_name_])  
  , mirror_dir_(pow(-1.0, (id_number_%2)+1))
  , stance_leg_yaw_(params->leg_stance_yaws.data[id_name_])
  , leg_state_(WALKING)    
  , impedance_state_(std::vector<double>(2))
{  
  Joint* null_joint; //HACK
  Link* base_link = new Link(this, null_joint, 0, params);
  link_container_.insert(std::map<int, Link*>::value_type(0, base_link));
  ROS_DEBUG("%s successfully added to leg %s in model.", base_link->name.c_str(), id_name_.c_str());
  Link* prev_link = base_link;
  for (int i = 1; i < num_joints_+1; ++i)
  {
    Joint* new_joint = new Joint(this, prev_link, i, params);  
    Link* new_link = new Link(this, new_joint, i, params);
    joint_container_.insert(std::map<int, Joint*>::value_type(i, new_joint));
    link_container_.insert(std::map<int, Link*>::value_type(i, new_link));
    prev_link = new_link;
    ROS_DEBUG("%s and %s successfully added to leg %s in model.",
	     new_link->name.c_str(), new_joint->name.c_str(), id_name_.c_str());
  } 
  tip_ = new Tip(this, prev_link);
  
  if (num_joints_ == 2)
  {
    Link* femur_link = getLinkByIDName(id_name_ + "_femur_link");
    min_leg_length_ = 0.0;
    max_leg_length_ = femur_link->length;
  }
  else if (num_joints_ == 3)
  {
    Link* femur_link = getLinkByIDName(id_name_ + "_femur_link");
    Link* tibia_link = getLinkByIDName(id_name_ + "_tibia_link");
    Joint* tibia_joint = getJointByIDName(id_name_ + "_tibia_joint");  
    min_leg_length_ = sqrt(sqr(tibia_link->length) + sqr(femur_link->length) -
                      2.0 * femur_link->length * tibia_link->length * cos(max(0.0, pi - tibia_joint->max_position)));
    max_leg_length_ = sqrt(sqr(tibia_link->length) + sqr(femur_link->length) -
			2.0 * femur_link->length * tibia_link->length * cos(pi - max(0.0, tibia_joint->min_position)));
    
    tripod_group_ = (id_number > 1) ? abs(id_number%2-1):id_number; //(0,3,5 -> 0) && (1,2,4 -> 1)    
  }  
  ROS_DEBUG("Leg %s has been initialised as a %d degree of freedom leg with %lu links and %lu joints.",
	   id_name_.c_str(), num_joints_, link_container_.size(), joint_container_.size());  
}

/***********************************************************************************************************************
 * Initialises leg by setting desired joint positions to current position from encoders and running forward kinematics
***********************************************************************************************************************/
void Leg::init(bool use_default_joint_positions)
{
  std::map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (use_default_joint_positions)
    {
      joint->current_position = clamped(0.0, joint->min_position, joint->max_position);
      joint->current_velocity = 0.0;
      joint->current_effort = 0.0;
    }
    joint->desired_position = joint->current_position;
    joint->prev_desired_position = joint->desired_position;
  }
  applyFK();
}

/***********************************************************************************************************************
 * Search through joint container and return joint object associated with given identification name
***********************************************************************************************************************/
Joint* Leg::getJointByIDName(std::string joint_name)
{
  std::map<int, Joint*>::iterator joint_it;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    if (joint->name == joint_name)
    {
      return joint;
    }
  }
  return NULL;
}

/***********************************************************************************************************************
 * Search through link container and return link object associated with given identification name
***********************************************************************************************************************/
Link* Leg::getLinkByIDName(std::string link_name)
{
  std::map<int, Link*>::iterator link_it;
  for (link_it = link_container_.begin(); link_it != link_container_.end(); ++link_it)
  {
    Link* link = link_it->second;
    if (link->name == link_name)
    {
      return link;
    }
  }
  return NULL;
}

/***********************************************************************************************************************
 * Applies impedance controller delta z position to requested tip position and sets as desired tip position
***********************************************************************************************************************/
void Leg::applyDeltaZ(Vector3d tip_position)
{
  if (leg_state_ != MANUAL)  // Don't apply delta Z to manually manipulated legs
  {
    tip_position[2] -= delta_z_;
  }
  desired_tip_position_ = tip_position;
}

/***********************************************************************************************************************
 * Applies inverse kinematics to achieve target tip position
***********************************************************************************************************************/
void Leg::applyLocalIK(double time_delta)
{
  vector<map<string, double>> dh_parameters;
  std::map<int, Joint*>::iterator joint_it = joint_container_.begin();
  joint_it++; //Skip first joint dh parameters since it is a fixed transformation
  map<string, double> dh_map;
  for (; joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;    
    dh_map.insert(map<string, double>::value_type("d", joint->reference_link->offset));
    dh_map.insert(map<string, double>::value_type("theta", joint->reference_link->actuating_joint->desired_position));
    dh_map.insert(map<string, double>::value_type("r", joint->reference_link->length));
    dh_map.insert(map<string, double>::value_type("alpha", joint->reference_link->twist));
    dh_parameters.push_back(dh_map);  
    dh_map.clear();
  }
  //Add tip dh params
  dh_map.insert(map<string, double>::value_type("d", tip_->reference_link->offset));
  dh_map.insert(map<string, double>::value_type("theta", tip_->reference_link->actuating_joint->desired_position));
  dh_map.insert(map<string, double>::value_type("r", tip_->reference_link->length));
  dh_map.insert(map<string, double>::value_type("alpha", tip_->reference_link->twist));
  dh_parameters.push_back(dh_map);   

  MatrixXd j(3,num_joints_);
  switch(num_joints_)
  {
    case(1):
      j = create1DOFJacobian(dh_parameters);
      break;
    case(2):
      j = create2DOFJacobian(dh_parameters);
      break;
    case(3):
      j = create3DOFJacobian(dh_parameters);
      break;
  };
  
  //cout << j << endl;
  
  double dls_cooeficient = 0.5;
  MatrixXd identity = MatrixXd::Identity(3, 3);
  MatrixXd ik_matrix(num_joints_, 3);
  //ik_matrix = ((j.transpose()*j).inverse())*j.transpose();
  ik_matrix = j.transpose()*((j*j.transpose() + sqr(dls_cooeficient)*identity).inverse());  
  //cout << ik_matrix << endl;
  
  Vector3d tip_delta_pos = desired_tip_position_ - local_tip_position_;
  VectorXd joint_delta_pos(num_joints_);
  joint_delta_pos = ik_matrix*tip_delta_pos;
  
  int index = 0;
  for (joint_it = joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    joint->desired_position = joint->prev_desired_position+joint_delta_pos[index];
    /*
    if (joint->name == "AR_tibia_joint")
    {
      cout << joint->desired_position << endl;
    }
    */
  }
  Vector3d result = applyFK();
  if (id_name_ == "AR")
  {
    cout << desired_tip_position_[0] << " " << result[0] << endl;
  }
}

/***********************************************************************************************************************
 * Applies forward kinematics
***********************************************************************************************************************/
Vector3d Leg::applyFK(bool set_local)
{
  
  Vector4d tip_position_4d(0,0,0,1);  
  Joint* femur_joint = getJointByIDName(id_name_ + "_femur_joint");
  Joint* coxa_joint = getJointByIDName(id_name_ + "_coxa_joint");
  coxa_joint->desired_position = -0.785;
  femur_joint->desired_position = 0.785;
  updateTransforms();
  Matrix4d transform = tip_->getBaseTransform();
  cout << transform << endl;
  tip_position_4d = transform*tip_position_4d;
  Vector3d tip_position(tip_position_4d[0], tip_position_4d[1], tip_position_4d[2]);
  /*
  Joint* femur_joint = getJointByIDName(id_name_ + "_femur_joint");
  Matrix4d transform_femur = femur_joint->getBaseTransform();
  Matrix4d transform = tip_->getBaseTransform();
  cout << transform << endl;
  tip_position_4d = transform*tip_position_4d;
  Vector3d tip_position(tip_position_4d[0], tip_position_4d[1], tip_position_4d[2]);
  */
  if (set_local)
  {
    local_tip_position_ = tip_position;
  }
  return tip_position;
}

/***********************************************************************************************************************
 * Updates joint state transforms using current joint positions
***********************************************************************************************************************/
void Leg::updateTransforms(void)
{
  std::map<int, Joint*>::iterator joint_it;  
  //Skip first joint since it's transform is constant
  for (joint_it = ++joint_container_.begin(); joint_it != joint_container_.end(); ++joint_it)
  {
    Joint* joint = joint_it->second;
    const Link* reference_link = joint->reference_link;
    joint->transform = createDHMatrix(reference_link->offset,
				      reference_link->actuating_joint->desired_position,
				      reference_link->length,
				      reference_link->twist); 
    cout << joint->transform << endl;
  } 
  const Link* reference_link = tip_->reference_link;
  tip_->transform = createDHMatrix(reference_link->offset,
				   reference_link->actuating_joint->desired_position,
				   reference_link->length,
				   reference_link->twist); 
}

/***********************************************************************************************************************
 * Joint data object
***********************************************************************************************************************/
Joint::Joint(Leg* leg, Link* link, int id, Parameters* params)
  : parent_leg(leg)
  , reference_link(link)
  , id_number(id)
  , name(leg->getIDName() + "_" + params->joint_id.data[id_number-1] + "_joint")
  , position_offset(params->joint_parameters[leg->getIDNumber()][id_number-1].data["offset"])
  , min_position(params->joint_parameters[leg->getIDNumber()][id_number-1].data["min"])
  , max_position(params->joint_parameters[leg->getIDNumber()][id_number-1].data["max"])
  , packed_position(params->joint_parameters[leg->getIDNumber()][id_number-1].data["packed"])
  , unpacked_position(params->joint_parameters[leg->getIDNumber()][id_number-1].data["unpacked"])
  , max_angular_speed(params->joint_parameters[leg->getIDNumber()][id_number-1].data["max_vel"])
{
  if (params->joint_parameters[leg->getIDNumber()][id_number-1].initialised)
  {
    transform = createDHMatrix(reference_link->offset,
			       reference_link->angle,
			       reference_link->length,
			       reference_link->twist);
    ROS_DEBUG("%s has been initialised with parameters:"
	    "offset: %f, min: %f, max: %f, packed: %f, unpacked: %f, max_vel: %f.",
	    name.c_str(), position_offset, min_position, max_position,
	    packed_position, unpacked_position, max_angular_speed);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", name.c_str());
    ros::shutdown();
  }  
}

/***********************************************************************************************************************
 * Link data object
***********************************************************************************************************************/
Link::Link(Leg* leg, Joint* joint, int id, Parameters* params)
  : parent_leg(leg)
  , actuating_joint(joint)
  , id_number(id)
  , name(leg->getIDName() + "_" + params->link_id.data[id_number] + "_link")
  , length(params->link_parameters[leg->getIDNumber()][id_number].data["r"])
  , angle(params->link_parameters[leg->getIDNumber()][id_number].data["theta"])
  , offset(params->link_parameters[leg->getIDNumber()][id_number].data["d"])
  , twist(params->link_parameters[leg->getIDNumber()][id_number].data["alpha"])
{   
  if (params->link_parameters[leg->getIDNumber()][id_number].initialised)
  {
    ROS_DEBUG("%s has been initialised with DH parameters: d: %f, theta: %f, r: %f, alpha: %f.",
	    name.c_str(), offset, angle, length, twist);
  }
  else
  {
    ROS_FATAL("Model initialisation error for %s", name.c_str());
    ros::shutdown();
  }  
}

/***********************************************************************************************************************
 * Tip data object
***********************************************************************************************************************/
Tip::Tip(Leg* leg, Link* link)
  : parent_leg(leg)
  , reference_link(link)
  , name(leg->getIDName() + "_tip")  
{    
  transform = createDHMatrix(reference_link->offset,
			     reference_link->angle,
			     reference_link->length,
			     reference_link->twist);
}

/***********************************************************************************************************************
***********************************************************************************************************************/
