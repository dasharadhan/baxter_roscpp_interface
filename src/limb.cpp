#include "baxter_roscpp_interface/limb.h"

#define PI 3.14159265358979

bool baxter_interface::Limb::left_limb_object_exists_flag_ = false;
bool baxter_interface::Limb::right_limb_object_exists_flag_ = false;

baxter_interface::Limb::Limb(const ros::NodeHandle &n, std::string limb_side) :
  node_handle_(n), limb_side_(limb_side)
{
  assert((limb_side_ == "left") || (limb_side_ == "right"));

  joint_names_.clear();
  joint_angles_.clear();
  joint_torques_.clear();

  topic_ns_ = "/robot/limb/" + limb_side_ + "/";

  if(limb_side_ == "left")
  {
    if(left_limb_object_exists_flag_)
    {
      ROS_WARN("Multiple initializations of \"Limb\" object for \"left\"");
    }

    left_limb_object_exists_flag_ = true;

    joint_names_.push_back("left_s0");
    joint_names_.push_back("left_s1");
    joint_names_.push_back("left_e0");
    joint_names_.push_back("left_e1");
    joint_names_.push_back("left_w0");
    joint_names_.push_back("left_w1");
    joint_names_.push_back("left_w2");
  }
  else if(limb_side_ == "right")
  {
    if(right_limb_object_exists_flag_)
    {
      ROS_WARN("Multiple initializations of \"Limb\" object for \"right\"");
    }

    right_limb_object_exists_flag_ = true;

    joint_names_.push_back("right_s0");
    joint_names_.push_back("right_s1");
    joint_names_.push_back("right_e0");
    joint_names_.push_back("right_e1");
    joint_names_.push_back("right_w0");
    joint_names_.push_back("right_w1");
    joint_names_.push_back("right_w2");
  }

  for(int itr = 0; itr < 7; itr++)
  {
    joint_angles_.push_back(0.0);
    joint_torques_.push_back(0.0);
  }

  subscription_flag_ = false;

  joint_states_sub_ = node_handle_.subscribe("/robot/joint_states", 1, 
      &baxter_interface::Limb::jointStatesTopicCallback, this);

  joint_command_pub_ = node_handle_.advertise<baxter_core_msgs::JointCommand>(
      topic_ns_ + "joint_command", 1);
}

void baxter_interface::Limb::jointStatesTopicCallback(
    const sensor_msgs::JointState &msg)
{
  int val_cnt = 0;

  if(msg.name.size() < 6)
  {
    return;
  }
  else
  {
    for(int itr = 0; itr < msg.name.size(); itr++)
    {
      for(int joint_itr = 0; joint_itr < 7; joint_itr++)
      {
        try
        {
          if(msg.name[itr] == joint_names_[joint_itr])
          {
            joint_angles_[joint_itr] = msg.position[itr];
            joint_torques_[joint_itr] = msg.effort[itr];
            val_cnt++;
            break;
          }
        }
        catch(...)
        {
          ROS_ERROR("\"/robot/joint_states\" callback exception!");
          return;
        }
      }

      if(val_cnt >= 7)
      {
        subscription_flag_ = true;
        return;
      }
    }
  }
}

std::vector<double> baxter_interface::Limb::getJointAngles(void)
{
  return joint_angles_;
}

std::vector<double> baxter_interface::Limb::getJointTorques(void)
{
  return joint_torques_;
}

void baxter_interface::Limb::setJointPositions(
    std::vector<double> cmd_angles, bool cmd_mode)
{
  baxter_core_msgs::JointCommand cmd_msg;

  cmd_msg.names.clear();
  cmd_msg.command.clear();

  for(int msg_itr = 0; msg_itr < 7; msg_itr++)
  {
    cmd_msg.names.push_back(joint_names_[msg_itr]);
    cmd_msg.command.push_back(cmd_angles[msg_itr]);
  }

  if(cmd_mode)
    cmd_msg.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
  else
    cmd_msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

  joint_command_pub_.publish(cmd_msg);
}

void baxter_interface::Limb::moveToJointPositions(
    std::vector<double> &cmd_angles, double timeout, double tolerance)
{
  new_joint_angles_.clear();
  commanded_joint_angles_.clear();

  ros::Rate rate(100);

  while(!ros::Time::now().isValid())
  {

  }

  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + ros::Duration(timeout);

  while(!subscription_flag_)
  {
    ros::Duration(1).sleep();

    if(ros::Time::now() > end_time)
    {
      ROS_ERROR("Nothing is being published to /robot/joint_states/");
      return;
    }
  }

  for(int cmd_itr = 0; cmd_itr < 7; cmd_itr++)
  {
    new_joint_angles_.push_back(joint_angles_[cmd_itr]);
    commanded_joint_angles_.push_back(cmd_angles[cmd_itr]);
  }

  start_time = ros::Time::now();
  end_time = start_time + ros::Duration(timeout);

  while((jointAngleDiff(commanded_joint_angles_, tolerance)) && ros::ok())
  {
    setJointPositions(filteredCmd());
    
    rate.sleep();

    if(ros::Time::now() > end_time)
    {
      ROS_ERROR("\"baxter_interface::Limb::moveToJointPositions\" timeout!");
    }
  }
    
}

baxter_interface::ErrorCodes baxter_interface::Limb::executeTrajectory(
  trajectory_msgs::JointTrajectory &jnt_trajectory)
{
  double start_time;
  double max_val;
  std::vector<double> cmd_angles;
  std::vector<double> pre_angles;
  std::vector<double> current_joint_angles;

  // Move to initial pose
  moveToJointPositions(jnt_trajectory.points[0].positions);
  
  for(unsigned int i = 1; i < jnt_trajectory.points.size(); i++)
  {
    if(!ros::ok())
    {
      return baxter_interface::ErrorCodes::OPERATION_CANCELLED;
    }
    
    max_val = 0;
    
    pre_angles = jnt_trajectory.points[i-1].positions;
    cmd_angles = jnt_trajectory.points[i].positions;
    
    for(unsigned int j = 0; j < 7; j++)
    {
      if(abs(cmd_angles[j]-pre_angles[j]) > max_val)
      {
        max_val = abs(cmd_angles[j]-pre_angles[j]);
      }
    }
    
    // Safety check for large joint value differences
    if(max_val > (5*PI/180))
    {
      ROS_INFO_STREAM("Large difference in joint values!");
      moveToJointPositions(cmd_angles);
      continue;
    }
    
    setJointPositions(cmd_angles);
    ROS_INFO_STREAM("Trajectory position : " << i);
    
    ros::Duration(0.001).sleep();
    
    start_time = ros::Time::now().toSec();
    bool angle_tolerance_met = false;
    
    while ((ros::Time::now().toSec() - start_time) < 2)
    {
      current_joint_angles = joint_angles_;
      unsigned int k = 0;
      for (k = 0; k < 7; k++)
      {
        if ((current_joint_angles[k] - cmd_angles[k]) > JOINT_ANGLE_TOLERANCE)
        //if ((current_joint_angles[k] - cmd_angles[k]) > 0.009)
        {
          //continue;
          goto control_joint_positions;
        }
      }

      if (k == 7)
      {
        angle_tolerance_met = true;
        break;
      }

control_joint_positions:
      setJointPositions(cmd_angles);
      ros::Duration(0.001).sleep();
    }

    if (angle_tolerance_met == false)
    {
      double max_diff = 0;

      for (int itr = 0; itr < 7; itr++)
      {
        if (abs(current_joint_angles[itr] - cmd_angles[itr]) > max_diff)
        {
          max_diff = abs(current_joint_angles[itr] - cmd_angles[itr]);
        }
      }

      ROS_INFO_STREAM("max_diff = " << max_diff);

      if (max_diff > 0.025)
      {
        ROS_INFO("Angle tolerance not met!");
        ROS_INFO("Exiting Node!");

        return baxter_interface::ErrorCodes::JOINT_TOLERANCE_NOT_MET;
      }
    }
  }

  return baxter_interface::ErrorCodes::OPERATION_SUCCESS;
}

std::vector<double> baxter_interface::Limb::filteredCmd(void)
{
  for(int cmd_itr = 0; cmd_itr < 7; cmd_itr++)
  {
    new_joint_angles_[cmd_itr] = (0.012488 * commanded_joint_angles_[cmd_itr]) 
      + (0.98751 * new_joint_angles_[cmd_itr]);
  }

  return new_joint_angles_;
}

bool baxter_interface::Limb::jointAngleDiff(
    const std::vector<double> &cmd_angles, const double &tolerance)
{ 
  for(int cmd_itr = 0; cmd_itr < 7; cmd_itr++)
  {
    double temp = joint_angles_[cmd_itr] - cmd_angles[cmd_itr];

    if(temp < 0)
      temp = -temp;

    if(temp <= tolerance)
      continue;
    else
      return true;
  }

  return false;
}
