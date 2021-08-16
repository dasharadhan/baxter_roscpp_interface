#include "baxter_roscpp_interface/gripper.h"

baxter_interface::Gripper::Gripper(
    const ros::NodeHandle &nh, std::string gripper_side) :
    nh_(nh),
    gripper_side_(gripper_side)
{
  ns_ = "robot/end_effector/" + gripper_side_ + "_gripper/";

  cmd_sequence_ = 0;

  gripper_cmd_pub_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>(
                        ns_ + "command", 10);

  gripper_prop_pub_ = nh_.advertise<baxter_core_msgs::EndEffectorProperties>(
                          ns_ + "rsdk/set_propertise", 10, true);

  gripper_state_pub_ = nh_.advertise<baxter_core_msgs::EndEffectorState>(
                          ns_ + "rsdk/set_state", 10, true);

  gripper_prop_sub_ = nh_.subscribe(ns_ + "properties", 1, 
      &baxter_interface::Gripper::endEffectorPropertiesTopicCallback, this);

  gripper_state_sub_ = nh_.subscribe(ns_ + "state", 1,
      &baxter_interface::Gripper::endEffectorStateTopicCallback, this);
      
  while(!ros::Time::now().isValid())
  {

  }

  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + ros::Duration(10);

  init_failed_ = true;

  while(ros::Time::now() < end_time)
  {
    ros::spinOnce();
    
    if(state_.enabled == true)
    {
      init_failed_ = false;
      break;
    }

    ros::Duration(0.1).sleep();
    
    ROS_INFO_STREAM("Waiting for Gripper Initialization!");
  }

  if(init_failed_)
  {
    ROS_INFO_STREAM("Gripper Initialization Failed!");
  }
}

void baxter_interface::Gripper::endEffectorPropertiesTopicCallback(
    const baxter_core_msgs::EndEffectorProperties &prop_msg)
{
  prop_ = prop_msg;
}

void baxter_interface::Gripper::endEffectorStateTopicCallback(
    const baxter_core_msgs::EndEffectorState &state_msg)
{
  state_ = state_msg;
}

bool baxter_interface::Gripper::checkIfCalibrated(void)
{
  return state_.calibrated;
}

bool baxter_interface::Gripper::clearCalibration(bool block, double timeout)
{
  if(prop_.ui_type != baxter_core_msgs::EndEffectorProperties::ELECTRIC_GRIPPER)
  {
    return false;
  }

  baxter_core_msgs::EndEffectorCommand ee_cmd;

  ee_cmd.id = prop_.id;
  ee_cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CLEAR_CALIBRATION;

  incrementCmdSequence();
  gripper_cmd_pub_.publish(ee_cmd);

  if(block)
  {
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);

    while(ros::Time::now() <= end_time)
    {
      if((!state_.calibrated) && state_.ready)
      {
        return true;
      }

      ros::Duration(0.1).sleep();
    }
  }

  return false;
}

bool baxter_interface::Gripper::resetGripper(bool block, double timeout)
{
  if(prop_.ui_type != baxter_core_msgs::EndEffectorProperties::ELECTRIC_GRIPPER)
  {
    return false;
  }

  baxter_core_msgs::EndEffectorCommand ee_cmd;

  ee_cmd.id = prop_.id;
  ee_cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RESET;

  incrementCmdSequence();
  gripper_cmd_pub_.publish(ee_cmd);

  if(block)
  {
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);

    while(ros::Time::now() <= end_time)
    {
      if((!state_.error) && state_.ready)
      {
        return true;
      }

      ros::Duration(0.1).sleep();
    }
  }

  return false;
}

bool baxter_interface::Gripper::calibrateGripper(bool block, double timeout)
{
  bool op_success = false;

  if(prop_.ui_type == baxter_core_msgs::EndEffectorProperties::ELECTRIC_GRIPPER)
  {
    if(state_.calibrated)
    {
      op_success = false;
      op_success = clearCalibration(block, timeout);
      if(!block)
      {
        ros::Duration(5).sleep();
      }
    }

    if(state_.error)
    {
      op_success = false;
      op_success = resetGripper(block, timeout);
      if(!block)
      {
        ros::Duration(1).sleep();
      }
    }

    if(state_.error || (!state_.ready))
    {
      return false;
    }
      
    baxter_core_msgs::EndEffectorCommand ee_cmd;

    ee_cmd.id = prop_.id;
    ee_cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;

    incrementCmdSequence();
    gripper_cmd_pub_.publish(ee_cmd);

    if(block)
    {
      ros::Time start_time = ros::Time::now();
      ros::Time end_time = start_time + ros::Duration(timeout);

      while(ros::Time::now() <= end_time)
      {
        if(state_.calibrated && state_.ready)
        {
          return true;
        }

        ros::Duration(0.1).sleep();
      }
    }
  }

  return false;
}

double baxter_interface::Gripper::getGripperPosition(void)
{
  return state_.position;
}

bool baxter_interface::Gripper::isGripperClosed(double close_threshold)
{
  if(state_.position <= close_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool baxter_interface::Gripper::closeGripper(bool block, double timeout)
{
  if(prop_.ui_type != baxter_core_msgs::EndEffectorProperties::ELECTRIC_GRIPPER)
  {
    return false;
  }

  bool op_success = true;

  if(state_.error || (!state_.calibrated))
  {
    op_success = calibrateGripper(block, timeout);
  }

  if((!op_success) || (!state_.ready))
  {
    return false;
  }

  baxter_core_msgs::EndEffectorCommand ee_cmd;

  std::string arguments = "{\"position\": 0.0}";

  ee_cmd.id = prop_.id;
  ee_cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  ee_cmd.args = arguments;

  incrementCmdSequence();
  gripper_cmd_pub_.publish(ee_cmd);

  if(block)
  {
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);

    while(ros::Time::now() <= end_time)
    {
      if((state_.position <= 5.0) || state_.gripping)
      {
        return true;
      }

      ros::Duration(0.1).sleep();
    }
  }

  return false;
}

bool baxter_interface::Gripper::openGripper(bool block, double timeout)
{
  if(prop_.ui_type != baxter_core_msgs::EndEffectorProperties::ELECTRIC_GRIPPER)
  {
    return false;
  }

  bool op_success = true;

  if(state_.error || (!state_.calibrated))
  {
    op_success = calibrateGripper(block, timeout);
  }

  if((!op_success) || (!state_.ready))
  {
    return false;
  }

  baxter_core_msgs::EndEffectorCommand ee_cmd;

  std::string arguments = "{\"position\": 100.0}";

  ee_cmd.id = prop_.id;
  ee_cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
  ee_cmd.args = arguments;

  incrementCmdSequence();
  gripper_cmd_pub_.publish(ee_cmd);

  if(block)
  {
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);

    while(ros::Time::now() <= end_time)
    {
      if((state_.position >= 95.0) && state_.ready)
      {
        return true;
      }

      ros::Duration(0.1).sleep();
    }
  }

  return false;
}

unsigned int baxter_interface::Gripper::clip(unsigned int val)
{
  if(val > 100)
  {
    return 100;
  }
  else
  {
    return val;
  }
}

void baxter_interface::Gripper::incrementCmdSequence(void)
{
  cmd_sequence_++;
}
