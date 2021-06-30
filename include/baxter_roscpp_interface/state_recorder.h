/*
 * Joint recorder class to record the Baxter state such as joint angles, 
 * IO button states, gripper state to a csv file
 */

#pragma once

#include "baxter_roscpp_interface/limb.h"
#include "baxter_roscpp_interface/gripper.h"
#include "baxter_roscpp_interface/logging_resources.h"
#include <fstream>
#include <ctime>
#include <boost/filesystem/path.hpp>

namespace baxter_interface
{
  class StateRecorder
  {
    public:
      /**
       * @brief   Constructor to initialize the Baxter StateRecorder object
       *
       * @param   n           Node handle
       */
      StateRecorder(const ros::NodeHandle &n);

      /**
       * @brief   Starts recording the robot state for the corresponding limb
       *
       * @return  Returns status of record start request
       */
      bool startRecording(void);

      /**
       * @brief   Stops recording the joint angles for the corresponding limb
       *
       * @return  Retursn status of record stop request
       */
      bool stopRecording(void);

    private:

      ros::NodeHandle node_handle_;

      /**
       * @brief   Log file handle
       */
      std::ofstream state_log_file_;

      /**
       * @brief   Log resources directory
       */
      boost::filesystem::path res_dir_;

      /**
       * @brief   File name format
       */
      std::string log_file_name_format_;

      /**
       * @brief   Log file path structure
       */
      std::string file_path_;

      /**
       * @brief   Log file path
       */
      std::string log_file_path_;

      /**
       * @brief   Flag set true when initialization fails
       */
      bool init_failed_;

      /**
       * @brief   Current joint angles
       */
      std::vector<double> joint_angles_;

      /**
       * @brief   Joint names
       */
      std::vector<std::string> joint_names_;

      /**
       * @brief   Mapping of joint names to corresponding fields in log file
       */
      std::map<std::string, int> joint_map_;

      /**
       * @brief   Left gripper control interface
       */
      baxter_interface::Gripper l_gripper_;

      /**
       * @brief   Right gripper control interface
       */
      baxter_interface::Gripper r_gripper_;

      /**
       * @brief   Current state of left gripper
       */
      float l_gripper_state_;

      /**
       * @brief   Current state of right gripper
       */
      float r_gripper_state_;

      /**
       * @brief   Subscriber handle to the Joint States topic
      */
      ros::Subscriber joint_states_sub_;

      /**
       * @brief   Subscriber handle to the left gripper state topic
       */
      ros::Subscriber l_gripper_state_sub_;

      /**
       * @brief   Subscriber handle to the right gripper state topic
       */
      ros::Subscriber r_gripper_state_sub_;

      /**
       * @brief   Subscriber initialized
       */
      bool subs_initialized_;

      /**
       * @brief   Joint states topic callback
       *
       * @param   msg       Message published on joint states topic
       */
      void jointStatesTopicCallback(const sensor_msgs::JointState &msg);

      /**
       * @brief   Callback for left end effector properties subscriber
       */
      void leftEndEffectorStateTopicCallback(
          const baxter_core_msgs::EndEffectorState &prop_msg);

      /**
       * @brief   Callback for right end effector properties subscriber
       */
      void rightEndEffectorStateTopicCallback(
          const baxter_core_msgs::EndEffectorState &prop_msg);
  };
}
