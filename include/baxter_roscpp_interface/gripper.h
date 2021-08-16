/*
 *  Interface to control Baxter parallel jaw gripper through roscpp nodes
 */

#ifndef BAXTER_INTERFACE_GRIPPER_H
#define BAXTER_INTERFACE_GRIPPER_H

#include "ros/ros.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/EndEffectorProperties.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_roscpp_interface/settings.h"
#include <vector>
#include <string>
#include <iostream>

namespace baxter_interface
{
  class Gripper
  {
    public:
      /*
        \brief  Constructor to initialize the gripper interface

        \param  nh            Node handle
        \param  gripper_side  The gripper to which the interface corresponds to
      */
      Gripper(const ros::NodeHandle &nh, std::string gripper_side);

      /*
        \brief  Checks if the gripper has been calibrated
      */
      bool checkIfCalibrated(void);

      /*
        \brief  Clears the gripper calibration

        \param  block       Set 'True' to block execution until calibration
                            is cleared
        \param  timeout     Maximum allowed time to wait for the process to 
                            complete
      */
      bool clearCalibration(bool block, double timeout = 5);

      /*
        \brief  Resets the gripper

        \param  block       Set 'True' to block execution until gripper is
                            reset
        \param  timeout     Maximum allowed time to wait for the process to 
                            complete
      */
      bool resetGripper(bool block, double timeout = 5);

      /*
        \brief  Performs gripper calibration

        \param  block       Set 'True' to block execution until calibration
                            is performed
        \param  timeout     Maximum allowed time to wait for the process to 
                            complete
      */
      bool calibrateGripper(bool block, double timeout = 5);
      
      /*
        \brief  Returns current gripper position
      */
      double getGripperPosition(void);
      
      /*
        \brief  Returns false gripper is open or true if close
        
        \param  pos_threshold   Gripper position below which the gripper is 
                                considered to be closed
      */
      bool isGripperClosed(double close_threshold = 15);

      /*
        \brief  Closes the gripper jaws till object is grasped

        \param  block       Set 'True' to block execution until gripper is 
                            closed
        \param  timeout     Maximum allowed time to wait for the process to 
                            complete
      */
      bool closeGripper(bool block, double timeout = 5);

      /*
        \brief  Opens the gripper jaws

        \param  block       Set 'True' to block execution until gripper is
                            opened
        \param  timeout     Maximum allowed time to wait for the process to 
                            complete
      */
      bool openGripper(bool block, double timeout = 5);

    private:

      /*
        \brief  The side to which the gripper corresponds to
      */
      std::string gripper_side_;

      /*
        \brief  The name of the gripper
      */
      std::string gripper_name_;

      /*
        \brief  Namespace of the gripper node handle
      */
      std::string ns_;

      /*
        \brief  Node handle for the gripper interface
      */
      ros::NodeHandle nh_;

      /*
        \brief  Handle for the gripper state topic subscriber
      */
      ros::Subscriber gripper_state_sub_;

      /*
        \brief  Handle for the gripper property topic subscriber
      */
      ros::Subscriber gripper_prop_sub_;

      /*
        \brief  Handle for the gripper command publisher
      */
      ros::Publisher gripper_cmd_pub_;

      /*
        \brief  Handle for the gripper property publisher
      */
      ros::Publisher gripper_prop_pub_;

      /*
        \brief  Handle for the gripper state publisher
      */
      ros::Publisher gripper_state_pub_;

      baxter_core_msgs::EndEffectorState state_;
      baxter_core_msgs::EndEffectorProperties prop_;

      /*
        \brief  Command sequence
      */
      uint32_t cmd_sequence_;

      /*
        \brief  Flag set on failure of gripper initialization
      */
      bool init_failed_;

      /*
        \brief  Callback for end effector properties subscriber
      */
      void endEffectorPropertiesTopicCallback(
          const baxter_core_msgs::EndEffectorProperties &prop_msg);

      /*
        \brief  Callback for end effector state subscriber
      */
      void endEffectorStateTopicCallback(
          const baxter_core_msgs::EndEffectorState &state_msg);

      /*
        \brief  Clips values to the range [0-100]

        \param  val   Value to clip
      */
      unsigned int clip(unsigned int val);

      /*
        \brief  Increments the command sequence number
      */
      void incrementCmdSequence(void);
  };
}

#endif
