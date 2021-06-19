/*
 *  Interface to control Baxter limbs through roscpp nodes
 */

#ifndef BAXTER_INTERFACE_LIMB_H
#define BAXTER_INTERFACE_LIMB_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_roscpp_interface/settings.h"
#include <vector>
#include <string>
#include <iostream>

namespace baxter_interface
{
  typedef enum{ OPERATION_SUCCESS, 
              OPERATION_FAILURE,
              OPERATION_CANCELLED,
              JOINT_TOLERANCE_NOT_MET,
              JOINT_LIMIT_ERROR,
              PARAMETER_ERROR }ErrorCodes;

  class Limb
  {
    public:
      /*!
        \brief    Constructor to initialize the Baxter Limb object

        \param    n             Node handle
        \param    limb_side     Which limb to interface (left or right)
      */
      Limb(const ros::NodeHandle &n, std::string limb_side);

      /*!
        \brief    Returns the current joint angles of the limb
      */
      std::vector<double> getJointAngles(void);

      /*!
        \brief    Returns the current joint torques of the limb
      */
      std::vector<double> getJointTorques(void);

      /*!
        \brief    Commands the limb to the provided positions

        \details  Moves the limb to the commanded position and ensures that
                  the joint positions have been attained. Blocks program 
                  execution until position is attained. This command smoothes
                  the movement using a low pass filter.

        \param    cmd_angles    Commanded joint positions
        \param    timeout       Seconds to wait before command timeout
        \param    tolerance     Joint angle tolerances
      */
      void moveToJointPositions(
          std::vector<double> &cmd_angles, 
          double timeout = 15,
          double tolerance = JOINT_ANGLE_TOLERANCE);

      /*!
        \brief    Commands the limb to the provided positions

        \details  Moves the limb to the commanded position. Does not block 
                  program execution or check if position has been attained.

        \param    cmd_angles    Commanded joint positions
        \param    cmd_mode      Setting this parameter to true uses the
                                'Raw Position Mode' and bypasses the Joint 
                                Controller Boards (JCBs). This results in more
                                unaffected motions while also bypassing the 
                                safety system modifications such as
                                collision avoidance.
      */
      void setJointPositions( std::vector<double> cmd_angles, 
                              bool cmd_mode = false);
      
      /*!
        \brief    Executes a given joint trajectory

        \details  Given a joint trajectory, this command executes it while
                  ensuring that the joint positions are attained within a 
                  certain tolerance

        \param    jnt_trajectory    Joint trajectory to execute
      */
      ErrorCodes executeTrajectory(
          trajectory_msgs::JointTrajectory &jnt_trajectory);

    private:
      /*!
        \brief    Stores the names of the joints for the corresponding limb
      */
      std::vector<std::string> joint_names_;

      /*!
        \brief    Stores the current joint positions for the corresponding limb
      */
      std::vector<double> joint_angles_;

      /*!
        \brief    Stores the current joint torques for the corresponding limb
      */
      std::vector<double> joint_torques_;

      /*!
        \brief    Temporary variable to store the filtered command
      */
      std::vector<double> new_joint_angles_;

      /*!
        \brief    Stores the commanded joint positions for the corresponding limb
      */
      std::vector<double> commanded_joint_angles_;

      /*!
        \brief    Stores the limb to which the interface corresponds to
      */
      std::string limb_side_;

      std::string topic_ns_;
      bool subscription_flag_;

      static bool left_limb_object_exists_flag_;
      static bool right_limb_object_exists_flag_;
      
      ros::NodeHandle node_handle_;

      /*!
        \brief    Subscriber handle to the Joint States topic
      */
      ros::Subscriber joint_states_sub_;

      /*!
        \brief    Publisher handle to the Joint Command topic
      */
      ros::Publisher joint_command_pub_;

      /*!
        \brief    Callback function for the Joint States topic Subscriber
      */
      void jointStatesTopicCallback(const sensor_msgs::JointState &msg);
      
      /*!
        \brief    Filters the commanded joint positions to smoothen the movement
      */
      std::vector<double> filteredCmd(void);

      /*!
        \brief    Checks if the current joint positions are within a specified
                  tolerance from the commanded joint positions

        \param    cmd_angles    Commanded joint positions
        \param    tolerance     Allowable tolerance
      */
      bool jointAngleDiff(const std::vector<double> &cmd_angles, 
                          const double &tolerance);
  };
}

#endif
