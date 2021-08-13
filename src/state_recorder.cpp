#include "baxter_roscpp_interface/state_recorder.h"

namespace baxter_interface
{

StateRecorder::StateRecorder(const ros::NodeHandle &n) :
  node_handle_(n)
{
  // Initialize joint names
  // Left arm joint names
  joint_names_.push_back("left_s0");
  joint_names_.push_back("left_s1");
  joint_names_.push_back("left_e0");
  joint_names_.push_back("left_e1");
  joint_names_.push_back("left_w0");
  joint_names_.push_back("left_w1");
  joint_names_.push_back("left_w2");

  // Right arm joint names
  joint_names_.push_back("right_s0");
  joint_names_.push_back("right_s1");
  joint_names_.push_back("right_e0");
  joint_names_.push_back("right_e1");
  joint_names_.push_back("right_w0");
  joint_names_.push_back("right_w1");
  joint_names_.push_back("right_w2");

  // Initialize mapping of joint names and corresponding fields
  // Left arm joint mapping
  joint_map_.insert(std::pair<std::string, int>("left_s0", 0));
  joint_map_.insert(std::pair<std::string, int>("left_s1", 1));
  joint_map_.insert(std::pair<std::string, int>("left_e0", 2));
  joint_map_.insert(std::pair<std::string, int>("left_e1", 3));
  joint_map_.insert(std::pair<std::string, int>("left_w0", 4));
  joint_map_.insert(std::pair<std::string, int>("left_w1", 5));
  joint_map_.insert(std::pair<std::string, int>("left_w2", 6));

  // Right arm joint mapping
  joint_map_.insert(std::pair<std::string, int>("right_s0", 7));
  joint_map_.insert(std::pair<std::string, int>("right_s1", 8));
  joint_map_.insert(std::pair<std::string, int>("right_e0", 9));
  joint_map_.insert(std::pair<std::string, int>("right_e1", 10));
  joint_map_.insert(std::pair<std::string, int>("right_w0", 11));
  joint_map_.insert(std::pair<std::string, int>("right_w1", 12));
  joint_map_.insert(std::pair<std::string, int>("right_w2", 13));

  log_file_name_format_ = "robot_state_Y%YM%mD%d_T%H%M%S.csv";

  init_failed_ = false;

  // Initialize log folder directory
  res_dir_ = boost::filesystem::path(BAXTER_INTERFACE_LOGGING_DIR);

  // Initialize log file path
  file_path_ = (res_dir_ / "state_recordings").string() + "/" +
               log_file_name_format_;

  // Resize joint_angles_ vector
  joint_angles_.resize(14,0);

  l_gripper_state_ = -1.0;
  r_gripper_state_ = -1.0;

  baxter_interface::Gripper l_gripper_interface(node_handle_, "left");
  baxter_interface::Gripper r_gripper_interface(node_handle_, "right");

  // Calibrate grippers in case they are not calibrated
  if(!l_gripper_interface.checkIfCalibrated())
  {
    if(!l_gripper_interface.calibrateGripper(true))
    {
      init_failed_ = true;
    }
  }

  if(!r_gripper_interface.checkIfCalibrated())
  {
    if(!r_gripper_interface.calibrateGripper(true))
    {
      init_failed_ = true;
    }
  }
}

bool StateRecorder::startRecording(void)
{
  std::time_t now = std::time(0);
  std::tm* timestamp = std::localtime(&now);

  if(init_failed_)
  {
    ROS_WARN("Initialization failed!");
    return false;
  }

  // Initialize log file name
  char log_file_path_char[250];
  strftime(log_file_path_char, 250, file_path_.c_str(), timestamp);
  log_file_path_ = std::string(log_file_path_char);

  // Open log file
  state_log_file_.open(log_file_path_, std::ofstream::out | std::ofstream::app);

  if(state_log_file_.is_open())
  {
    ROS_INFO_STREAM("Recording robot state to " << log_file_path_);
  }
  else
  {
    ROS_INFO_STREAM("Error opening file");
    ROS_INFO_STREAM("Cannot create file " << log_file_path_);
    return false;
  }

  start_time_ = ros::Time::now().toNSec();

  joint_states_sub_ = node_handle_.subscribe("/robot/joint_states", 1,
      &StateRecorder::jointStatesTopicCallback, this);

  l_gripper_state_sub_ = node_handle_.subscribe(
      "/robot/end_effector/left_gripper/state", 1,
      &StateRecorder::leftEndEffectorStateTopicCallback, this);

  r_gripper_state_sub_ = node_handle_.subscribe(
      "/robot/end_effector/right_gripper/state", 1,
      &StateRecorder::rightEndEffectorStateTopicCallback, this);

  return true;
}

bool StateRecorder::startRecording(std::string f_name)
{ 

  if(init_failed_)
  {
    ROS_WARN("Initialization failed!");
    return false;
  }

  // Initialize log file name
  log_file_path_ = (res_dir_ / "state_recordings").string() + "/" + f_name;

  // Open log file
  state_log_file_.open(log_file_path_, std::ofstream::out | std::ofstream::app);

  if(state_log_file_.is_open())
  {
    ROS_INFO_STREAM("Recording robot state to " << log_file_path_);
  }
  else
  {
    ROS_INFO_STREAM("Error opening file");
    ROS_INFO_STREAM("Cannot create file " << log_file_path_);
    return false;
  }

  start_time_ = ros::Time::now().toNSec();

  joint_states_sub_ = node_handle_.subscribe("/robot/joint_states", 1,
      &StateRecorder::jointStatesTopicCallback, this);

  l_gripper_state_sub_ = node_handle_.subscribe(
      "/robot/end_effector/left_gripper/state", 1,
      &StateRecorder::leftEndEffectorStateTopicCallback, this);

  r_gripper_state_sub_ = node_handle_.subscribe(
      "/robot/end_effector/right_gripper/state", 1,
      &StateRecorder::rightEndEffectorStateTopicCallback, this);

  return true;
}

bool StateRecorder::stopRecording(void)
{
  joint_states_sub_.shutdown();
  l_gripper_state_sub_.shutdown();
  r_gripper_state_sub_.shutdown();

  state_log_file_.close();

  return true;
}

void StateRecorder::jointStatesTopicCallback(const sensor_msgs::JointState &msg)
{
  std::map<std::string, int>::iterator map_itr;

  if(msg.name.size() > 6)
  {
    for(int itr = 0; itr < msg.name.size(); itr++)
    {
      map_itr = joint_map_.find(msg.name[itr]);
      if(map_itr != joint_map_.end())
      {
        joint_angles_[joint_map_[msg.name[itr]]] = msg.position[itr];
      }
    }

    // Record robot states
    // Time in nano seconds since start of recording
    state_log_file_ << msg.header.stamp.toNSec() - start_time_;
    state_log_file_ << ",";

    // Left arm state
    for(int itr = 0; itr < 7; itr++)
    {
      state_log_file_ << joint_angles_[itr] << ",";
    }
    state_log_file_ << l_gripper_state_ << ",";

    // Right arm state
    for(int itr = 7; itr < 14; itr++)
    {
      state_log_file_ << joint_angles_[itr] << ",";
    }
    state_log_file_ << r_gripper_state_ << "\n";

    state_log_file_.flush();
  }
}

void StateRecorder::leftEndEffectorStateTopicCallback(
    const baxter_core_msgs::EndEffectorState &state_msg)
{
  l_gripper_state_ = state_msg.position;
}

void StateRecorder::rightEndEffectorStateTopicCallback(
    const baxter_core_msgs::EndEffectorState &state_msg)
{
  r_gripper_state_ = state_msg.position;
}

}
