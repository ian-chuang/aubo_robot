#ifndef XS_HARDWARE_INTERFACE_OBJ_H
#define XS_HARDWARE_INTERFACE_OBJ_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mutex>

class AuboHW : public hardware_interface::RobotHW
{
public:
  AuboHW(ros::NodeHandle &nh);
  void update(const ros::TimerEvent &e);
  void read();
  void write(ros::Duration elapsed_time);
  void joint_state_cb(const sensor_msgs::JointState &msg);
  void wait_for_joint_states();
  void register_interfaces();

private:
  ros::NodeHandle nh;
  int loop_hz;
  std::vector<std::string> joint_names;
  unsigned int num_joints;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  joint_limits_interface::PositionJointSaturationInterface jnt_pos_saturation_interface;

  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;
  std::vector<double> cmd_prev;

  std::mutex js_mutex;
  sensor_msgs::JointState joint_states;
  ros::Subscriber sub_joint_states;
  ros::Publisher pub_joint_positions;

  ros::Timer tmr_control_loop;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;
};

#endif