#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mutex>

#define AUBO_DOF 6

class AuboHW : public hardware_interface::RobotHW
{
public:
  AuboHW(ros::NodeHandle &nh) : nh(nh)
  {
    int loop_hz = 200;
    std::vector<std::string> joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    pub_joint_positions = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/moveItController_cmd", 100);
    sub_joint_states = nh.subscribe("/aubo_driver/joint_states", 1, &AuboHW::joint_state_cb, this);

    cmd.resize(AUBO_DOF);
    pos.resize(AUBO_DOF);
    vel.resize(AUBO_DOF);
    eff.resize(AUBO_DOF);
    cmd_prev.resize(AUBO_DOF);

    ros::Rate loop_rate(loop_hz);
    while (joint_states.position.size() == 0 && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    // Initialize the joint_position_commands vector to the current joint states
    for (size_t i{0}; i < AUBO_DOF; i++)
    {
      cmd.at(i) = joint_states.position.at(i);
      cmd_prev.at(i) = cmd.at(i);
    }

    for (int i = 0; i < AUBO_DOF; i++)
    {
      // connect and register the joint state interface
      std::cout << joint_names[i] << std::endl;
      hardware_interface::JointStateHandle state_handle(joint_names.at(i), &pos.at(i), &vel.at(i), &eff.at(i));
      jnt_state_interface.registerHandle(state_handle);

      // connect and register the joint position interface
      hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names.at(i)), &cmd.at(i));
      jnt_pos_interface.registerHandle(pos_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);

    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz);
    tmr_control_loop = nh.createTimer(update_freq, &AuboHW::update, this);
  }

  void update(const ros::TimerEvent &e)
  {
    ros::Duration elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write();
  }

  void read()
  {
    js_mutex.lock();
    for (int i = 0; i < AUBO_DOF; i++)
    {
      pos.at(i) = joint_states.position.at(i);
      // vel.at(i) = 0;
      // eff.at(i) = 0;
    }
    js_mutex.unlock();
  }

  void write()
  {
    trajectory_msgs::JointTrajectoryPoint msg;

    for (int i = 0; i < AUBO_DOF; i++)
    {
      msg.positions.push_back(cmd.at(i));
      msg.velocities.push_back(0);
      msg.accelerations.push_back(0);
    }

    if (cmd_prev != msg.positions)
    {
      pub_joint_positions.publish(msg);
      cmd_prev = msg.positions;
    }
  }

  void joint_state_cb(const sensor_msgs::JointState &msg)
  {
    js_mutex.lock();
    joint_states = msg;
    js_mutex.unlock();
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;
  std::vector<double> cmd_prev;

  ros::NodeHandle nh;
  ros::Subscriber sub_joint_states;
  ros::Publisher pub_joint_positions;

  std::mutex js_mutex;
  sensor_msgs::JointState joint_states;
  ros::Timer tmr_control_loop;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_hardware_interface");
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner(2);
  AuboHW RobotInterface(nh);
  spinner.spin();
  return 0;
}