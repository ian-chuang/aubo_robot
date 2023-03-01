#include "aubo_driver/aubo_driver.h"

namespace aubo_driver {

AuboDriver::AuboDriver(ros::NodeHandle nh, RobotParam params)
{
    nh_ = nh;
    params_ = params;

    bool ret = connectToRobotController();
    if (ret == false) {
        ROS_WARN("[Aubo Driver] Failed to start driver");
        return;
    }
    
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("aubo_driver/joint_states", 1);
    robot_state_timer_ = nh_.createTimer(ros::Duration(1.0 / params_.robot_state_hz_), &AuboDriver::updateRobotStateCallback, this);
    robot_state_timer_.start();   
}

AuboDriver::~AuboDriver()
{
    robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
}

void AuboDriver::updateRobotStateCallback(const ros::TimerEvent& e)
{

    aubo_robot_namespace::wayPoint_S waypoint;
    int ret1 = robot_receive_service_.robotServiceGetCurrentWaypointInfo(waypoint);
    if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode) 
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(params_.num_joints_);
        joint_state.position.resize(params_.num_joints_);
        for(int i = 0; i < params_.num_joints_; i++)
        {
            joint_state.name[i] = params_.joint_names_[i];
            joint_state.position[i] = waypoint.jointpos[i];
        }
        joint_states_pub_.publish(joint_state);
    }
    else {
        ROS_WARN("[Aubo Driver] Failed to read joint positions");
    }
}

bool AuboDriver::connectToRobotController()
{

    int ret1 = robot_send_service_.robotServiceLogin(
        params_.server_host_.c_str(), 
        params_.server_port_, 
        "aubo", 
        "123456"
    );
    if (ret1 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Can't login to robot");
        return false;
    } 

    int ret2 = robot_receive_service_.robotServiceLogin(
        params_.server_host_.c_str(), 
        params_.server_port_, 
        "aubo", 
        "123456"
    );
    if (ret2 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Can't login to robot");
        return false;
    } 

    return true;
}


}

