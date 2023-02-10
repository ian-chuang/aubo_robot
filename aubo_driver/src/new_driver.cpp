#include "aubo_driver/aubo_driver.h"

namespace aubo_driver {

AuboDriver::AuboDriver() :
{
    bool ret = init();
    if (ret == false) {
        return;
    }
    
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/aubo_driver/joint_states", 1);
    moveit_controller_subs_ = nh_.subscribe("moveItController_cmd", 1, &AuboDriver::moveItPosCallback,this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_), &AuboDriver::timerCallback, this);
    timer_.start();
}

AuboDriver::~AuboDriver()
{
    robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
}

void AuboDriver::updateRobotStateCallback(const ros::TimerEvent& e)
{
    int ret1 = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
    if (ret1 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Failed to read joint positions")
        return;
    }

    for (int i = 0; i < num_joints; i++)
    {
        joint_positions.at(i) = rs.wayPoint_.jointpos[i];
    }

    int ret2 = robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
    if (ret2 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Failed to get robot diagnostic info")
        return;
    }

    rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;
}

void AuboDriver::moveItPosCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)   
{
    while (rib_buffer_size_ >= MINIMUM_BUFFER_SIZE) {
        //pass
    }

    robot_send_service_.robotServiceSetRobotPosData2Canbus(msg->data.data());   
}



bool AuboDriver::connectToRobotController()
{
    int ret1 = robot_send_service_.robotServiceLogin(
        robot_param_.server_host_.c_str(), 
        robot_param_.server_port_, 
        "aubo", 
        "123456"
    );
    if (ret1 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Can't login to robot")
        return false;
    } 

    int ret2 = robot_receive_service_.robotServiceLogin(
        robot_param_.server_host_.c_str(), 
        robot_param_.server_port_, 
        "aubo", 
        "123456"
    );
    if (ret2 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Can't login to robot")
        return false;
    } 

    bool real_robot_exist;
    int ret3 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
    if (ret3 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_WARN("[Aubo Driver] Error calling robotServiceGetIsRealRobotExist")
        return false;
    }
    if (!real_robot_exist) {
        ROS_WARN("[Aubo Driver] Real robot doesn't exist")
        return false;
    }  

    return true
}

bool AuboDriver::init()
{
    int max_retry_times = 5;

    bool ret1 = connectToRobotController();
    for (int i = 0; i < max_link_times && !ret1; i++) {
        ret1 = connectToRobotController();
    }

    if (!ret1) {
        ROS_ERROR("[Aubo Driver] Failed to connect to Aubo and start driver")
        return false;
    }

    int ret2 = robot_send_service_.robotServiceEnterTcp2CanbusMode();
    for (int i = 0; i < max_link_times && ret2 != aubo_robot_namespace::InterfaceCallSuccCode; i++) {
        // already connected, disconnect first.
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        ret2 = robot_send_service_.robotServiceEnterTcp2CanbusMode();
    }

    if (ret2 != aubo_robot_namespace::InterfaceCallSuccCode) {
        ROS_ERROR("[Aubo Driver] Failed to switch from TCP to CAN Bus mode")
        return false;
    }

    ROS_INFO("[Aubo Driver] Successfully started driver")
    return true;
}


}

