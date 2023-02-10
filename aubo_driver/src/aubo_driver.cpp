/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "aubo_driver/aubo_driver.h"

namespace aubo_driver {

std::string AuboDriver::joint_name_[ARM_DOF] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

AuboDriver::AuboDriver(int num = 0) :delay_clear_times(0),buffer_size_(400),io_flag_delay_(0.02),data_recieved_(false),data_count_(0),real_robot_exist_(false),emergency_stopped_(false),protective_stopped_(false),normal_stopped_(false),
    controller_connected_flag_(false),start_move_(false),control_mode_ (aubo_driver::SendTargetGoal),rib_buffer_size_(0),jti(ARM_DOF,1.0/200),jto(ARM_DOF),collision_class_(6)
{
    rs.robot_controller_ = ROBOT_CONTROLLER;

    /** publish messages **/
    // joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/aubo_driver/joint_states", 300);
    target_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/aubo_driver/target_joint_states", 300);
    robot_status_pub_ = nh_.advertise<industrial_msgs::RobotStatus>("/aubo_driver/robot_status", 100);
    io_pub_ = nh_.advertise<aubo_msgs::IOState>("/aubo_driver/io_states", 10);


    /** subscribe topics **/
    joint_command_subs_ = nh_.subscribe("aubo_driver/command", 100, &AuboDriver::moveItPosCallback,this);


}

AuboDriver::~AuboDriver()
{
    /** leave Tcp2CanbusMode, surrender the control to the robot-controller**/
    if(control_option_ == aubo_driver::RosMoveIt)
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    /** log out　**/
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
}

void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
    if(controller_connected_flag_)
    {


        
        /** Query the states of robot joints **/
        int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);      /** this method upates the joint states more quickly **/
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[ARM_DOF];
            for(int i = 0; i < 6; i++)
                joints[i] = rs.wayPoint_.jointpos[i];
            setCurrentPosition(joints);  // update the current robot joint states to ROS Controller

            /** Get the buff size of thr rib **/
            robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
            rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;

            if (rib_buffer_size_ != 0) { std::cout << rib_buffer_size_ << std::endl; }

//            robot_receive_service_.robotServiceGetRobotCurrentState(rs.state_);            // this is controlled by Robot Controller
//            robot_receive_service_.getErrDescByCode(rs.code_);
            if(real_robot_exist_)
            {
                // publish robot_status information to the controller action server.
                robot_status_.mode.val            = (int8)rs.robot_diagnosis_info_.orpeStatus;
                robot_status_.e_stopped.val       = (int8)(rs.robot_diagnosis_info_.softEmergency || emergency_stopped_);
                robot_status_.drives_powered.val  = (int8)rs.robot_diagnosis_info_.armPowerStatus;
                robot_status_.motion_possible.val = (int)(!start_move_);
                robot_status_.in_motion.val       = (int)start_move_;
                robot_status_.in_error.val        = (int)protective_stopped_;   //used for protective stop.
                robot_status_.error_code          = (int32)rs.robot_diagnosis_info_.singularityOverSpeedAlarm;
            }
        }
        else if(ret == aubo_robot_namespace::ErrCode_SocketDisconnect)
        {
            /** Here we check the connection to satisfy the ROS-I specification **/
            /** Try to connect with the robot again **/
            if(!connectToRobotController())
            {
                ROS_ERROR("Cann't connect to the robot controller!");
            }
        }

        //publish the rib_status to the controller simulator
        rib_status_.data[0] = buf_queue_.size();
        rib_status_.data[1] = control_mode_;
        rib_status_.data[2] = controller_connected_flag_;
    }
    else
    {
        /** maintain the ros-controller states from the ros environment **/
        setCurrentPosition(target_point_);      //return back immediately

        ROS_INFO("SETTING CURRENT POSITION TO BE TARGET POINT");
    }

    robot_status_pub_.publish(robot_status_);
    rib_pub_.publish(rib_status_);

    if(control_mode_ == aubo_driver::SynchronizeWithRealRobot /*|| rs.robot_controller == ROBOT_CONTROLLER*/)
    {
        if(controller_connected_flag_)
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(axis_number_);
            joint_state.position.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                joint_state.name[i] = joint_name_[i];
                joint_state.position[i] = current_joints_[i];
            }
            //            joint_states_pub_.publish(joint_state);

            memcpy(last_recieve_point_, current_joints_, sizeof(double) * axis_number_);
            memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
        }
        else
        {
            ROS_INFO("There is no to the robot controller!");
        }
    }
    else if(control_mode_ == aubo_driver::SendTargetGoal)
    {
        sensor_msgs::JointState joint_state;
        control_msgs::FollowJointTrajectoryFeedback joint_feedback;

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(axis_number_);
        joint_feedback.joint_names.resize(axis_number_);
        joint_state.position.resize(axis_number_);
        joint_feedback.actual.positions.resize(axis_number_);
        for(int i = 0; i<axis_number_; i++)
        {
            joint_state.name[i] = joint_name_[i];
            if(controller_connected_flag_)
                joint_state.position[i] = current_joints_[i];
            else
                joint_state.position[i] = target_point_[i];

            joint_feedback.joint_names[i] = joint_name_[i];
            joint_feedback.actual.positions[i] = joint_state.position[i];
        }
        joint_states_pub_.publish(joint_state);
        joint_feedback_pub_.publish(joint_feedback);

        /** If the controller is robot-controller, then synchronize the ros-controller states **/
        if(control_option_ == aubo_driver::AuboAPI)
        {
            memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
            std_msgs::Float32MultiArray joints;
            joints.data.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                joints.data[i] = target_point_[i];
            }
            joint_target_pub_.publish(joints);
        }
    }
}



double* AuboDriver::getCurrentPosition()
{
    return current_joints_;
} 

void AuboDriver::setCurrentPosition(double *target)
{
    for(int i = 0; i < axis_number_;i++)
    {
        current_joints_[i] = target[i];
    }
}

double* AuboDriver::getTagrtPosition()
{
    return target_point_;
}

void AuboDriver::setTagrtPosition(double *target)
{
    for(int i = 0; i < axis_number_;i++)
    {
        target_point_[i] = target[i];
    }
}

bool AuboDriver::setRobotJointsByMoveIt()
{
    int ret = 0;
    // First check if the buf_queue_ is Empty
    if(!buf_queue_.empty())
    {
        PlanningState ps = buf_queue_.front();
        buf_queue_.pop();     

        if(controller_connected_flag_)      // actually no need this judgment
        {
            if(emergency_stopped_)
            {
                //clear the buffer, there will be a jerk
                start_move_ = false;
                while(!buf_queue_.empty())
                    buf_queue_.pop();
            }
            else if(protective_stopped_ || normal_stopped_)
            {
                //cancle.data will be set 0 in the aubo_robot_simulator.py when clear this one trajectory data

                // std_msgs::UInt8 cancle;
                // cancle.data = 1;
                // cancle_trajectory_pub_.publish(cancle);

                //first slow down, until the velocity to 0.
                // memcpy(&jti.currentPosition[0], ps.joint_pos_, axis_number_*sizeof(double));
                // memcpy(&jti.currentVelocity[0], ps.joint_vel_, axis_number_*sizeof(double));
                // memcpy(&jti.currentAcceleration[0], ps.joint_acc_, axis_number_*sizeof(double));
                // memset(&jti.targetVelocity[0], 0, axis_number_*sizeof(double));
                // bool update = otgVelocityModeParameterUpdate(jti);
                // int resultValue = 0;
                // while(resultValue != 1)
                // {
                //    resultValue = otgVelocityModeResult(1, jto);
                //    double jointAngle[] = {jto.newPosition[0],jto.newPosition[1],jto.newPosition[2],jto.newPosition[3],jto.newPosition[4],jto.newPosition[5]};
                //    ROS_INFO("WHAT THE PROTECT STOP?\n\n");
                //    ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(jointAngle);
                   //std::cout<<jointAngle[0]<<","<<jointAngle[1]<<","<<jointAngle[2]<<","<<jointAngle[3]<<","<<jointAngle[4]<<","<<jointAngle[5]<<","<<std::endl;

                // }
                //clear the buffer
                // start_move_ = false;
                // while(!buf_queue_.empty())
                //     buf_queue_.pop();

                //clear the flag
                // if(normal_stopped_)
                // {
                //     normal_stopped_ = false;
                //     delay_clear_times = STOP_DELAY_CLEAR_TIMES;
                // }
            }
            else
            {
                // ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(ps.joint_pos_);
                // std::cout << "SENDING: " << ps.joint_pos_[0]<<","<<ps.joint_pos_[1]<<","<<ps.joint_pos_[2]<<","<<ps.joint_pos_[3]<<","<<ps.joint_pos_[4]<<","<<ps.joint_pos_[5]<<std::endl;
            }
#ifdef LOG_INFO_DEBUG
            //            struct timeb tb;
            //            ftime(&tb);
            //            std::cout<<tb.millitm<<std::endl;
            //std::cout<<ps.joint_pos_[0]<<","<<ps.joint_pos_[1]<<","<<ps.joint_pos_[2]<<","<<ps.joint_pos_[3]<<","<<ps.joint_pos_[4]<<","<<ps.joint_pos_[5]<<std::endl;
#endif
        }
        setTagrtPosition(ps.joint_pos_);
    }
    else
    {
        // if(start_move_)
        //     start_move_ = false;
    }
}


void AuboDriver::moveItPosCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    double jointAngle[axis_number_];
    for(int i = 0; i < axis_number_; i++)
        jointAngle[i] = msg->positions[i];

    if(!emergency_stopped_ && controller_connected_flag_ && control_option_ == aubo_driver::RosMoveIt)
    {

        while (rib_buffer_size_ >= MINIMUM_BUFFER_SIZE) {
            //pass
        }

        /** The need a buffer to connect to the RIB to confirm the REAL TIME**/
        if(roadPointCompare(jointAngle, last_recieve_point_))
        {
            data_count_ = 0;
            PlanningState ps;

            memcpy(ps.joint_pos_, jointAngle, sizeof(double) * axis_number_);
            memcpy(ps.joint_vel_, &msg->velocities[0], sizeof(double) * axis_number_);
            memcpy(ps.joint_acc_, &msg->accelerations[0], sizeof(double) * axis_number_);
            memcpy(last_recieve_point_, jointAngle, sizeof(double) * axis_number_);
            robot_send_service_.robotServiceSetRobotPosData2Canbus(ps.joint_pos_);


            std::cout<<jointAngle[0]<<","<<jointAngle[1]<<","<<jointAngle[2]<<","<<jointAngle[3]<<","<<jointAngle[4]<<","<<jointAngle[5]<<","<<std::endl;

            

            start_move_ = true; // set to false if no publishers??

            // rib_buffer_size_ = 0;
        }
        else {
            ROS_INFO("ROADPOINTCOMPARE FALSE");
        }
    }
    else
    {
        ROS_INFO("CANCEL NEW WAYPOINT, %d, %d, %d, %d, %d", !emergency_stopped_, controller_connected_flag_, rib_buffer_size_ < MINIMUM_BUFFER_SIZE, control_option_ == aubo_driver::RosMoveIt, rib_buffer_size_);

        setTagrtPosition(jointAngle);
        rib_buffer_size_ = 0;
        start_move_ = false;
    }
}


bool AuboDriver::connectToRobotController()
{
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;

    std::string s;
    ros::param::get("/aubo_driver/server_host", s); //The server_host should be corresponding to the robot controller setup.
    server_host_ = (s=="")? "192.168.0.198" : s;
    std::cout<<"server_host:"<<server_host_<<std::endl;

    /** log in ***/
    int max_link_times = 5;
    int count = 0;
    do {
        count++;
        ret1 = robot_send_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
    }while(ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);

    if(ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        ret2 = robot_receive_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
        controller_connected_flag_  = true;
        std::cout<<"login success."<<std::endl;
        /** 接口调用: 获取真实臂是否存在 **/
        ret2 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
        if(ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            (real_robot_exist_)? std::cout<<"real robot exist."<<std::endl:std::cout<<"real robot doesn't exist."<<std::endl;
            //power on the robot.
        }
//        ros::param::set("/aubo_driver/robot_connected","1");
        return true;
    }
    else
    {
//        ros::param::set("/aubo_driver/robot_connected","0");
        controller_connected_flag_  = false;
        std::cout<<"login failed."<<std::endl;
        return false;
    }
}

void AuboDriver::run()
{
    ROS_INFO("Start the driver!");

    /** connect to the robot controller **/
    if(connectToRobotController())
    {
        /** Switches to ros-controller **/
        int ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_INFO("Switches to ros-controller successfully");
            control_option_ = aubo_driver::RosMoveIt;
        }
        else if(ret == aubo_robot_namespace::ErrCode_ResponseReturnError)
        {
            //already connect, disconnect first.
            ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
            ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_INFO("Switches to ros-controller successfully");
                control_option_ = aubo_driver::RosMoveIt;
            }
            else
            {
                control_option_ = aubo_driver::AuboAPI;
                ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
            }
        }
        else
        {
            control_option_ = aubo_driver::AuboAPI;
            ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
        }

        ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[8];
            for(int i = 0; i < 6; i++)
                joints[i] = rs.wayPoint_.jointpos[i];

            setCurrentPosition(joints);
            setTagrtPosition(joints);
            //send this information to the controller simulator to initialize the position
            std_msgs::Float32MultiArray robot_joints;
            robot_joints.data.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                robot_joints.data[i] = current_joints_[i];
            }
            /** If the driver node launched after the robot_simulator node, this will initialize the joint_positions **/
            joint_target_pub_.publish(robot_joints);
             /** If the driver node launched after the robot_simulator node, this will initialize the joint_positions **/
            ros::param::set("initial_joint_state", joints);
        }
    }

    ros::param::set("/aubo_driver/robot_connected","1");

    //communication Timer between ros node and real robot controller.
    timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_), &AuboDriver::timerCallback, this);
    timer_.start();

    /** get the io states of the robot **/
    mb_publish_thread_ = new std::thread(boost::bind(&AuboDriver::publishIOMsg, this));
}




}

