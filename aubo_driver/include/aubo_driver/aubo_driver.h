#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <ros/ros.h>
#include <mutex>
#include <iostream>
#include <vector>
#include <string>
#include "aubo_driver/AuboRobotMetaType.h"
#include "aubo_driver/serviceinterface.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "otg/otgnewslib.h"

namespace aubo_driver
{
    struct RobotParam
    {
        std::string server_host_;
        int server_port_;
        int robot_state_hz_;
        std::vector<std::string> joint_names_;
        int num_joints_;
    };

    class AuboDriver
    {
        public:
            AuboDriver(ros::NodeHandle nh, RobotParam params);
            ~AuboDriver();

            ServiceInterface robot_send_service_;          

        private:
            
            bool connectToRobotController(); 
            void updateRobotStateCallback(const ros::TimerEvent& e);

            ServiceInterface robot_receive_service_; 

            ros::NodeHandle nh_;
            ros::Publisher joint_states_pub_;
            ros::Subscriber command_sub_;
            ros::Timer robot_state_timer_;       

            RobotParam params_;

    };
}

#endif /* AUBO_DRIVER_H_ */
