#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <thread>
#include <string>
#include <sys/timeb.h>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetPayload.h>
#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>
#include <aubo_msgs/GetFK.h>
#include <aubo_msgs/GetFKRequest.h>
#include <aubo_msgs/GetFKResponse.h>
#include <aubo_msgs/GetIK.h>
#include <aubo_msgs/GetIKRequest.h>
#include <aubo_msgs/GetIKResponse.h>
#include <aubo_msgs/IOState.h>
#include <aubo_msgs/Digital.h>
#include <aubo_msgs/Analog.h>
#include <aubo_msgs/JointPos.h>
#include <industrial_msgs/RobotStatus.h>
#include "aubo_driver/AuboRobotMetaType.h"
#include "aubo_driver/serviceinterface.h"
#include "sensor_msgs/JointState.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <fstream>
#include "otg/otgnewslib.h"

namespace aubo_driver
{
    struct RobotParam
    {
        int collision_class_;
        std::string server_host_;
        int server_port_;
        int minimum_buffer_size_;
        std::vector<std::string> joint_names_;
        int num_joints_;
    };

    struct RobotState
    {
        int rib_buffer_size_;
        int control_mode_;
        bool emergency_stopped_;
        bool reverse_connected_;
    };

    class AuboDriver
    {
        public:
            AuboDriver(int num);
            ~AuboDriver();


            void run();
            bool connectToRobotController();

        public:

            ServiceInterface robot_send_service_;      //send
            ServiceInterface robot_receive_service_;     //receive

            RobotState robot_state_;
            RobotParam robot_param_;

            ros::Publisher joint_states_pub_;

        private:
            void moveItPosCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);
            

            
            double last_recieve_point_[ARM_DOF];   /** To avoid joining the same waypoint to the queue **/
            int control_option_;
            




            ros::NodeHandle nh_;
            
            ros::Timer timer_;

    };
}

#endif /* AUBO_DRIVER_H_ */
