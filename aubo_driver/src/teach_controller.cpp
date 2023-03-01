#include "aubo_driver/aubo_driver.h"
#include "aubo_msgs/TeachCommand.h"

using namespace aubo_driver;


class TeachController {
  public:

    TeachController(ros::NodeHandle& nh, AuboDriver& driver) : nh_(nh), driver_(driver) {
      teach_sub_ = nh_.subscribe("teach", 1, &TeachController::teachCallback, this);
    }

  private:

    void teachCallback(const aubo_msgs::TeachCommand::ConstPtr &msg) {
      bool direction = msg->direction == aubo_msgs::TeachCommand::POSITIVE;

      switch(msg->command) {
        case aubo_msgs::TeachCommand::MOV_X:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::MOV_X, direction);
          break;
        case aubo_msgs::TeachCommand::MOV_Y:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::MOV_Y, direction);
          break;
        case aubo_msgs::TeachCommand::MOV_Z:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::MOV_Z, direction);
          break;
        case aubo_msgs::TeachCommand::ROT_X:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::ROT_X, direction);
          break;
        case aubo_msgs::TeachCommand::ROT_Y:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::ROT_Y, direction);
          break;
        case aubo_msgs::TeachCommand::ROT_Z:
          driver_.robot_send_service_.robotServiceTeachStart(aubo_robot_namespace::teach_mode::ROT_Z, direction);
          break;
        case aubo_msgs::TeachCommand::STOP:
          driver_.robot_send_service_.robotServiceTeachStop();
          break;
        default:
          // stop if incorrect input
          driver_.robot_send_service_.robotServiceTeachStop();
      }
    }

    AuboDriver& driver_;
    ros::NodeHandle& nh_;
    ros::Subscriber teach_sub_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_driver");  
    ros::NodeHandle nh;

    RobotParam params;
    params.server_host_ = "192.168.0.197";
    params.server_port_ = 8899;
    params.robot_state_hz_ = 200;
    params.num_joints_ = 6;
    params.joint_names_ = {
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    };  

    AuboDriver driver(nh, params);

    driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(1);
    driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(1);
    driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(0.4);
    driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(0.4);

    aubo_robot_namespace::JointVelcAccParam joint_param;
    for (int i = 0; i < aubo_robot_namespace::ARM_DOF; i++) {
      joint_param.jointPara[i] = 1;
    }

    driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(joint_param);
    driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(joint_param);
    
    TeachController teach_controller(nh, driver);

    ros::spin();
    
    return 0;
}


