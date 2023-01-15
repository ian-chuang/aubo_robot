#include <aubo_ros_control/aubo_hardware_interface_obj.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_hardware_interface");
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner(2);
  AuboHW RobotInterface(nh);
  spinner.spin();
  return 0;
}