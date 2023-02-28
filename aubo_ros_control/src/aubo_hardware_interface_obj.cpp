#include <aubo_ros_control/aubo_hardware_interface_obj.h>

AuboHW::AuboHW(ros::NodeHandle &nh) : nh(nh)
{
  loop_hz = 200;
  joint_names.push_back("shoulder_pan_joint");
  joint_names.push_back("shoulder_lift_joint");
  joint_names.push_back("elbow_joint");
  joint_names.push_back("wrist_1_joint");
  joint_names.push_back("wrist_2_joint");
  joint_names.push_back("wrist_3_joint");
  num_joints = joint_names.size();

  cmd.resize(num_joints);
  pos.resize(num_joints);
  vel.resize(num_joints);
  eff.resize(num_joints);
  cmd_prev.resize(num_joints);

  pub_joint_positions = nh.advertise<std_msgs::Float64MultiArray>("aubo_driver/command", 1);
  sub_joint_states = nh.subscribe("/aubo_driver/joint_states", 1, &AuboHW::joint_state_cb, this);

  wait_for_joint_states();
  register_interfaces();

  controller_manager.reset(new controller_manager::ControllerManager(this, nh));

  // start control loop
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz);
  tmr_control_loop = nh.createTimer(update_freq, &AuboHW::update, this);
}

void AuboHW::wait_for_joint_states() {
  // Wait for joint states to be published
  ros::Rate loop_rate(loop_hz);
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize the cmd vector to the current joint states
  for (size_t i{0}; i < num_joints; i++)
  {
    cmd.at(i) = joint_states.position.at(i);
    cmd_prev.at(i) = cmd.at(i);
  }
}

void AuboHW::register_interfaces() {
  urdf::Model model;
  urdf::JointConstSharedPtr ptr;
  model.initParam("/robot_description");

  for (int i = 0; i < num_joints; i++)
  {
    // get joint limits
    joint_limits_interface::JointLimits limits;
    ptr = model.getJoint(joint_names.at(i));
    getJointLimits(ptr, limits);
    getJointLimits(joint_names.at(i), nh, limits);

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names.at(i), &pos.at(i), &vel.at(i), &eff.at(i));
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names.at(i)), &cmd.at(i));
    jnt_pos_interface.registerHandle(pos_handle);

    // connect and register the joint position saturation interface
    joint_limits_interface::PositionJointSaturationHandle jointPositionSaturationHandle(pos_handle, limits);
    jnt_pos_saturation_interface.registerHandle(jointPositionSaturationHandle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_pos_saturation_interface);
}

void AuboHW::update(const ros::TimerEvent &e)
{
  ros::Duration elapsed_time = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager->update(ros::Time::now(), elapsed_time);
  write(elapsed_time);
}

void AuboHW::read()
{
  js_mutex.lock();
  for (int i = 0; i < num_joints; i++)
  {
    pos.at(i) = joint_states.position.at(i);
  }
  js_mutex.unlock();
}

void AuboHW::write(ros::Duration elapsed_time)
{
  jnt_pos_saturation_interface.enforceLimits(elapsed_time);

  std_msgs::Float64MultiArray msg;
  for (int i = 0; i < num_joints; i++)
  {
    msg.data.push_back(cmd.at(i));
  }

  if (cmd_prev != msg.data)
  {
    pub_joint_positions.publish(msg);
    cmd_prev = msg.data;
  }
}

void AuboHW::joint_state_cb(const sensor_msgs::JointState &msg)
{
  js_mutex.lock();
  joint_states = msg;
  js_mutex.unlock();
}

