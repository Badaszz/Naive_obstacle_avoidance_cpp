// includes
#include "cpp_basics_project/robot_interface_class.hpp"

// class constructor
RobotInterface::RobotInterface() : Node("robot_interface") {
  RCLCPP_INFO(this->get_logger(), "Initializing Robot Interface ...");

  // declare and initialize cmd_vel publisher
  cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", 10);
  RCLCPP_INFO(this->get_logger(), "Initialized Publisher: /cmd_vel");

  // declare and initialize callback group
  callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // declare and initialize scan subscriber
  rclcpp::QoS scan_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  scan_sub_options.callback_group = callback_group;
  scan_sub = this->create_subscription<LaserScan>(
      "/scan", scan_sub_qos,
      std::bind(&RobotInterface::scan_callback, this, std::placeholders::_1),
      scan_sub_options);
  RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /scan");

  // declare and initialize odom subscriber
  rclcpp::QoS odom_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  odom_sub_options.callback_group = callback_group;
  odom_sub = this->create_subscription<Odometry>(
      "/odom", odom_sub_qos,
      std::bind(&RobotInterface::odom_callback, this, std::placeholders::_1),
      odom_sub_options);
  RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /odom");

  // declare and initialize imu subscriber
  rclcpp::QoS imu_sub_qos(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
  imu_sub_options.callback_group = callback_group;
  imu_sub = this->create_subscription<Imu>(
      "/imu", imu_sub_qos,
      std::bind(&RobotInterface::imu_callback, this, std::placeholders::_1),
      imu_sub_options);
  RCLCPP_INFO(this->get_logger(), "Initialized Subscriber: /imu");

  // declare and initialize control timer callback
  control_timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RobotInterface::control_callback, this), callback_group);
  RCLCPP_INFO(this->get_logger(), "Initialized Control Timer");

  RCLCPP_INFO(this->get_logger(), "Robot Interface Initialized !");
}

// class destructor
RobotInterface::~RobotInterface() {
  // indicate robot interface node termination
  RCLCPP_INFO(this->get_logger(), "Terminating Robot Interface ...");
  RCLCPP_INFO(this->get_logger(), "Robot Interface Terminated !");
}

// class methods and callbacks

void RobotInterface::scan_callback(const LaserScan::SharedPtr scan_msg) {
  // simple method to get scan data
  scan_angle_min = scan_msg->angle_min;
  scan_angle_max = scan_msg->angle_max;
  scan_angle_increment = scan_msg->angle_increment;
  scan_range_min = scan_msg->range_min;
  scan_range_max = scan_msg->range_max;
  scan_ranges.clear();
  scan_ranges.assign(scan_msg->ranges.begin(), scan_msg->ranges.end());
}

void RobotInterface::odom_callback(const Odometry::SharedPtr odom_msg) {
  // simple method to get odom data
  odom_position_x = odom_msg->pose.pose.position.x;
  odom_position_y = odom_msg->pose.pose.position.y;
  odom_position_z = odom_msg->pose.pose.position.z;
  angles = euler_from_quaternion(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
  odom_orientation_r = angles["r"];
  odom_orientation_p = angles["p"];
  odom_orientation_y = angles["y"];
}

void RobotInterface::imu_callback(const Imu::SharedPtr imu_msg) {
  // simple method to get imu data
  imu_angular_velocity_x = imu_msg->angular_velocity.x;
  imu_angular_velocity_y = imu_msg->angular_velocity.y;
  imu_angular_velocity_z = imu_msg->angular_velocity.z;
  imu_linear_acceleration_x = imu_msg->linear_acceleration.x;
  imu_linear_acceleration_y = imu_msg->linear_acceleration.y;
  imu_linear_acceleration_z = imu_msg->linear_acceleration.z;
}

void RobotInterface::control_callback() {
  // set robot speeds
  twist_cmd.linear.x = linear_velocity;
  twist_cmd.angular.z = angular_velocity;
  // publish the twist command
  publish_twist_cmd();
  // print debug
  // RCLCPP_INFO(
  //     this->get_logger(),
  //     "lin_vel: %+0.3f, ang_vel: %+0.3f, scan_angle_min: %+0.3f, "
  //     "scan_angle_max: %+0.3f, scan_angle_inc: %+0.3f, scan_range_min: "
  //     "%+0.3f, scan_range_max: %+0.3f, odom_px: %+0.3f, odom_py: %+0.3f, "
  //     "odom_pz: %+0.3f, odom_or: %+0.3f, odom_op: %+0.3f, odom_oy: %+0.3f, "
  //     "imu_ang_vel_x: %+0.3f, imu_ang_vel_y: %+0.3f, imu_ang_vel_z: %+0.3f, "
  //     "imu_lin_acc_x: %+0.3f, imu_lin_acc_y: %+0.3f, imu_lin_acc_z: %+0.3f",
  //     twist_cmd.linear.x, twist_cmd.angular.y, scan_angle_min,
  //     scan_angle_max, scan_angle_increment, scan_range_min, scan_range_max,
  //     odom_position_x, odom_position_y, odom_position_z, odom_orientation_r,
  //     odom_orientation_p, odom_orientation_y, imu_angular_velocity_x,
  //     imu_angular_velocity_y, imu_angular_velocity_z,
  //     imu_linear_acceleration_x, imu_linear_acceleration_y,
  //     imu_linear_acceleration_z);
}

void RobotInterface::publish_twist_cmd() {
  // linear speed control
  twist_cmd.linear.x = std::min(twist_cmd.linear.x, 0.150);
  // angular speed control
  twist_cmd.angular.z = std::min(twist_cmd.angular.z, 0.450);
  // publish command
  cmd_vel_pub->publish(twist_cmd);
}

std::map<std::string, double>
RobotInterface::euler_from_quaternion(double quat_x, double quat_y,
                                      double quat_z, double quat_w) {
  // function to convert quaternions to euler angles
  // calculate roll
  double sinr_cosp = 2 * (quat_w * quat_x + quat_y * quat_z);
  double cosr_cosp = 1 - 2 * (quat_x * quat_x + quat_y * quat_y);
  double roll = atan2(sinr_cosp, cosr_cosp);
  // calculate pitch
  double sinp = 2 * (quat_w * quat_y - quat_z * quat_x);
  double pitch = asin(sinp);
  // calculate yaw
  double siny_cosp = 2 * (quat_w * quat_z + quat_x * quat_y);
  double cosy_cosp = 1 - 2 * (quat_y * quat_y + quat_z * quat_z);
  double yaw = atan2(siny_cosp, cosy_cosp);
  // store the angle values in a map
  std::map<std::string, double> angles;
  angles["r"] = roll;
  angles["p"] = pitch;
  angles["y"] = yaw;
  // return the angle values
  return angles;
}

// End of Code
