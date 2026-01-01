// includes
// ros2 rclcpp includes
#include "rclcpp/executors.hpp"                         // executors
#include "rclcpp/executors/multi_threaded_executor.hpp" // multithreaded executor
#include "rclcpp/node.hpp"                              // base node
// header includes
#include "cpp_basics_project/robot_interface_class.hpp" // robot interface class
// standard cpp includes
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// instantiate robot interface program as a global variable
std::shared_ptr<RobotInterface> robot_interface;

//~//~// start your function definitions after this line //~//~//

void sample_move(double linear, double angular) {
  // sample function to move the robot
  // this function can be deleted later!
  // access global variable robot_interface
  robot_interface->linear_velocity = linear;
  robot_interface->angular_velocity = angular;
}

// Robot Movements

std::vector<double> get_lin_ang_velocity() {
  return {robot_interface->linear_velocity, robot_interface->angular_velocity};
}

void stop_robot() {
  robot_interface->linear_velocity = 0;
  robot_interface->angular_velocity = 0;
}

void move_forward(double lin_speed) {
  robot_interface->linear_velocity = lin_speed;
  robot_interface->angular_velocity = 0;
}

void move_backward(double lin_speed) {
  robot_interface->linear_velocity = -lin_speed;
  robot_interface->angular_velocity = 0;
}

void turn_left(double ang_speed) {
  robot_interface->angular_velocity = ang_speed;
  robot_interface->linear_velocity = 0;
}

void turn_right(double ang_speed) {
  robot_interface->angular_velocity = -ang_speed;
  robot_interface->linear_velocity = 0;
}

void timed_move_front(double speed, double time) {
  move_forward(speed);
  std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
  stop_robot();
}

void timed_move_back(double speed, double time) {
  move_backward(speed);
  std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
  stop_robot();
}

void timed_turn_left(double ang_speed, double time) {
  turn_left(ang_speed);
  std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
  stop_robot();
}

// void timed_turn_left(double ang_speed, double time) {
//   turn_left(ang_speed);
//   std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
//   stop_robot();
// }

void timed_turn_right(double ang_speed, double time) {
  turn_right(ang_speed);
  std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
  stop_robot();
}

void move_distance_front(double distance, double speed) {
  timed_move_front(speed, distance / speed);
}

void move_distance_back(double distance, double speed) {
  timed_move_back(speed, distance / speed);
}

void turn_angle_left(double ang_speed, double angle) {
  timed_turn_left(ang_speed, angle / ang_speed);
}

void turn_angle_right(double ang_speed, double angle) {
  timed_turn_right(ang_speed, angle / ang_speed);
}

// Laser Scanner functions

float get_min_scan_angle() { return robot_interface->scan_angle_min; }

float get_max_scan_angle() { return robot_interface->scan_angle_max; }

float get_angle_increment() { return robot_interface->scan_angle_increment; }

float get_min_scan_range() { return robot_interface->scan_range_min; }

float get_max_scan_range() { return robot_interface->scan_range_max; }

std::vector<double> get_scan_ranges() { return robot_interface->scan_ranges; }

double get_scan_range_by_index(int index) { return get_scan_ranges()[index]; }

float get_front_scan_range() {
  auto ranges = get_scan_ranges();
  if (!ranges.empty()) {
    return ranges[ranges.size() / 2];
  }
  return std::numeric_limits<float>::quiet_NaN();
}

float get_back_scan_range() {
  auto ranges = get_scan_ranges();
  if (!ranges.empty()) {
    return ranges[0];
  }
  return std::numeric_limits<float>::quiet_NaN();
}

float get_left_scan_range() {
  auto ranges = get_scan_ranges();
  if (!ranges.empty()) {
    return ranges[3 * ranges.size() / 4];
  }
  return std::numeric_limits<float>::quiet_NaN();
}

float get_right_scan_range() {
  auto ranges = get_scan_ranges();
  if (!ranges.empty()) {
    return ranges[ranges.size() / 4];
  }
  return std::numeric_limits<float>::quiet_NaN();
}

// Odometry Functions

std::map<std::string, float> get_current_position() {
  std::map<std::string, float> ans;
  ans["x"] = robot_interface->odom_position_x;
  ans["y"] = robot_interface->odom_position_y;
  ans["z"] = robot_interface->odom_position_z;
  return ans;
}

std::map<std::string, float> get_current_orientation() {
  std::map<std::string, float> ans;
  ans["r"] = robot_interface->odom_orientation_r;
  ans["p"] = robot_interface->odom_orientation_p;
  ans["y"] = robot_interface->odom_orientation_y;
  return ans;
}

float get_eucld_dist(float x1, float y1, float x2, float y2) {
  return std::sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
}

// IMU functions

std::map<std::string, float> get_current_imu_angular() {
  std::map<std::string, float> ans;
  ans["x"] = robot_interface->imu_angular_velocity_x;
  ans["y"] = robot_interface->imu_angular_velocity_y;
  ans["z"] = robot_interface->imu_angular_velocity_z;
  return ans;
}

std::map<std::string, float> get_current_imu_linear() {
  std::map<std::string, float> ans;
  ans["x"] = robot_interface->imu_linear_acceleration_x;
  ans["y"] = robot_interface->imu_linear_acceleration_y;
  ans["z"] = robot_interface->imu_linear_acceleration_z;
  return ans;
}

//~//~// finish your function definitions before this line //~//~//

void sleep_ms(int milliseconds) {
  // function to sleep the main thread for specified milliseconds
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void spin_node(rclcpp::executors::MultiThreadedExecutor *executor) {
  // make the robot interface program to run in a separate thread
  // NOTE: THE ROBOT WILL NOT WORK IF THIS FUNCTION IS REMOVED !!!
  executor->spin();
}

int main(int argc, char **argv) {

  // initialize ros2 with cpp
  rclcpp::init(argc, argv);
  // instantiate robot interface program module as a global variable
  // std::shared_ptr<RobotInterface>
  robot_interface = std::make_shared<RobotInterface>();
  // start robot interface program execution
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_interface);
  // run robot interface program in a separate thread
  std::thread executor_thread(spin_node, &executor);
  // wait for a few seconds for program to initialize
  std::cout << "Getting Ready in 5 Seconds... \n";
  sleep_ms(5000);
  std::cout << "READY !!! \n";

  // set global default decimal precision to 3
  std::cout << std::fixed << std::setprecision(3);

  try {
    //~//~// start your program after this line //~//~//

    //~//~// write code here to run only once //~//~//
    // sample_move(0.500, 0.000);
    // sleep_ms(1000);
    // sample_move(0.000, 0.000);
    // sleep_ms(1000);

    //~//~// write code here to run continuously //~//~//
    while (true) {
      //   sample_move(0.100, 0.000);
      //   sleep_ms(1000);
      //   sample_move(0.000, 0.000);
      //   sleep_ms(1000);
      std::cout << "Starting test now \n";

      // robot movements
      move_distance_front(1.000, 0.100);
      sleep_ms(1000);
      move_distance_back(1.000, 0.100);
      sleep_ms(1000);
      turn_angle_left(1.000, 1.570);
      sleep_ms(1000);
      turn_angle_right(1.000, 1.570);

      // Odometry
      std::cout << "Position1: \n";
      for (auto item : get_current_position()) {
        std::cout << item.first << ":" << item.second << std::endl;
      }
      float x_1 = get_current_position()["x"];
      float y_1 = get_current_position()["y"];

      move_distance_front(1.000, 0.100);
      sleep_ms(1000);

      std::cout << "Position2: \n";
      for (auto item : get_current_position()) {
        std::cout << item.first << ":" << item.second << std::endl;
      }
      float x_2 = get_current_position()["x"];
      float y_2 = get_current_position()["y"];

      std::cout << "distance travelled :" << get_eucld_dist(x_1, y_1, x_2, y_2)
                << std::endl;

      std::cout << "Orientation: \n";
      for (auto item : get_current_orientation()) {
        std::cout << item.first << ":" << item.second << std::endl;
      }

      // IMU
      std::cout << "IMU Angular Velocity: \n";
      for (const auto &item : get_current_imu_angular()) {
        std::cout << item.first << ":" << item.second << std::endl;
      }
      std::cout << "IMU Linear Velocity: \n";
      for (const auto &item : get_current_imu_linear()) {
        std::cout << item.first << ":" << item.second << std::endl;
      }
      std::cout << "DONE SUCCESSFULLY" << std::endl;
      break;
    }

    //~//~// finish your program before this line //~//~//
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    // In case of a standard exception, we still need to join the thread
    executor_thread.join();
    return 1;
  } catch (...) {
    // stop the robot
    robot_interface->linear_velocity = 0.000;
    robot_interface->angular_velocity = 0.000;
    sleep_ms(500);
    // clean up before shutdown
    executor_thread.join();
  }

  executor.cancel();
  // shutdown ros2
  rclcpp::shutdown();

  return 0;
}

// End of Code
