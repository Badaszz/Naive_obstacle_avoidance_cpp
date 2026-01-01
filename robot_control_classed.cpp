// includes
// ros2 rclcpp includes
#include "rclcpp/executors.hpp"                         // executors
#include "rclcpp/executors/multi_threaded_executor.hpp" // multithreaded executor
#include "rclcpp/node.hpp"                              // base node
// header includes
#include "cpp_basics_project/robot_interface_class.hpp" // robot interface class
// standard cpp includes
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

//~//~// start your class after this line //~//~//

class RobotControl {
public:
  // class constructor
  RobotControl(std::shared_ptr<RobotInterface> robot_interface)
      : robot_interface(robot_interface){};

  // class destructor
  ~RobotControl() {
    // write your termination codes here if any
  }

  std::vector<double> get_lin_ang_velocity() {
    return {robot_interface->linear_velocity,
            robot_interface->angular_velocity};
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
    robot_interface->linear_velocity = 0;
    robot_interface->angular_velocity = ang_speed;
  }

  void turn_right(double ang_speed) {
    robot_interface->linear_velocity = 0;
    robot_interface->angular_velocity = -ang_speed;
  }

  void timed_move_front(double speed, double time) {
    this->move_forward(speed);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
    this->stop_robot();
  }

  void timed_move_back(double speed, double time) {
    this->move_backward(speed);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
    this->stop_robot();
  }

  void timed_turn_left(double speed, double time) {
    this->turn_left(speed);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
    this->stop_robot();
  }

  void timed_turn_right(double speed, double time) {
    this->turn_right(speed);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(time * 1000)));
    this->stop_robot();
  }

  void move_distance_front(double speed, double distance) {
    this->timed_move_front(speed, distance / speed);
  }

  void move_distance_back(double speed, double distance) {
    this->timed_move_back(speed, distance / speed);
  }

  void turn_angle_left(double ang_speed, double angle) {
    this->timed_turn_left(ang_speed, angle / ang_speed);
  }

  void turn_angle_right(double ang_speed, double angle) {
    this->timed_turn_right(ang_speed, angle / ang_speed);
  }

  // Laser Scanner

  float get_min_scan_angle() { return robot_interface->scan_angle_min; }

  float get_max_scan_angle() { return robot_interface->scan_angle_max; }

  float get_angle_increment() { return robot_interface->scan_angle_increment; }

  float get_min_scan_range() { return robot_interface->scan_range_min; }

  float get_max_scan_range() { return robot_interface->scan_range_max; }

  std::vector<double> get_scan_ranges() { return robot_interface->scan_ranges; }

  double get_scan_range_by_index(int index) {
    return this->get_scan_ranges()[index];
  }

  float get_front_scan_range() {
    auto ranges = this->get_scan_ranges();
    if (!ranges.empty()) {
      return ranges[ranges.size() / 2];
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  float get_back_scan_range() {
    auto ranges = this->get_scan_ranges();
    if (!ranges.empty()) {
      return ranges[0];
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  float get_left_scan_range() {
    auto ranges = this->get_scan_ranges();
    if (!ranges.empty()) {
      return ranges[3 * ranges.size() / 4];
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  float get_right_scan_range() {
    auto ranges = this->get_scan_ranges();
    if (!ranges.empty()) {
      return ranges[ranges.size() / 4];
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::pair<double, int> get_min_scan_range_no_inf() {
    auto ranges = robot_interface->scan_ranges;
    double min_val = std::numeric_limits<double>::infinity();
    int min_idx = -1;
    for (size_t i = 0; i < ranges.size(); i++) {
      double r = ranges[i];
      if (std::isinf(r))
        continue; // skip infinity
      if (r < min_val) {
        min_val = r;
        min_idx = i;
      }
    }
    return {min_val, min_idx};
  }

  std::pair<double, int> get_max_scan_range_no_inf() {
    auto ranges = robot_interface->scan_ranges;
    double max_val = -std::numeric_limits<double>::infinity();
    int max_idx = -1;
    for (size_t i = 0; i < ranges.size(); i++) {
      double r = ranges[i];
      if (std::isinf(r))
        continue; // skip infinity
      if (r > max_val) {
        max_val = r;
        max_idx = i;
      }
    }

    return {max_val, max_idx};
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

  // IMU Functions

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

  // Obstacle Prediction
  int obstacle_prediction() {

    // 1. Get all scan ranges
    std::vector<double> scan_ranges = this->get_scan_ranges();
    if (scan_ranges.empty()) {
      std::cout << "No scan data." << std::endl;
      return -1;
    }

    float threshold = 0.3; // meters

    // Determine the front 90 degrees
    int front_index = scan_ranges.size() / 2;
    int half_span = scan_ranges.size() / 4;

    // Clamp indices to avoid out-of-range indexing
    int lower = std::max(0, front_index - half_span);
    int upper = std::min((int)scan_ranges.size() - 1, front_index + half_span);

    // Extract the frontal scan area
    std::vector<double> front_scan_ranges;
    front_scan_ranges.reserve(upper - lower + 1);

    for (int i = lower; i <= upper; i++) {
      front_scan_ranges.push_back(scan_ranges[i]);
    }

    // Remove infinity & find minimum range
    double min_val = std::numeric_limits<double>::infinity();

    for (double r : front_scan_ranges) {
      if (std::isinf(r))
        continue;
      if (r < min_val)
        min_val = r;
    }

    // If still INF → no valid readings
    if (std::isinf(min_val)) {
      std::cout << "No valid scan readings." << std::endl;
      return -1;
    }

    // Threshold check
    if (min_val > threshold) {
      std::cout << "Prediction: No obstacle" << std::endl;
      return 0;
    }

    // Convert to centimeters & count occurrences
    std::map<int, int> count_of_ranges;

    for (double r : front_scan_ranges) {
      if (std::isinf(r))
        continue;

      int r_cm = static_cast<int>((r * 100.0)); // convert to cm
      count_of_ranges[r_cm]++;
    }

    // Count how many values appear more than once
    int repeated_count = 0;

    for (const auto &pair : count_of_ranges) {
      if (pair.second > 1) {
        repeated_count++;
      }
    }

    // Predict obstacle vs wall
    if (repeated_count > 2) {
      std::cout << "Prediction: Wall" << std::endl;
      return 1;
    } else {
      std::cout << "Prediction: Obstacle" << std::endl;
      return 2;
    }
  }

  void direction_tracking() {
    float yaw = get_current_orientation()["y"];

    // Normalize yaw to [0, 6.28318)
    yaw += 3.14159;

    // 16 compass directions (0.39269875 each)
    static const char *directions[16] = {
        "-S-", "SSE", "S-E", "ESE", "-E-", "ENE", "N-E", "NNE",
        "-N-", "NNW", "N-W", "WNW", "-W-", "WSW", "S-W", "SSW"};

    // Each segment = 2π / 16 (0.39269875 each)
    float segment = 6.28318 / 16.0;

    // Compute index by dividing yaw by segment then finding the modulo 16
    int index = static_cast<int>(yaw / segment) % 16;

    std::cout << directions[index] << std::endl;
  }

  void naive_obstacle_avoider(int duration = 30) {
    const float THRESHOLD = 0.35; // meters
    auto start_time = std::chrono::steady_clock::now();
    float total_distance = 0.0;
    std::map<std::string, float> prev_position = this->get_current_position();

    while (true) {
      // time check
      auto now = std::chrono::steady_clock::now();
      float elapsed =
          std::chrono::duration_cast<std::chrono::seconds>(now - start_time)
              .count();
      if (elapsed > duration)
        break;

      std::vector<double> ranges = this->get_scan_ranges();
      int size = ranges.size();
      float angle_min = this->get_min_scan_angle();
      float angle_increment = this->get_angle_increment();

      // Helper: safe_min function
      auto safe_min = [](const std::vector<double> &values) {
        double m = std::numeric_limits<double>::infinity();
        for (double v : values) {
          if (!std::isinf(v) && !std::isnan(v)) {
            if (v < m)
              m = v;
          }
        }
        return m;
      };

      const float TOLERANCE = 0.45; // ~20 degrees

      // Helper to get the correct ranges
      auto angle_slice = [&](float target_angle_rad, float window_rad) {
        std::vector<double> out;

        // Calculate start and end indices based on the desired angle range
        float angle_start = target_angle_rad - window_rad;
        float angle_end = target_angle_rad + window_rad;

        // Convert angle to index using the LIDAR configuration
        // (angle - angle_min) / angle_increment
        int start_index = static_cast<int>(
            std::round((angle_start - angle_min) / angle_increment));
        int end_index = static_cast<int>(
            std::round((angle_end - angle_min) / angle_increment));

        start_index = std::max(0, start_index);
        end_index = std::min(size - 1, end_index);

        for (int i = start_index; i <= end_index; i++) {
          // indices
          if (!std::isinf(ranges[i] && !std::isnan(ranges[i]))) {
            out.push_back(ranges[i]);
          }
        }
        return out;
      };

      std::vector<double> front_r = angle_slice(0.0, TOLERANCE);
      std::vector<double> front_left_r = angle_slice(0.785, TOLERANCE);
      std::vector<double> left_r = angle_slice(1.571, TOLERANCE);
      std::vector<double> front_right_r = angle_slice(2.365, TOLERANCE);
      std::vector<double> right_r = angle_slice(3.142, TOLERANCE);

      double min_f = safe_min(front_r);
      double min_fl = safe_min(front_left_r);
      double min_fr = safe_min(front_right_r);
      double min_l = safe_min(left_r);
      double min_r = safe_min(right_r);

      std::cout << min_l << " " << min_fl << " " << min_f << " " << min_fr
                << " " << min_r << std::endl;

      // OBSTACLE AVOIDANCE LOGIC
      // Define your threshold (distance in meters)
      const float SMALL_TURN = 0.4; // ~20 degrees in radians
      const float FORWARD_STEP = 0.10;

      if (min_f > THRESHOLD && min_fl < THRESHOLD && min_fr < THRESHOLD) {
        this->move_distance_front(0.15, FORWARD_STEP);
      } else if (min_f > THRESHOLD && min_fl > THRESHOLD &&
                 min_fr > THRESHOLD) {
        this->move_distance_front(0.15, FORWARD_STEP);
      } else if (min_f > THRESHOLD && min_fl < THRESHOLD &&
                 min_fr > THRESHOLD) {
        this->move_distance_front(0.15, FORWARD_STEP);
      } else if (min_f > THRESHOLD && min_fl > THRESHOLD &&
                 min_fr < THRESHOLD) {
        this->move_distance_front(0.15, FORWARD_STEP);
      } else if (min_f < THRESHOLD && min_fl < THRESHOLD &&
                 min_fr > THRESHOLD) {
        this->turn_angle_right(0.2, SMALL_TURN);
      } else if (min_f < THRESHOLD && min_fr < THRESHOLD &&
                 min_fl > THRESHOLD) {
        this->turn_angle_left(0.2, SMALL_TURN);
      } else if (min_f < THRESHOLD && min_fr > THRESHOLD &&
                 min_fl > THRESHOLD) {
        if (min_fl > min_fr) {
          this->turn_angle_left(0.2, SMALL_TURN);
        } else {
          this->turn_angle_right(0.2, SMALL_TURN);
        }
      } else {
        if (min_fl > min_r) {
          this->turn_angle_left(0.2, 0.9); // Bigger turn
          this->move_distance_front(0.15, FORWARD_STEP);
        } else {
          this->turn_angle_right(0.2, 0.9);
          this->move_distance_front(0.15, FORWARD_STEP);
        }
      }

      // DISTANCE TRACKING
      std::map<std::string, float> curr_position = this->get_current_position();
      float step_distance =
          get_eucld_dist(prev_position["x"], prev_position["y"],
                         curr_position["x"], curr_position["y"]);
      total_distance += step_distance;
      prev_position = curr_position;

      std::cout << "Distance traveled: " << total_distance << " meters\n";
      std::cout << "Angular Velocity (z): "
                << this->get_current_imu_angular()["z"] << " rad/s\n";
      std::cout << "Linear Acceleration (x): "
                << this->get_current_imu_linear()["x"] << " m/s^2\n\n";

      this->direction_tracking();
      this->obstacle_prediction();

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->stop_robot();
    std::cout << "Navigation finished.\n";
  }

protected:
  //~//~// add protected functions & variables after this line //~//~//

private:
  std::shared_ptr<RobotInterface> robot_interface;
  //~//~// add private functions & variables after this line //~//~//

}; // class RobotControl

//~//~// finish your class before this line //~//~//

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
  std::shared_ptr<RobotInterface> robot_interface =
      std::make_shared<RobotInterface>();
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
    std::cout << "Starting Test" << std::endl;
    RobotControl rc(robot_interface);

    rc.naive_obstacle_avoider(300);
    while (true) {
      sleep_ms(1000); // replace sleep with your program code
      break;
    }

    //~//~// finish your program before this line //~//~//
  } catch (...) {
    // stop the robot
    robot_interface->linear_velocity = 0.000;
    robot_interface->angular_velocity = 0.000;
    sleep_ms(500);
    // clean up before shutdown
    executor.cancel();
    executor.remove_node(robot_interface);
  }

  // shutdown ros2
  rclcpp::shutdown();

  return 0;
}

// End of Code
