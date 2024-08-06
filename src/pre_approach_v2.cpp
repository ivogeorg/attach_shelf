#include <chrono>
#include <functional>
#include <ios>
#include <sstream>
#include <string>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using std::placeholders::_1;
using GoToLoading = attach_shelf::srv::GoToLoading;

class RB1Approach : public rclcpp::Node {
private:
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry odom_data_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<GoToLoading>::SharedPtr client_;

  enum class Motion { FORWARD, TURN, STOP };
  Motion motion_;

  const double LINEAR_BASE = 0.5;
  const double ANGULAR_BASE = 0.25;
  const double LINEAR_TOLERANCE = 0.005;  // meters
  const double ANGULAR_TOLERANCE = 0.012; // about 2/3 of a deg
  const double ANGULAR_TOLERANCE_DEG = ANGULAR_TOLERANCE * RAD2DEG;
  bool moving_forward_;
  bool turning_;
  bool have_odom_;
  bool have_scan_;
  bool laser_scanner_parametrized_;
  double yaw_;
  const int FRONT_FANOUT = 4;
  double angle_increment, range_min;
  int front;

  // Arguments/parameters
  double obstacle_;
  double degrees_;
  bool final_approach_;

public:
  RB1Approach(int argc, char **argv);
  ~RB1Approach() = default;

private:
  void on_timer();
  void parametrize_laser_scanner(sensor_msgs::msg::LaserScan &scan_data);
  double front_obstacle_dist();
  void wait_for_laser_scan_publisher();
  void wait_for_odometery_publisher();
  void service_response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);

  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = *msg;
    have_scan_ = true;
  }

  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //   RCLCPP_DEBUG(this->get_logger(), "Odometry callback");
    odom_data_ = *msg;
    yaw_ = yaw_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //   RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
    have_odom_ = true;
  }

  inline double yaw_from_quaternion(double x, double y, double z, double w) {
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
  }

  inline double normalize_angle(double angle) {
    double res = angle;
    while (res > PI_)
      res -= 2.0 * PI_;
    while (res < -PI_)
      res += 2.0 * PI_;
    return res;
  }
};

RB1Approach::RB1Approach(int argc, char **argv)
    : Node("pre_approach_node_v2"), timer_{this->create_wall_timer(
                                        100ms, std::bind(&RB1Approach::on_timer,
                                                         this))},
      vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 1)},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&RB1Approach::laser_scan_callback, this, _1))},
      odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&RB1Approach::odometry_callback, this, _1))},
      client_{this->create_client<GoToLoading>("approach_shelf")},
      motion_{Motion::FORWARD}, moving_forward_{true}, turning_{false},
      have_odom_{false}, have_scan_{false}, laser_scanner_parametrized_{false} {
  wait_for_laser_scan_publisher();
  wait_for_odometery_publisher();

  // Argument: obstacle
  obstacle_ = std::stof(argv[2]);
  RCLCPP_INFO(this->get_logger(), "Argument 'obstacle' value %f m", obstacle_);

  // Argument: degrees
  degrees_ = std::stof(argv[4]);
  RCLCPP_INFO(this->get_logger(), "Argument 'degrees' value %f deg", degrees_);

  // Argument: final_approach
  std::istringstream(argv[6]) >> std::boolalpha >> final_approach_;
  RCLCPP_INFO(this->get_logger(), "Argument 'final_approach' value '%s'",
              final_approach_ ? "true" : "false");
}

void RB1Approach::on_timer() {
  // Motion: go to wall (obstacle), then turn (degrees)
  // This is a fall-through loop, which sets the Twist
  // and publishes before exiting
  if (!have_odom_ || !have_scan_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for data");
    return;
  }

  if (!laser_scanner_parametrized_) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for laser scanner to be parametrized");
    parametrize_laser_scanner(last_laser_);
    return;
  }

  geometry_msgs::msg::Twist twist;

  switch (motion_) {
  case Motion::FORWARD:
    if (moving_forward_) {
      if (front_obstacle_dist() >= obstacle_ + LINEAR_TOLERANCE) {
        RCLCPP_DEBUG(this->get_logger(), "Approaching wall, distance = %f m",
                     front_obstacle_dist());
        twist.linear.x = LINEAR_BASE;
        twist.angular.z = 0.0;
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Stopping at wall, distance = %f m",
                     front_obstacle_dist());
        moving_forward_ = false;
        motion_ = Motion::TURN;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
      }
    }
    break;
  case Motion::TURN:
    static double last_angle;
    static double turn_angle;
    static double goal_angle;

    // if not turning, initialize to start
    if (!turning_) {
      last_angle = yaw_;
      turn_angle = 0;
      goal_angle = degrees_ * DEG2RAD;
    }

    if ((goal_angle > 0 &&
         (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) ||
        (goal_angle < 0 && (abs(turn_angle - ANGULAR_TOLERANCE) <
                            abs(goal_angle)))) { // need to turn (more)
      RCLCPP_DEBUG(this->get_logger(), "Starting rotation, angle = %f deg",
                   turn_angle * RAD2DEG);
      twist.linear.x = 0.0;
      twist.angular.z = (goal_angle > 0) ? ANGULAR_BASE : -ANGULAR_BASE;

      double temp_yaw = yaw_;
      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;

      turning_ = true;
    } else {
      // reached goal angle within tolerance, stop turning
      //   RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", yaw_);
      RCLCPP_DEBUG(this->get_logger(), "Stopping rotation, angle = %f deg",
                   turn_angle * RAD2DEG);
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;

      turning_ = false;
      motion_ = Motion::STOP;
    }
    break;
  case Motion::STOP:
    RCLCPP_INFO(this->get_logger(), "Pre-approach completed");
    // If robot not stopped, stop it, else call service
    if (abs(twist.linear.x) > 0.0 || abs(twist.angular.z) > 0.0) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Robot stopped");
      timer_->cancel(); // timer not needed

      auto request = std::make_shared<GoToLoading::Request>();
      request->attach_to_shelf = final_approach_;

      auto result_future = client_->async_send_request(
          request, std::bind(&RB1Approach::service_response_callback, this,
                             std::placeholders::_1));
      RCLCPP_DEBUG(this->get_logger(),
                   "Requested final approach to be completed: '%s",
                   final_approach_ ? "yes" : "no");
    }
    break;
  default:
    RCLCPP_WARN(this->get_logger(), "Unrecognized motion. Stopping robot");
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
  }

  vel_pub_->publish(twist);
}

void RB1Approach::parametrize_laser_scanner(
    sensor_msgs::msg::LaserScan &scan_data) {
  int size = static_cast<int>(scan_data.ranges.size());
  front = static_cast<int>(round(size / 2.0));
  RCLCPP_DEBUG(this->get_logger(), "front index = %d", front);
  RCLCPP_DEBUG(this->get_logger(), "front range = %f",
               last_laser_.ranges[front]);

  laser_scanner_parametrized_ = true;

  RCLCPP_INFO(this->get_logger(), "Laser scanner parametrized");
}

double RB1Approach::front_obstacle_dist() {
  double front_dist = 0.0;
  for (int i = front - FRONT_FANOUT; i <= front + FRONT_FANOUT; ++i) {
    if (!std::isinf(last_laser_.ranges[i]) &&
        last_laser_.ranges[i] > front_dist)
      front_dist = last_laser_.ranges[i];
  }
  return front_dist;
}

void RB1Approach::wait_for_laser_scan_publisher() {
  // ROS 2 does't have an equivalent to wait_for_publisher
  // this is one way to solve the problem
  while (this->count_publishers("scan") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for 'scan' topic publisher. Exiting.");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "'scan' topic publisher not available, waiting...");
  }
  RCLCPP_INFO(this->get_logger(), "'scan' topic publisher acquired");
}

void RB1Approach::wait_for_odometery_publisher() {
  // ROS 2 does't have an equivalent to wait_for_publisher
  // this is one way to solve the problem
  while (this->count_publishers("odom") == 0) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Interrupted while waiting for 'odom' topic publisher. Exiting.");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "'odom' topic publisher not available, waiting...");
  }
  RCLCPP_INFO(this->get_logger(), "'odom' topic publisher acquired");
}

void RB1Approach::service_response_callback(
    rclcpp::Client<GoToLoading>::SharedFuture future) {
  auto status = future.wait_for(1s);
  bool final_approach_complete = false;
  if (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Service '/approach_shelf' in progress...");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Service '/approach_shelf' response received");
    auto result = future.get();
    final_approach_complete = result->complete;
    RCLCPP_INFO(this->get_logger(), "Final approach complete: '%s'",
                final_approach_complete ? "true" : "false");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RB1Approach>(argc, argv);
  auto logger = rclcpp::get_logger(node->get_name());

  // Set the log level
  std::map<int, std::string> levels = {
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, "DEBUG"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, "INFO"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, "WARN"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, "ERROR"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, "FATAL"}};
  int level = RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO;
  if (rcutils_logging_set_logger_level(logger.get_name(), level) !=
      RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to set logger level '%s' for %s.",
                 (levels[level]).c_str(), node->get_name());
  } else {
    RCLCPP_INFO(logger, "Successfully set logger level '%s' for %s.",
                (levels[level]).c_str(), node->get_name());
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
