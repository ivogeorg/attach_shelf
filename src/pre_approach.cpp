#include <chrono>
#include <functional>
#include <string>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using std::placeholders::_1;

class RB1Approach : public rclcpp::Node {
private:
  double obstacle_;
  double degrees_;
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry odom_data_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  enum class Motion { FORWARD, TURN, STOP };
  Motion motion_;
  const double LINEAR_BASE = 0.1;
  const double ANGULAR_BASE = 0.1;
  const double LINEAR_TOLERANCE = 0.01; // meters
  const double ANGULAR_TOLERANCE = 0.5; // degrees
  bool moving_forward_;
  bool turning_;
  bool laser_scanner_parametrized_;
  double yaw_;
  const int FRONT_FANOUT = 4;
  double angle_increment, range_min, front;

public:
  RB1Approach(double obstacle = 0.3, double degrees = -90)
      : Node("rb1_pre_approach_node"), obstacle_{obstacle}, degrees_{degrees},
        timer_{this->create_wall_timer(
            100ms, std::bind(&RB1Approach::on_timer, this))},
        vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 1)},
        scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&RB1Approach::laser_scan_callback, this, _1))},
        odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&RB1Approach::odometry_callback, this, _1))},
        motion_{Motion::FORWARD}, moving_forward_{true}, turning_{false},
        laser_scanner_parametrized_{false} {
    wait_for_laser_scan_publisher();
    wait_for_odometery_publisher();

    // will there be last_laser_?
    parametrize_laser_scanner(last_laser_);
  }

  ~RB1Approach() = default;

private:
  void on_timer() {
    // Motion: go to wall (obstacle), then turn (degrees)
    // This is a fall-through loop, which sets the Twist
    // and publishes before exiting

    if (!laser_scanner_parametrized_) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for laser scanner to be parametrized");
      return;
    }

    geometry_msgs::msg::Twist twist;

    switch (motion_) {
    case Motion::FORWARD:
      // TODO
      // if moving_forward_
      //    if abs(distance-to-wall (forward) + tolerance)) is more than
      //    'obstacle'  ???
      twist.linear.x = LINEAR_BASE;
      twist.angular.z = 0.0;
      //     else moving_forward_ == false;
      //          motion_ = Motion::TURN;
      //          twist.linear.x = 0.0;
      //          twist.angular.z = 0.0;
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
        twist.linear.x = 0.0;
        twist.angular.z = ANGULAR_BASE;

        double temp_yaw = yaw_;
        double delta_angle = normalize_angle(temp_yaw - last_angle);

        turn_angle += delta_angle;
        last_angle = temp_yaw;

        turning_ = true;
      } else {
        // reached goal angle within tolerance, stop turning
        //   RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", yaw_);
        RCLCPP_INFO(this->get_logger(),
                    "Turned within tolerance of new direction");
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;

        turning_ = false;
        motion_ = Motion::STOP;
      }
      break;
    case Motion::STOP:
    default:
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }

    vel_pub_->publish(twist);
  }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = *msg;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //   RCLCPP_DEBUG(this->get_logger(), "Odometry callback");
    odom_data_ = *msg;
    yaw_ = yaw_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //   RCLCPP_DEBUG(this->get_logger(), "Current orientation is %f", yaw_);
  }

  void parametrize_laser_scanner(sensor_msgs::msg::LaserScan &scan_data) {
    angle_increment = scan_data.angle_increment;
    range_min = scan_data.range_min;

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "angle_increment = %f", angle_increment);
    // end DEBUG

    int thirty_deg_indices = static_cast<int>((PI_ / 6.0) / angle_increment);

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "thirty_deg_indices = %d",
                 thirty_deg_indices);
    // end DEBUG

    double front_center_double = PI_ / angle_increment;
    int front_center_ix = static_cast<int>(lround(front_center_double));
    front = front_center_ix;

    // DEBUG
    RCLCPP_DEBUG(this->get_logger(), "front_center_double = %f",
                 front_center_double);
    RCLCPP_DEBUG(this->get_logger(), "front_center_ix = %d", front_center_ix);
    // end DEBUG

    // TODO: More parametrization for shelf legs
    // robot_patrol/patrol_with_service.cpp (325)

    laser_scanner_parametrized_ = true;
  }

  // TODO: is this the correct algorithm?
  double front_obstacle_dist() {
    double front_dist = 0.0;
    for (int i = front - FRONT_FANOUT; i <= front + FRONT_FANOUT; ++i) {
      if (!std::isinf(last_laser_.ranges[i]) &&
          last_laser_.ranges[i] > front_dist)
        front_dist = last_laser_.ranges[i];
    }
    return front_dist;
  }

  void wait_for_laser_scan_publisher() {
    // ROS 2 does't have an equivalent to wait_for_publisher
    // this is one way to solve the problem
    while (this->count_publishers("scan") == 0) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Interrupted while waiting for 'scan' topic publisher. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "'scan' topic publisher not available, waiting...");
    }
  }

  void wait_for_odometery_publisher() {
    // ROS 2 does't have an equivalent to wait_for_publisher
    // this is one way to solve the problem
    while (this->count_publishers("odom") == 0) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Interrupted while waiting for 'odom' topic publisher. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "'odom' topic publisher not available, waiting...");
    }
  }

  double yaw_from_quaternion(double x, double y, double z, double w) {
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
  }

  double normalize_angle(double angle) {
    double res = angle;
    while (res > PI_)
      res -= 2.0 * PI_;
    while (res < -PI_)
      res += 2.0 * PI_;
    return res;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RB1Approach>());
  rclcpp::shutdown();

  return 0;
}

// ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90