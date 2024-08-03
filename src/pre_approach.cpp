#include <chrono>
#include <functional>
#include <string>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  enum class Motion { FORWARD, TURN, STOP };
  Motion motion_;
  const double LINEAR_BASE = 0.1;
  const double ANGULAR_BASE = 0.1;
  bool moving_forward_;
  bool turning_;

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
        motion_{Motion::FORWARD}, moving_forward_{true}, turning_{false} {
    // TODO
    // wait for necessary topic publishers and service providers
  }

  ~RB1Approach() = default;

private:
  void on_timer() {
    // Motion: go to wall (obstacle), then turn (degrees)
    // This is a fall-through loop, which sets the Twist
    // and publishes before

    geometry_msgs::msg::Twist twist;

    switch (motion_) {
    case Motion::FORWARD:
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
      // if turning_
      //    rotate()
      twist.linear.x = 0.0;
      twist.angular.z = ANGULAR_BASE;
      //     else turning_ == false;
      //          motion_ = Motion::STOP;
      //          twist.linear.x = 0.0;
      //          twist.angular.z = 0.0;
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

  void parametrize_laser_scanner() {
    // TODO
  }

  double front_obstacle_dist() {
    // TODO
    return 0.0;
  }

  // TODO
  // wait for /scan
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RB1Approach>());
  rclcpp::shutdown();

  return 0;
}

// ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90