#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using std::placeholders::_1;

class LaserScannerPoker : public rclcpp::Node {
public:
  LaserScannerPoker();
  ~LaserScannerPoker() = default;

private:
  sensor_msgs::msg::LaserScan last_laser_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool done_;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

LaserScannerPoker::LaserScannerPoker()
    : Node("laser_scanner_poker_node"),
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10,
          std::bind(&LaserScannerPoker::laser_scan_callback, this, _1))},
      done_{false} {}

void LaserScannerPoker::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!done_) {
    last_laser_ = *msg;

    RCLCPP_INFO(this->get_logger(), "angle_min = %f rad", last_laser_.angle_min);
    RCLCPP_INFO(this->get_logger(), "angle_max = %f rad", last_laser_.angle_max);
    RCLCPP_INFO(this->get_logger(), "angle_increment = %f rad", last_laser_.angle_increment);
    RCLCPP_INFO(this->get_logger(), "range_min = %f rad", last_laser_.range_min);
    RCLCPP_INFO(this->get_logger(), "range_max = %f rad", last_laser_.range_max);
    RCLCPP_INFO(this->get_logger(), "ranges size = %d", static_cast<int>(last_laser_.ranges.size()));

    done_ = true;
  }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScannerPoker>());
    rclcpp::shutdown();

    return 0;
}
