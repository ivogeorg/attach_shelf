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

    RCLCPP_INFO(this->get_logger(), "angle_min = %f rad",
                last_laser_.angle_min);
    RCLCPP_INFO(this->get_logger(), "angle_max = %f rad",
                last_laser_.angle_max);
    RCLCPP_INFO(this->get_logger(), "angle_increment = %f rad",
                last_laser_.angle_increment);
    RCLCPP_INFO(this->get_logger(), "range_min = %f rad",
                last_laser_.range_min);
    RCLCPP_INFO(this->get_logger(), "range_max = %f rad",
                last_laser_.range_max);
    RCLCPP_INFO(this->get_logger(), "ranges size = %d",
                static_cast<int>(last_laser_.ranges.size()));
    RCLCPP_INFO(this->get_logger(), "intensities size = %d",
                static_cast<int>(last_laser_.intensities.size()));

    // TODO: get 'front', 'left', and 'right'
    int front, left, right;
    int size = static_cast<int>(last_laser_.ranges.size());

    // 'front' should be just ranges size /2
    front = static_cast<int>(round(size / 2.0));
    RCLCPP_INFO(this->get_logger(), "front index = %d", front);
    RCLCPP_INFO(this->get_logger(), "front range = %f",
                last_laser_.ranges[front]);

    RCLCPP_INFO(this->get_logger(), "ranges[0] = %f", last_laser_.ranges[0]);
    RCLCPP_INFO(this->get_logger(), "ranges[size-1] = %f",
                last_laser_.ranges[size - 1]);

    /*
    [laser_scanner_poker_node-1] [INFO] [1722710149.280709048] [laser_scanner_poker_node]: angle_min = -2.356200 rad
[laser_scanner_poker_node-1] [INFO] [1722710149.280871616] [laser_scanner_poker_node]: angle_max = 2.356200 rad
[laser_scanner_poker_node-1] [INFO] [1722710149.280890899] [laser_scanner_poker_node]: angle_increment = 0.004363 rad
[laser_scanner_poker_node-1] [INFO] [1722710149.280921259] [laser_scanner_poker_node]: range_min = 0.060000 rad
[laser_scanner_poker_node-1] [INFO] [1722710149.280936033] [laser_scanner_poker_node]: range_max = 20.000000 rad
[laser_scanner_poker_node-1] [INFO] [1722710149.280950238] [laser_scanner_poker_node]: ranges size = 1081
[laser_scanner_poker_node-1] [INFO] [1722710149.280962554] [laser_scanner_poker_node]: intensities size = 1081
[laser_scanner_poker_node-1] [INFO] [1722710149.280974630] [laser_scanner_poker_node]: front index = 541
[laser_scanner_poker_node-1] [INFO] [1722710149.280986514] [laser_scanner_poker_node]: front range = 0.555333
[laser_scanner_poker_node-1] [INFO] [1722710149.280997468] [laser_scanner_poker_node]: ranges[0] = 2.795250
[laser_scanner_poker_node-1] [INFO] [1722710149.281006847] [laser_scanner_poker_node]: ranges[size-1] = 4.384594
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
    */


    done_ = true;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScannerPoker>());
  rclcpp::shutdown();

  return 0;
}
