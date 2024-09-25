#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using std::placeholders::_1;

class IntensityPoker : public rclcpp::Node {
public:
  IntensityPoker();
  ~IntensityPoker() = default;

private:
  const int REFLECTIVE_INTENSITY_THRESHOLD = 3000;
  sensor_msgs::msg::LaserScan last_laser_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool done_;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

IntensityPoker::IntensityPoker()
    : Node("intensity_poker_node"),
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10,
          std::bind(&IntensityPoker::laser_scan_callback, this, _1))},
      done_{false} {}

void IntensityPoker::laser_scan_callback(
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

    int front;
    int size = static_cast<int>(last_laser_.ranges.size());

    // 'front' should be just ranges size /2
    front = static_cast<int>(round(size / 2.0));
    RCLCPP_INFO(this->get_logger(), "front index = %d", front);
    RCLCPP_INFO(this->get_logger(), "front range = %f",
                last_laser_.ranges[front]);

    double intensities_min = 10000.0;
    double intensities_max = 0.0;
    double intensities_sum = 0.0;
    int intensities_size = static_cast<int>(last_laser_.intensities.size());

    for (auto &i : last_laser_.intensities) {
      if (i < intensities_min)
        intensities_min = i;
      if (i > intensities_max)
        intensities_max = i;
      intensities_sum += i;
    }

    double intensities_avg = intensities_sum / intensities_size;

    std::vector<std::pair<int, double>> reflective_points;
    int high_intensity_count = 0;
    for (int i = 0; i < static_cast<int>(last_laser_.intensities.size()); ++i) {
      //   if (last_laser_.intensities[i] > intensities_avg) {
      if (last_laser_.intensities[i] > REFLECTIVE_INTENSITY_THRESHOLD) {
        ++high_intensity_count;
        reflective_points.push_back(
            std::make_pair(i, last_laser_.intensities[i]));
      }
    }

    RCLCPP_INFO(this->get_logger(), "Intensities min = %f", intensities_min);
    RCLCPP_INFO(this->get_logger(), "Intensities max = %f", intensities_max);
    RCLCPP_INFO(this->get_logger(), "Intensities size = %d", intensities_size);
    RCLCPP_INFO(this->get_logger(), "Intensities avg = %f", intensities_avg);
    RCLCPP_INFO(this->get_logger(), "High intensity count = %d",
                high_intensity_count);

    // std::cout << "Reflective points with intensity (index:intensity) above "
    //           << REFLECTIVE_INTENSITY_THRESHOLD << '\n';
    // for (auto &p : reflective_points)
    //   std::cout << p.first << ':' << p.second << '\n';
    // std::cout << '\n';

    std::cout << "Reflective points with intensity (index:intensity) above "
              << REFLECTIVE_INTENSITY_THRESHOLD << '\n';
    for (auto &p : reflective_points)
      std::cout << p.first << ':' << last_laser_.ranges[p.first] << '\n';
    std::cout << '\n';

    // std::cout << "All points with intensities (index:range-intensity\n";
    // for (int i = 0; i < static_cast<int>(last_laser_.ranges.size()); ++i)
    //   std::cout << i << ':' << last_laser_.ranges[i] << '-'
    //             << last_laser_.intensities[i] << '\n';

    std::cout << '\n' << std::flush;

    /**
    Intensities min = 0.000000
    Intensities max = 8000.000000
    Intensities size = 1081
    Intensities avg = 192.414431
    High intensity count = 26
    457 458 459 460 461 462 463 464 465 466 467 468 469 470
    652 653 654 655 656 657 658 659 660 661 662 663

    All 8000
    **/

    // TODO: segmentation of the indices (need two segments)
    // https://stackoverflow.com/questions/11513484/1d-number-array-clustering

    // TODO: Find the averages: avg_1, avg_2
    // TODO: Compare them and identify the lower and the higher segment
    // TODO: Find the highest index of the lower segment and call it right_ix
    // TODO: Find the lowest index of the upper segment and call it left_ix
    // TODO: The midpoint index in the arc [right_ix, left_ix] is part of the
    // midpoint
    // TODO: Get the length of the bisecting line from
    // `robot_front_laser_base_link`
    //       and the midpoint of the line between the points at left_ix and
    //       right_ix

    // TODO: How to arrive at `cart_frame`?

    done_ = true;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntensityPoker>());
  rclcpp::shutdown();

  return 0;
}
