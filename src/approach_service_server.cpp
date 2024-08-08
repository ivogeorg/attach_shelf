#include <algorithm>
#include <chrono>
#include <iostream>
#include <tuple>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using GoToLoading = attach_shelf::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;

class ApproachServiceServer : public rclcpp::Node {
private:
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  bool have_scan_;
  sensor_msgs::msg::LaserScan last_laser_;

  const double REFLECTIVE_INTENSITY_VALUE = 8000; // for reflective points
  const int POINT_DIST_THRESHOLD = 10;            // for point set segmentation

  std::string odom_frame_;
  std::string laser_frame_;
  std::string cart_frame_;

  bool broadcast_odom_cart_;  // connecting cart_frame to TF tree
  bool listen_to_odom_laser_; // odom_laser_ serves as basis for odom_cart_
  bool listen_to_laser_cart_; // laser_cart_ serves in the final approach

  geometry_msgs::msg::TransformStamped odom_cart_;
  geometry_msgs::msg::TransformStamped odom_laser_;
  geometry_msgs::msg::TransformStamped laser_cart_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr listener_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

public:
  ApproachServiceServer();
  ~ApproachServiceServer() = default;

private:
  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = *msg;
    have_scan_ = true;
  }

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response);
  std::vector<std::vector<int>> segment(std::vector<int> &v);
  void listener_cb();
  void broadcaster_cb();
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);
};

ApproachServiceServer::ApproachServiceServer()
    : Node("approach_service_server_node"),
      srv_{create_service<GoToLoading>(
          "approach_shelf",
          std::bind(&ApproachServiceServer::service_callback, this, _1, _2))},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10,
          std::bind(&ApproachServiceServer::laser_scan_callback, this, _1))},
      have_scan_{false}, odom_frame_{"odom"}, broadcast_odom_cart_{false},
      listen_to_odom_laser_{false}, listen_to_laser_cart_{false},
      laser_frame_{"robot_front_laser_base_link"}, cart_frame_{"cart_frame"},
      tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)},
      listener_timer_{this->create_wall_timer(
          1s, std::bind(&ApproachServiceServer::listener_cb, this))},
      tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
      broadcaster_timer_{this->create_wall_timer(
          100ms, std::bind(&ApproachServiceServer::broadcaster_cb, this))} {}

void ApproachServiceServer::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Recived request with attach_to_service='%s'",
              request->attach_to_shelf ? "true" : "false");

  while (!have_scan_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for data");
    rclcpp::sleep_for(100ms);
  }

  // TODO: implement the service

  // Functionality:

  // 1. Detect reflective plates.
  std::vector<int> reflective_point_indices;
  for (int i = 0; i < static_cast<int>(last_laser_.intensities.size()); ++i)
    if (last_laser_.intensities[i] == REFLECTIVE_INTENSITY_VALUE)
      reflective_point_indices.push_back(i);

  // segment the set
  std::vector<std::vector<int>> reflective_vector_set;
  reflective_vector_set = segment(reflective_point_indices);

  RCLCPP_DEBUG(this->get_logger(), "Segmented reflective points into %d sets",
               static_cast<int>(reflective_vector_set.size()));

  // the segments are sorted, externally and internally
  // pick the two innermost points, one from each set
  int left_ix, right_ix;
  left_ix = reflective_vector_set[0][reflective_vector_set[0].size() - 1];
  right_ix = reflective_vector_set[1][0];

  RCLCPP_DEBUG(this->get_logger(), "left_ix = %d, right_ix = %d", left_ix,
               right_ix);

  // SAS triangle
  double left_range, right_range, sas_angle;
  left_range = last_laser_.ranges[left_ix];
  right_range = last_laser_.ranges[right_ix];
  sas_angle = (right_ix - left_ix) * last_laser_.angle_increment;

  RCLCPP_DEBUG(this->get_logger(),
               "left_range = %f, right_range = %f, angle = %f", left_range,
               right_range, sas_angle);

  // some x and y from solved SAS triangle, likely both negative, say x=-0.15,
  // y=-0.75
  //   double x = -0.15, y = -0.75;

  // get the transform `odom`->`robot_front_laser_base_link`

  // 2. Add a `cart_frame` TF frame in between them.
  //    Solving the SAS triangle, get the distance and angle to the midpoint
  //    Calculate the transform from `robot_front_laser_base_link`
  //    Use a (non-static) TransformPublisher to publish `cart_frame`

  // 3. Move the robot to `cart_frame` using a TransformListener.
  // 4. Move the robot 30 cm forward and stop.
  // 5. Lift the elevator to attach to the cart/shelf.

  response->complete = false;
}

std::vector<std::vector<int>>
ApproachServiceServer::segment(std::vector<int> &v) {
  std::vector<std::vector<int>> vector_set;

  std::sort(v.begin(), v.end());

  int last = 0;
  std::vector<int> vec;
  for (auto &p : v) {
    if (p - last != p) { // not the first point
      if (p - last < POINT_DIST_THRESHOLD) {
        vec.push_back(p);
      } else {
        vector_set.push_back(vec);
        // vec is copied, so can continue using it
        vec.clear();
        vec.push_back(p);
      }
    } else {
      vec.push_back(p); // first point, so just insert
    }
    last = p;
  }
  vector_set.push_back(vec);

  return vector_set;
}

void ApproachServiceServer::listener_cb() {
  // TODO

  if (listen_to_odom_laser_) {
    // TODO: listen for TF `odom`->`robot_front_laser_base_link`
  } else if (listen_to_laser_cart_) {
    // TODO: listen for TF `robot_front_laser_base_link`->`cart_frame`
  }
}

void ApproachServiceServer::broadcaster_cb() {
  // TODO

  if (broadcast_odom_cart_) {
    // TODO: broadcast TF `odom`->`cart_frame`
  } // otherwise, don't do anything
}

std::tuple<double, double, double>
ApproachServiceServer::solve_sas_triangle(double left_side, double right_side, double sas_angle) {
  // TODO

  // returning x_offset, y_offset, yaw
  return std::make_tuple(0.0, 0.0, 0.0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ApproachServiceServer>();
  auto logger = rclcpp::get_logger(node->get_name());

  // Set the log level
  std::map<int, std::string> levels = {
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, "DEBUG"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, "INFO"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, "WARN"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, "ERROR"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, "FATAL"}};
  int level = RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG;
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