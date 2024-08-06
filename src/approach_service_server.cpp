#include <ios>
#include <sstream>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// https://github.com/ivogeorg/robot_patrol/blob/main/src/direction_service.cpp
// #include "robot_patrol/srv/get_direction.hpp"
// using GetDirection = robot_patrol::srv::GetDirection;

// Functionality:
// 1. Detect reflective plates.
// 2. Add a `cart_frame` TF frame in between them.
// 3. Move the robot to `cart_frame` using a TransformListener.
// 4. Move the robot 30 cm forward and stop.
// 5. Lift the elevator to attach to the cart/shelf.

using GoToLoading = attach_shelf::srv::GoToLoading;
using LaserScan = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;

class ApproachServiceServer : public rclcpp::Node {
private:
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  bool final_approach_;

public:
  ApproachServiceServer(int argc, char **argv);
  ~ApproachServiceServer() = default;

private:
  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response);
};

ApproachServiceServer::ApproachServiceServer(int argc, char **argv)
    : Node("approach_service_server_node"),
      srv_{create_service<GoToLoading>(
          "approach_shelf",
          std::bind(&ApproachServiceServer::service_callback, this, _1, _2))},
      final_approach_{false} {

  // Argument: final_approach
  std::istringstream(argv[2]) >> std::boolalpha >> final_approach_;
  RCLCPP_INFO(this->get_logger(), "Argument 'final_approach' value '%s'",
              final_approach_ ? "true" : "false");
}

void ApproachServiceServer::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ApproachServiceServer>(argc, argv);

  //   auto logger = rclcpp::get_logger("pre_approach_node_v2");
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