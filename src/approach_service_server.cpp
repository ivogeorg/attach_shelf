#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

// https://github.com/ivogeorg/robot_patrol/blob/main/src/direction_service.cpp
// #include "robot_patrol/srv/get_direction.hpp"
// using GetDirection = robot_patrol::srv::GetDirection;

// Functionality:
// 1. Detect reflective plates.
// 2. Add a `cart_frame` frame in between them.
// 3. Move the robot to `cart_frame` using a TransformListener.
// 4. Move the robot 30 cm forward and stop.
// 5. Lift the elevator to attach to the cart/shelf.

using GoToLoading = attach_shelf::srv::GoToLoading;

int main(int argc, char **argv) {

    return 0;
}