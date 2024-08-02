#include <string>

#include "rclcpp/rclcpp.hpp"

class RB1Approach : public rclcpp::Node {
private:
  double obstacle_;
  double degrees_;

public:
  RB1Approach(double obstacle = 0.3, double degrees = -90)
      : Node("rb1_pre_approach_node"), obstacle_{obstacle}, degrees_{degrees} {}

  ~RB1Approach() = default;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RB1Approach>());
  rclcpp::shutdown();

  return 0;
}

// ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90