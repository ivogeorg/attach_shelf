/**
 * @file tf_utils.cpp
 * @brief Various functions for tf2::Transform, TransformStamped, PoseStamped.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include "attach_shelf/tf_utils.hpp"

#include <chrono>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "builtin_interfaces/msg/detail/time__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class StaticPubFrames : public rclcpp::Node {
private:
  const std::string tf_name_load_pos_{"tf_load_pos"};
  const std::string tf_name_face_ship_pos_{"tf_face_ship_pos"};
  const std::string root_frame_{"map"};

  const std::map<std::string, std::tuple<double, double, double, double>>
      poses = {{"loading_position", {5.653875, -0.186439, -0.746498, 0.665388}},
               {"face_shipping_position",
                {2.552175, -0.092728, 0.715685, 0.698423}}};

  geometry_msgs::msg::TransformStamped tf_stamped_load_pos_;
  geometry_msgs::msg::TransformStamped tf_stamped_face_ship_pos_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  //   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  StaticPubFrames();
  ~StaticPubFrames() = default;

  void send_transforms_from_poses();
  void send_transforms_from_composition();
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
StaticPubFrames::StaticPubFrames()
    : Node("static_tf_broadcaster_from_pose"),
      static_tf_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)},
      //   tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_buffer_{std::make_shared<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)} {}

void StaticPubFrames::send_transforms_from_poses() {
  geometry_msgs::msg::PoseStamped load_pos = make_pose(
      this->get_clock()->now(), root_frame_, poses.at("loading_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(load_pos, tf_name_load_pos_));

  geometry_msgs::msg::PoseStamped face_ship_pos =
      make_pose(this->get_clock()->now(), root_frame_,
                poses.at("face_shipping_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(face_ship_pos, tf_name_face_ship_pos_));

  // wait for 3 seconds (2s were enough) for the TFs to register
  rclcpp::sleep_for(3s);
}

void StaticPubFrames::send_transforms_from_composition() {
  // TODO
  // 1. Get TransformStamped root_frame_id->composition_frame_id
  // 2. Fill out TransformStamped composition_frame_id->target_frame_id

  geometry_msgs::msg::TransformStamped ts_msg_left =
      tf_stamped_from_root_frame_to_composition_frame_3d(
          "map", "robot_front_laser_base_link", tf_buffer_);

  geometry_msgs::msg::TransformStamped ts_msg_right =
      tf_stamped_from_composition_frame_to_target_frame_2d(
          this->get_clock()->now(), "robot_front_laser_base_link",
          "laser_origin_offset", 0.0, 0.0);

  static_tf_broadcaster_->sendTransform(tf_stamped_from_composition(
      "map", ts_msg_left, "laser_origin_offset", ts_msg_right));

  // wait for 3 seconds (2s were enough) for the TFs to register
  rclcpp::sleep_for(3s);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StaticPubFrames>();
  //   node->send_transforms_from_poses();
  node->send_transforms_from_composition();

  rclcpp::shutdown();

  return 0;
}
