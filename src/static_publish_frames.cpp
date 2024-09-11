/**
 * @file static_publish_frames.cpp
 * @brief Frames from poses.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <chrono>

#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @class StaticPubFrames
 * @brief A multi-callback node implementing the final approach service.
 */
class StaticPubFrames : public rclcpp::Node {
private:
  const std::string tf_name_load_pos_{"tf_load_pos"};
  const std::string tf_name_face_ship_pos_{"tf_face_ship"};
  const std::string root_frame_{"map"};

  const std::map<std::string, std::tuple<double, double, double, double>>
      poses = {{"loading_position", {5.653875, -0.186439, -0.746498, 0.665388}},
               {"face_shipping_position",
                {2.552175, -0.092728, 0.715685, 0.698423}}};

  geometry_msgs::msg::TransformStamped tf_stamped_load_pos_;
  geometry_msgs::msg::TransformStamped tf_stamped_face_ship_pos_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  geometry_msgs::msg::TransformStamped
  tf_stamped_from_pose_stamped(const geometry_msgs::msg::PoseStamped pose,
                               const std::string child_frame_id);
  void send_transforms();

public:
  StaticPubFrames();
  ~StaticPubFrames() = default;
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
StaticPubFrames::StaticPubFrames()
    : Node("static_tf_broadcaster_from_pose"),
      static_tf_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)} {
  send_transforms();
}

geometry_msgs::msg::TransformStamped
StaticPubFrames::tf_stamped_from_pose_stamped(
    const geometry_msgs::msg::PoseStamped pose,
    const std::string child_frame_id) {
  geometry_msgs::msg::TransformStamped tf_stamped;

  tf_stamped.header.stamp = pose.header.stamp;
  tf_stamped.header.frame_id = pose.header.frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = pose.pose.position.x;
  tf_stamped.transform.translation.y = pose.pose.position.y;
  tf_stamped.transform.rotation.z = pose.pose.orientation.z;
  tf_stamped.transform.rotation.w = pose.pose.orientation.w;

  return tf_stamped;
}

void StaticPubFrames::send_transforms() {
  geometry_msgs::msg::PoseStamped load_pos, face_ship_pos;
  geometry_msgs::msg::TransformStamped tf_stamped;

  auto coords = poses.at("loading_position");
  load_pos.header.stamp = this->get_clock()->now();
  load_pos.header.frame_id = root_frame_;
  load_pos.pose.position.x = std::get<0>(coords);
  load_pos.pose.position.y = std::get<1>(coords);
  load_pos.pose.orientation.z = std::get<2>(coords);
  load_pos.pose.orientation.w = std::get<3>(coords);

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(
            load_pos, 
            tf_name_load_pos_));

  coords = poses.at("face_shipping_position");
  face_ship_pos.header.stamp = this->get_clock()->now();
  face_ship_pos.header.frame_id = root_frame_;
  face_ship_pos.pose.position.x = std::get<0>(coords);
  face_ship_pos.pose.position.y = std::get<1>(coords);
  face_ship_pos.pose.orientation.z = std::get<2>(coords);
  face_ship_pos.pose.orientation.w = std::get<3>(coords);

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(
            face_ship_pos, 
            tf_name_face_ship_pos_));

  // wait for 3 seconds (2s were enough) for the TFs to register
  rclcpp::sleep_for(3s);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StaticPubFrames>();

  rclcpp::shutdown();

  return 0;
}
