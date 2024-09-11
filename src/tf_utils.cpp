/**
 * @file tf_utils.cpp
 * @brief Various functions for tf2::Transform, TransformStamped, PoseStamped.
 * @author Ivo Georgiev
 * @version 0.9
 */

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
#include "tf2_ros/static_transform_broadcaster.h"

// ///////////////// FOR HEADER FILE /////////////////

geometry_msgs::msg::TransformStamped
tf_stamped_from_pose_stamped(const geometry_msgs::msg::PoseStamped pose,
                             const std::string child_frame_id);
geometry_msgs::msg::PoseStamped make_pose(builtin_interfaces::msg::Time stamp,
                                          std::string parent_frame_id,
                                          double position_x, double position_y,
                                          double orientation_z,
                                          double orientation_w);

geometry_msgs::msg::PoseStamped
make_pose(builtin_interfaces::msg::Time stamp, std::string parent_frame_id,
          std::tuple<double, double, double, double> coords);

geometry_msgs::msg::PoseStamped make_pose(builtin_interfaces::msg::Time stamp,
                                          std::string parent_frame_id,
                                          geometry_msgs::msg::Pose pose);

geometry_msgs::msg::TransformStamped
tf_stamped_from_composition(geometry_msgs::msg::TransformStamped left,
                            geometry_msgs::msg::TransformStamped right);

tf2::Transform transform_from_tf_stamped(geometry_msgs::msg::TransformStamped);

geometry_msgs::msg::TransformStamped tf_stamped_from_transform(tf2::Transform);

// ///////////////// END HEADER FILE /////////////////

using namespace std::chrono_literals;

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

public:
  StaticPubFrames();
  ~StaticPubFrames() = default;

  void send_transforms();
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
StaticPubFrames::StaticPubFrames()
    : Node("static_tf_broadcaster_from_pose"),
      static_tf_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)} {}

void StaticPubFrames::send_transforms() {
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

// ///////////////// UTILITIES /////////////////
geometry_msgs::msg::TransformStamped
tf_stamped_from_pose_stamped(const geometry_msgs::msg::PoseStamped pose,
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

geometry_msgs::msg::PoseStamped make_pose(builtin_interfaces::msg::Time stamp,
                                          std::string parent_frame_id,
                                          double position_x, double position_y,
                                          double orientation_z,
                                          double orientation_w) {
  geometry_msgs::msg::PoseStamped p;

  p.header.stamp = stamp;
  p.header.frame_id = parent_frame_id;
  p.pose.position.x = position_x;
  p.pose.position.y = position_y;
  p.pose.orientation.z = orientation_z;
  p.pose.orientation.w = orientation_w;

  return p;
}

geometry_msgs::msg::PoseStamped
make_pose(builtin_interfaces::msg::Time stamp, std::string parent_frame_id,
          std::tuple<double, double, double, double> coords) {
  geometry_msgs::msg::PoseStamped p;

  p.header.stamp = stamp;
  p.header.frame_id = parent_frame_id;
  p.pose.position.x = std::get<0>(coords);
  p.pose.position.y = std::get<1>(coords);
  p.pose.orientation.z = std::get<2>(coords);
  p.pose.orientation.w = std::get<3>(coords);

  return p;
}

geometry_msgs::msg::PoseStamped make_pose(builtin_interfaces::msg::Time stamp,
                                          std::string parent_frame_id,
                                          geometry_msgs::msg::Pose pose) {
  geometry_msgs::msg::PoseStamped p;

  p.header.stamp = stamp;
  p.header.frame_id = parent_frame_id;
  p.pose.position.x = pose.position.x;
  p.pose.position.y = pose.position.y;
  p.pose.orientation.z = pose.orientation.z;
  p.pose.orientation.w = pose.orientation.w;

  return p;
}

geometry_msgs::msg::TransformStamped
tf_stamped_from_composition(geometry_msgs::msg::TransformStamped left,
                            geometry_msgs::msg::TransformStamped right) {
  geometry_msgs::msg::TransformStamped ts_msg;

  tf2::Transform t_left = transform_from_tf_stamped(left);
  tf2::Transform t_right = transform_from_tf_stamped(right);

  tf2::Transform composition = t_left * t_right;

  return tf_stamped_from_transform(composition);
}

tf2::Transform
transform_from_tf_stamped(geometry_msgs::msg::TransformStamped ts_msg) {
  tf2::Transform tf;

  tf.setOrigin(tf2::Vector3(ts_msg.transform.translation.x,
                            ts_msg.transform.translation.y,
                            ts_msg.transform.translation.z));

  tf.setRotation(tf2::Quaternion(
      ts_msg.transform.rotation.x, ts_msg.transform.rotation.y,
      ts_msg.transform.rotation.z, ts_msg.transform.rotation.w));

  return tf;
}

geometry_msgs::msg::TransformStamped
tf_stamped_from_transform(tf2::Transform tf) {
  geometry_msgs::msg::TransformStamped ts_msg;

  ts_msg.transform.translation.x = tf.getOrigin().x();
  ts_msg.transform.translation.y = tf.getOrigin().y();
  ts_msg.transform.translation.z = tf.getOrigin().z();

  ts_msg.transform.rotation.x = tf.getRotation().x();
  ts_msg.transform.rotation.y = tf.getRotation().y();
  ts_msg.transform.rotation.z = tf.getRotation().z();
  ts_msg.transform.rotation.w = tf.getRotation().w();

  return ts_msg;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StaticPubFrames>();
  node->send_transforms();

  rclcpp::shutdown();

  return 0;
}
