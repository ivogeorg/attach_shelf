/**
 * @file tf_utils.cpp
 * @brief Various functions for tf2::Transform, TransformStamped, PoseStamped.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include "attach_shelf/tf_utils.hpp"

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

geometry_msgs::msg::TransformStamped tf_stamped_from_composition(
    std::string left_frame_id, geometry_msgs::msg::TransformStamped left,
    std::string right_frame_id, geometry_msgs::msg::TransformStamped right) {
  geometry_msgs::msg::TransformStamped ts_msg;

  tf2::Transform t_left = transform_from_tf_stamped(left);
  tf2::Transform t_right = transform_from_tf_stamped(right);

  tf2::Transform composition = t_left * t_right;

  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Composition transform frame_id = \"%s\"",
  //               composition.);
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Composition ts_msg child_frame_id = \"%s\"",
  //               ts_msg.child_frame_id.c_str());

  ts_msg = tf_stamped_from_transform(composition);
  ts_msg.header.stamp = right.header.stamp;
  ts_msg.header.frame_id = left_frame_id;
  ts_msg.child_frame_id = right_frame_id;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left ts_msg frame_id = \"%s\"",
              left.header.frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Left ts_msg child_frame_id = \"%s\"",
              left.child_frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right ts_msg frame_id = \"%s\"",
              right.header.frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Right ts_msg child_frame_id = \"%s\"",
              right.child_frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Composition ts_msg frame_id = \"%s\"",
              ts_msg.header.frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Composition ts_msg child_frame_id = \"%s\"",
              ts_msg.child_frame_id.c_str());

  return ts_msg;
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

geometry_msgs::msg::TransformStamped
tf_stamped_from_composition_frame_to_target_frame_2d(
    builtin_interfaces::msg::Time stamp, std::string composition_frame_id,
    std::string target_frame_id, double translation_x, double translation_y) {
  geometry_msgs::msg::TransformStamped ts_msg;

  ts_msg.header.stamp = stamp;
  ts_msg.header.frame_id = composition_frame_id;
  ts_msg.child_frame_id = target_frame_id;
  ts_msg.transform.translation.x = translation_x;
  ts_msg.transform.translation.y = translation_y;
  ts_msg.transform.translation.y = 0.0;
  ts_msg.transform.rotation.x = 0.0;
  ts_msg.transform.rotation.y = 0.0;
  ts_msg.transform.rotation.z = 0.0;
  ts_msg.transform.rotation.w = 0.1;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ts_msg frame_id = \"%s\"",
              ts_msg.header.frame_id.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ts_msg child_frame_id = \"%s\"",
              ts_msg.child_frame_id.c_str());

  return ts_msg;
}

geometry_msgs::msg::TransformStamped
tf_stamped_from_root_frame_to_composition_frame_3d(
    std::string root_frame_id, std::string composition_frame_id,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  geometry_msgs::msg::TransformStamped ts_msg;
  while (ts_msg.header.frame_id == "") {
    try {
      ts_msg = tf_buffer->lookupTransform(root_frame_id, composition_frame_id,
                                          tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "Could not transform from %s to %s: %s",
          root_frame_id.c_str(), composition_frame_id.c_str(), ex.what());
      // return ts_msg;
    }
  }
  // TODO: needs work here
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "TF from \"%s\" to \"%s\": x=%f, y=%f", root_frame_id.c_str(),
              composition_frame_id.c_str(), ts_msg.transform.translation.x,
              ts_msg.transform.translation.y);
  return ts_msg;
}
