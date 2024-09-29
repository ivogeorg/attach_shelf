/**
 * @file tf_utils.cpp
 * @brief Various functions for tf2::Transform, TransformStamped, PoseStamped.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include "attach_shelf/tf_utils.hpp"

#include <cmath>
#include <string>
#include <tuple>
#include <utility>

#include "builtin_interfaces/msg/detail/time__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

// forward declarations
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch,
                                                     double yaw);

// library function definitions
geometry_msgs::msg::TransformStamped
tf_stamped_from_pose_stamped(const geometry_msgs::msg::PoseStamped pose,
                             const std::string child_frame_id) {
  geometry_msgs::msg::TransformStamped tf_stamped;

  tf_stamped.header.stamp = pose.header.stamp;
  tf_stamped.header.frame_id = pose.header.frame_id;
  tf_stamped.child_frame_id = child_frame_id;
  tf_stamped.transform.translation.x = pose.pose.position.x;
  tf_stamped.transform.translation.y = pose.pose.position.y;
  tf_stamped.transform.translation.z = pose.pose.position.z;
  tf_stamped.transform.rotation.x = pose.pose.orientation.x;
  tf_stamped.transform.rotation.y = pose.pose.orientation.y;
  tf_stamped.transform.rotation.z = pose.pose.orientation.z;
  tf_stamped.transform.rotation.w = pose.pose.orientation.w;

  return tf_stamped;
}

geometry_msgs::msg::PoseStamped
make_pose(builtin_interfaces::msg::Time stamp, std::string parent_frame_id,
          double position_x, double position_y, double position_z,
          double orientation_x, double orientation_y, double orientation_z,
          double orientation_w) {
  geometry_msgs::msg::PoseStamped p;

  p.header.stamp = stamp;
  p.header.frame_id = parent_frame_id;
  p.pose.position.x = position_x;
  p.pose.position.y = position_y;
  p.pose.position.z = position_z;
  p.pose.orientation.x = orientation_x;
  p.pose.orientation.y = orientation_y;
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
  p.pose.position.z = pose.position.z;
  p.pose.orientation.x = pose.orientation.x;
  p.pose.orientation.y = pose.orientation.y;
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

  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left ts_msg frame_id =
  //   \"%s\"",
  //               left.header.frame_id.c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Left ts_msg child_frame_id = \"%s\"",
  //               left.child_frame_id.c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right ts_msg frame_id =
  //   \"%s\"",
  //               right.header.frame_id.c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Right ts_msg child_frame_id = \"%s\"",
  //               right.child_frame_id.c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Composition ts_msg frame_id = \"%s\"",
  //               ts_msg.header.frame_id.c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //               "Composition ts_msg child_frame_id = \"%s\"",
  //               ts_msg.child_frame_id.c_str());

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
  ts_msg.transform.translation.z = 0.0;
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

// TODO: The error messages are commented out because they flood
//        stdout. Should find a more graceful way to handle.
geometry_msgs::msg::TransformStamped
tf_stamped_from_frame_to_frame_3d(std::string from_frame_id,
                                  std::string to_frame_id,
                                  std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  geometry_msgs::msg::TransformStamped ts_msg;
  while (ts_msg.header.frame_id == "") {
    try {
      ts_msg = tf_buffer->lookupTransform(from_frame_id, to_frame_id,
                                          tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      //              "Could not transform from %s to %s: %s",
      //              from_frame_id.c_str(), to_frame_id.c_str(), ex.what());
      // return ts_msg;
    }
  }
  // TODO: needs work here
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  //             "TF from \"%s\" to \"%s\": x=%f, y=%f",
  //             from_frame_id.c_str(), to_frame_id.c_str(),
  //             ts_msg.transform.translation.x,
  //             ts_msg.transform.translation.y);
  return ts_msg;
}

geometry_msgs::msg::TransformStamped
tf_stamped_from_root_frame_to_composition_frame_3d(
    std::string root_frame_id, std::string composition_frame_id,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  return tf_stamped_from_frame_to_frame_3d(root_frame_id, composition_frame_id,
                                           tf_buffer);
}

geometry_msgs::msg::TransformStamped tf_stamped_from_relative_coordinates(
    builtin_interfaces::msg::Time stamp, std::string root_frame_id,
    std::string origin_frame_id, std::string target_frame_id,
    double origin_to_target_x, double origin_to_target_y,
    double origin_to_target_yaw, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {

  // 1. Convert {yaw} to quaternion {z, w}
  geometry_msgs::msg::Quaternion origin_to_target_q =
      quaternion_from_yaw(origin_to_target_yaw);

  // 2. make_pose relative to origin_frame_id with coordinates
  geometry_msgs::msg::PoseStamped origin_to_target_coordinates_pose =
      make_pose(stamp, origin_frame_id, origin_to_target_x, origin_to_target_y,
                0.0, 0.0, 0.0, origin_to_target_q.z, origin_to_target_q.w);

  // 3. tf_stamped_from_pose_stamped with target_frame_id  => this will be right
  // side
  geometry_msgs::msg::TransformStamped ts_msg_right_side =
      tf_stamped_from_pose_stamped(origin_to_target_coordinates_pose,
                                   target_frame_id);

  // 4. tf_stamped_from_root_frame_to_composition_frame_3d with root_frame_id
  // and origin_frame_id => this will be left side
  geometry_msgs::msg::TransformStamped ts_msg_left_side =
      tf_stamped_from_root_frame_to_composition_frame_3d(
          root_frame_id, origin_frame_id, tf_buffer);

  // 5. tf_stamped_from_composition with root_frame_id and target_frame_id
  geometry_msgs::msg::TransformStamped composition =
      tf_stamped_from_composition(root_frame_id, ts_msg_left_side,
                                  target_frame_id, ts_msg_right_side);

  RCLCPP_DEBUG(
      rclcpp::get_logger("rclcpp"),
      "(tf_utils) Returning TF (%f, %f, %f)(%f, %f, %f, %f)",
      composition.transform.translation.x, composition.transform.translation.y,
      composition.transform.translation.z, composition.transform.rotation.x,
      composition.transform.rotation.y, composition.transform.rotation.z,
      composition.transform.rotation.w);

  return composition;
}

geometry_msgs::msg::TransformStamped tf_stamped_from_relative_coordinates_3d(
    builtin_interfaces::msg::Time stamp, std::string root_frame_id,
    std::string origin_frame_id, std::string target_frame_id,
    double origin_to_target_x, double origin_to_target_y,
    double origin_to_target_z, double origin_to_target_roll,
    double origin_to_target_pitch, double origin_to_target_yaw,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {

  // 1. Convert {yaw} to quaternion {z, w}
  geometry_msgs::msg::Quaternion origin_to_target_q = quaternion_from_euler(
      origin_to_target_roll, origin_to_target_pitch, origin_to_target_yaw);

  // 2. make_pose relative to origin_frame_id with coordinates
  geometry_msgs::msg::PoseStamped origin_to_target_coordinates_pose =
      make_pose(stamp, origin_frame_id, origin_to_target_x, origin_to_target_y,
                origin_to_target_z, origin_to_target_q.x, origin_to_target_q.y,
                origin_to_target_q.z, origin_to_target_q.w);

  // 3. tf_stamped_from_pose_stamped with target_frame_id  => this will be right
  // side
  geometry_msgs::msg::TransformStamped ts_msg_right_side =
      tf_stamped_from_pose_stamped(origin_to_target_coordinates_pose,
                                   target_frame_id);

  // 4. tf_stamped_from_root_frame_to_composition_frame_3d with root_frame_id
  // and origin_frame_id => this will be left side
  geometry_msgs::msg::TransformStamped ts_msg_left_side =
      tf_stamped_from_root_frame_to_composition_frame_3d(
          root_frame_id, origin_frame_id, tf_buffer);

  // 5. tf_stamped_from_composition with root_frame_id and target_frame_id
  geometry_msgs::msg::TransformStamped composition =
      tf_stamped_from_composition(root_frame_id, ts_msg_left_side,
                                  target_frame_id, ts_msg_right_side);

  // This won't show unless the "rclcpp" logger level is set to DEBUG
  RCLCPP_DEBUG(
      rclcpp::get_logger("rclcpp"),
      "(tf_utils) Returning TF (%f, %f, %f)(%f, %f, %f, %f)",
      composition.transform.translation.x, composition.transform.translation.y,
      composition.transform.translation.z, composition.transform.rotation.x,
      composition.transform.rotation.y, composition.transform.rotation.z,
      composition.transform.rotation.w);

  return composition;
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.x = q.y = 0.0;
  q.z = sy;
  q.w = cy;
  return q;
}

geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch,
                                                     double yaw) {
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}
