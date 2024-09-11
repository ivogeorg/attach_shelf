#ifndef TF_UTILS_HPP__
#define TF_UTILS_HPP__

#include <string>
#include <tuple>

#include "builtin_interfaces/msg/detail/time__struct.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"


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


#endif // TF_UTILS_HPP__