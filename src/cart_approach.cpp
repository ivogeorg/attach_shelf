/**
 * @file approach_service_server.cpp
 * @brief Implements a final approach service for the RB1 robot.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include "attach_shelf/tf_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <tuple>

#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::placeholders;

/**
 * @class CustomSubscriptionOptions
 * @brief Used to set callback groups for topic subsctiptions.
 * A very simple derived class to allow init-list initialization
 * of callback groups for topic subscriptions due to the signature
 * of rclcpp::Node::create_subscription.
 */
// Necessary for topic subscriptions
class CustomSubscriptionOptions : public rclcpp::SubscriptionOptions {
public:
  CustomSubscriptionOptions(rclcpp::CallbackGroup::SharedPtr cb_group)
      : rclcpp::SubscriptionOptions() {
    this->callback_group = cb_group;
  }
  ~CustomSubscriptionOptions() = default;
};

/**
 * @class CartApproach
 * @brief A multi-callback node implementing the final approach service.
 */
class CartApproach : public rclcpp::Node {
public:
  CartApproach();
  ~CartApproach() = default;

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  CustomSubscriptionOptions topic_pub_sub_options_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_pose_sub_;
  std::string cmd_vel_topic_name_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_up_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_down_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_pub_;

  bool have_scan_;
  bool have_odom_;
  bool have_amcl_pose_;

  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry last_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
  double last_yaw_odom_;
  double last_yaw_amcl_;

  const double REFLECTIVE_INTENSITY_VALUE = 8000;     // for sim
  const double REFLECTIVE_INTENSITY_THRESHOLD = 3000; // for lab
  const int POINT_DIST_THRESHOLD = 10; // for point set segmentation

  std::string odom_frame_;
  std::string laser_frame_;
  std::string cart_frame_;

  bool broadcast_odom_cart_;  // connecting cart_frame to TF tree
  bool listen_to_odom_laser_; // odom_laser_t_ serves as basis for odom_cart_t_
  bool listen_to_robot_base_cart_; // laser_cart_t_ serves in the final approach

  geometry_msgs::msg::TransformStamped odom_cart_t_;
  geometry_msgs::msg::TransformStamped odom_laser_t_;
  geometry_msgs::msg::TransformStamped laser_cart_t_;
  geometry_msgs::msg::TransformStamped base_cart_t_;

  //   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; // TODO: Consider StaticTL
  rclcpp::TimerBase::SharedPtr listener_timer_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  //   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  const double LINEAR_TOLERANCE = 0.08;  // m
  const double ANGULAR_TOLERANCE = 0.01; // rad
  const double LINEAR_BASE = 0.1;        // m/s
  const double ANGULAR_BASE = 0.05;      // rad/s

  /**
   * @class MotionDirection
   * @brief An enum type for the directions of linear motion.
   * @see method go_to_frame() uses as parameter
   */
  enum class MotionDirection { FORWARD, BACKWARD };

  /**
   * @brief Linear distance the robot should correct for.
   * Before the robot rotates to orient itself toward the
   * origin of `cart_frame`, it should move forward the
   * distance between `robot_base_frame` and
   * `robot_front_laser_base_link`.
   */
  const double BASE_LINK_TO_LASER_LINK = 0.21; // m

  /**
   * @class RotationFrame
   * @brief An enum type for the frames of rotation.
   * @see method rotate() uses as parameter
   */
  enum class RotationFrame { WORLD, ROBOT };

  // callbacks
  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  inline void amcl_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  inline double get_current_yaw();

  // cart service utilities
  std::vector<std::vector<int>> segment(std::vector<int> &v,
                                        const int threshold);
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);

  // navigation
  void precise_autolocalization();

  // motion
  void move(double dist_m, MotionDirection dir, double speed); // TODO: ???
  void turn(double angle_rad, double speed);                   // TODO: ???

  /**
   * @brief Rotational-only robot motion, based on `twist.angular.z` messages
   * @param rad Degrees to turn
   * @param speed Speed of rotation in rad/s
   * @param angular_tolerance Determines the accuracy of rotation
   * @param frame World or robot frame
   * Contains an internal loop and blocks until rotation complete. Publishes
   * `geometry_msgs::msg::Twist` messages to topic `cmd_vel` or equivalent.
   * Approaches the target angle depending on the sign so as never overshoot.
   */
  void rotate(double rad, double speed, double angular_tolerance,
              RotationFrame frame = RotationFrame::ROBOT);

  bool
  go_to_frame(std::string origin_frame_id, std::string target_frame_id,
              MotionDirection dir, double min_lin_speed, double max_lin_speed,
              double min_ang_speed, double max_ang_speed, double lin_tolerance,
              double ang_tolerance, std::shared_ptr<tf2_ros::Buffer> tf_buff,
              rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub);

  // misc utilities
  double clip_speed(double value, double min, double max);
  inline double normalize_angle(double angle);
  inline double yaw_from_quaternion(double x, double y, double z, double w);

  // testing
  rclcpp::TimerBase::SharedPtr test_timer_;
  void test_cart_approach();
  void publish_laser_origin_offset();
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
CartApproach::CartApproach()
    : Node("cart_approach_test_node"),
      cb_group_{
          this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)},
      topic_pub_sub_options_{cb_group_},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&CartApproach::laser_scan_callback, this, _1),
          topic_pub_sub_options_)},
      odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10, std::bind(&CartApproach::odometry_callback, this, _1),
          topic_pub_sub_options_)},
      amcl_pose_sub_{this->create_subscription<
          geometry_msgs::msg::PoseWithCovarianceStamped>(
          "amcl_pose", 10,
          std::bind(&CartApproach::amcl_pose_callback, this, _1),
          topic_pub_sub_options_)},
      cmd_vel_topic_name_{"/diffbot_base_controller/cmd_vel_unstamped"},
      vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
          cmd_vel_topic_name_, 1)},
      elev_up_pub_{
          this->create_publisher<std_msgs::msg::String>("elevator_up", 1)},
      elev_down_pub_{
          this->create_publisher<std_msgs::msg::String>("elevator_down", 1)},
      initialpose_pub_{
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "initialpose", 1)},
      have_scan_{false}, have_odom_{false}, have_amcl_pose_{false},
      odom_frame_{"odom"}, laser_frame_{"robot_front_laser_base_link"},
      cart_frame_{"cart_frame"}, tf_buffer_{std::make_shared<tf2_ros::Buffer>(
                                     this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)},
      static_tf_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)},
      test_timer_{this->create_wall_timer(
          100ms, std::bind(&CartApproach::test_cart_approach, this),
          cb_group_)} {
  RCLCPP_INFO(this->get_logger(), "Cart approach testing sandbox started");
}

/**
 * @brief Callback for the sensor_msgs::msg::LaserScan subsctiption.
 * @param msg The lastest LaserScan message posted on the /scan topic
 */
inline void CartApproach::laser_scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_laser_ = *msg;
  have_scan_ = true;
}

/**
 * @brief Callback for the nav_msgs::msg::Odometry subsctiption.
 * @param msg The lastest Odometry message posted on the /odom topic
 */
inline void
CartApproach::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_odom_ = *msg;
  last_yaw_odom_ = yaw_from_quaternion(
      last_odom_.pose.pose.orientation.x, last_odom_.pose.pose.orientation.y,
      last_odom_.pose.pose.orientation.z, last_odom_.pose.pose.orientation.w);
  have_odom_ = true;
}

/**
 * @brief Callback for `amcl_pose` published by `amcl`
 * @param msg Latest stamped robot pose with covariance
 * Sensor fusion of odometry, lidar, and other scans. Far
 * superior to `odom`. Runs within the `nav2` stack.
 */
inline void CartApproach::amcl_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  last_amcl_pose_ = *msg;
  last_yaw_amcl_ = yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  have_amcl_pose_ = true;
}

/**
 * @brief Callback for
 * @param msg
 */
void CartApproach::precise_autolocalization() {

  // TODO
  // 1. Need subscription to /amcl_pose
  // 2. Need a publisher to /initialpose
  // 3. Publish "init_position" to /initialpose
  // 4. Get and record current robot_yaw (/amcl_pose)
  // 5. Initialize rotation_sum
  // 5. Until {x, y, z} covariances below 0.025
  //    6. Rotate -180 deg and add to rotation_sum
  //    7. Rotate 360 deg and add to rotation_sum
  //    8. Rotate -180 deg and add to rotation_sum
  // 9. Rotate robot_yaw - rotation_sum
}

void CartApproach::rotate(double goal_angle_rad, double speed,
                          double angular_tolerance, RotationFrame frame) {
  double last_angle = get_current_yaw();
  double turn_angle = 0.0;

  // If WORLD frame, subtract current robot yaw
  double goal_angle = (frame == RotationFrame::ROBOT)
                          ? goal_angle_rad
                          : normalize_angle(goal_angle_rad - get_current_yaw());

  geometry_msgs::msg::Twist twist;
  twist.angular.z = (goal_angle > 0) ? speed : -speed;
  twist.linear.x = 0.0;

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "(rotate) Angle passed: %.2f",
               goal_angle_rad * 180.0 / PI_);
  switch (frame) {
  case RotationFrame::ROBOT:
    RCLCPP_DEBUG(this->get_logger(), "(rotate) Robot frame");
    break;
  case RotationFrame::WORLD:
    RCLCPP_DEBUG(this->get_logger(), "(rotate) World frame");
    break;
  }
  RCLCPP_DEBUG(this->get_logger(), "(rotate) Current yaw: %.2f",
               get_current_yaw() * 180 / PI_);
  RCLCPP_DEBUG(this->get_logger(), "(rotate) Goal angle: %.2f",
               goal_angle * 180 / PI_);
  RCLCPP_DEBUG(this->get_logger(), "(rotate) Difference from goal: %.2f",
               (goal_angle - goal_angle_rad) * 180 / PI_);
  // end DEBUG

  // Necessary code duplication to avoid inaccuracy
  // depending on the direction of rotation.
  // Notice the condition in the while loops.
  double loop_rate = 10;
  rclcpp::Rate rate(loop_rate); // 10 Hz

  if (goal_angle > 0) {
    while (rclcpp::ok() &&
           (abs(turn_angle + angular_tolerance) < abs(goal_angle))) {

      vel_pub_->publish(twist);
      rate.sleep();

      double temp_yaw = get_current_yaw();
      RCLCPP_DEBUG(this->get_logger(), "(rotate) Current yaw: %.2f",
                   get_current_yaw() * 180 / PI_);

      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;
    }
  } else {
    while (rclcpp::ok() &&
           (abs(turn_angle - angular_tolerance) < abs(goal_angle))) {

      vel_pub_->publish(twist);
      rate.sleep();

      double temp_yaw = get_current_yaw();
      RCLCPP_DEBUG(this->get_logger(), "(rotate) Current yaw: %.2f",
                   get_current_yaw() * 180 / PI_);

      double delta_angle = normalize_angle(temp_yaw - last_angle);

      turn_angle += delta_angle;
      last_angle = temp_yaw;
    }
  }

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(), "(rotate) Stopping robot");
  // end DEBUG

  // stop robot
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  vel_pub_->publish(twist);

  // DEBUG
  RCLCPP_DEBUG(this->get_logger(),
               "(rotate) Angular difference from goal: %.2f",
               abs(goal_angle - turn_angle) * 180.0 / PI_);
  // end DEBUG
}

/**
 * @brief Returns the current yaw.
 * The yaw is constantly updated in the subsctiption callbacks.
 * There are two yaw values being kept and updated, from the
 * topics `odom` and from `amcl_pose`, the latter only if the
 * `nav2` stack is active.
 */
inline double CartApproach::get_current_yaw() {
  // NOTE:
  // A good place to select between last_yaw_amcl_
  // and last_yaw_odom_ based on parameters.

  //   return last_yaw_amcl_;
  return last_yaw_odom_;
}

/**
 * @brief Normalizes an angle between PI and -PI
 * @param angle Angle to normalize
 * @return Normalized angle
 */
inline double CartApproach::normalize_angle(double angle) {
  double res = angle;
  while (res > PI_)
    res -= 2.0 * PI_;
  while (res < -PI_)
    res += 2.0 * PI_;
  return res;
}

/**
 * @brief Simple conversion of a quaternion to Euler angle yaw.
 * @param x, y, z, w Quaternion elements
 */
inline double CartApproach::yaw_from_quaternion(double x, double y, double z,
                                                double w) {
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

/**
 * @brief Simple linear motion
 * A blocking call, not a fall-through, so don't use in
 * callbacks. Because it relaculates the distance at
 * each loop, the tolerance can be smaller.
 */
void CartApproach::move(double dist_m, MotionDirection dir, double speed) {

  // Use straight /odom)
  geometry_msgs::msg::Twist twist;
  twist.linear.x = (dir == MotionDirection::FORWARD) ? speed : -speed;
  twist.angular.z = 0.0;
  double dist = 0.0, x_init, y_init, x, y;
  x_init = last_odom_.pose.pose.position.x;
  y_init = last_odom_.pose.pose.position.y;
  RCLCPP_DEBUG(this->get_logger(), "Linear motion goal %f m", dist_m);
  while (abs(dist + LINEAR_TOLERANCE * 0.25) < dist_m) {
    vel_pub_->publish(twist);
    // rclcpp::sleep_for(100ms);

    x = last_odom_.pose.pose.position.x;
    y = last_odom_.pose.pose.position.y;
    dist += sqrt(pow(x - x_init, 2.0) + pow(y - y_init, 2.0));
    x_init = x;
    y_init = y;
  }
  // Stop robot
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  vel_pub_->publish(twist);
  RCLCPP_DEBUG(this->get_logger(), "Done moving, dist = %f m. Stopping", dist);
}

void CartApproach::turn(double angle_rad, double speed) {
  double last_angle = last_yaw_odom_;
  double turn_angle = 0;
  double goal_angle = angle_rad;
  geometry_msgs::msg::Twist twist;

  RCLCPP_DEBUG(this->get_logger(), "Turning goal %f rad", goal_angle);
  while ((goal_angle > 0 &&
          (abs(turn_angle + ANGULAR_TOLERANCE) < abs(goal_angle))) ||
         (goal_angle < 0 && (abs(turn_angle - ANGULAR_TOLERANCE) <
                             abs(goal_angle)))) { // need to turn (more)
    twist.linear.x = 0.0;
    twist.angular.z = (goal_angle > 0) ? speed : -speed;
    vel_pub_->publish(twist);

    double temp_yaw = last_yaw_odom_;
    double delta_angle = normalize_angle(temp_yaw - last_angle);

    turn_angle += delta_angle;
    last_angle = temp_yaw;
  }
  // reached goal angle within tolerance, stop turning
  //   RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", last_yaw_odom_);
  //   RCLCPP_DEBUG(this->get_logger(), "Stopping rotation, angle = %f deg",
  //                turn_angle * RAD2DEG);
  // Stop robot
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  vel_pub_->publish(twist);
  RCLCPP_DEBUG(this->get_logger(),
               "Done turning, turn angle = %f rad. Stopping", turn_angle);
}

/**
 * @brief Segments a sorted linear collection of numbers
 * @param v a vector of integers to segment
 * @param threshold the minimum distance between segments
 * @return A vector of vectors each holding a segment
 */
std::vector<std::vector<int>> CartApproach::segment(std::vector<int> &v,
                                                    const int threshold) {
  std::vector<std::vector<int>> vector_set;

  std::sort(v.begin(), v.end());

  int last = 0;
  std::vector<int> vec;
  for (auto &p : v) {
    if (p - last != p) { // not the first point
      if (p - last < threshold) {
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

/**
 * @brief Finding precise coordinates of `cart_frame`
 * Use trigonometry to return the x and y offset of
 * `cart_frame` from `robot_front_laser_base_link`
 * and the yaw the robot needs to turn to face
 * straight, first toward `cart_frame` and then in
 * along the length of the shelf/cart.
 * @param l_side the range of the innermost reflective point on the left
 * @param r_side the range of the innermost reflective point on the right
 * @param sas_angle the angle between the two rays
 * @return A tuple {x, y, yaw} (See long description above)
 */
std::tuple<double, double, double>
CartApproach::solve_sas_triangle(double l_side, double r_side,
                                 double sas_angle) {

  /***********************************************************
                frame_length
  \------------|--y-->.------------------/
   \rsa        |pi/2  .             lsa/
    \          |     .               /
     \         |     .             /
      \        |    .            /                 +x
       \    h/x|    .bisect    /
        \      |   .         /                      ^
   l_side\     |   .--ba   /r_side                  |
          \    |---ha   \/                          |
           \   |w .  \ /                    +yaw /--|--\ -yaw
            \  |a.   /                           V  |  V
             \ |y. /                      -y <----- | -----> +y
              \|./                        (in laser link frame)
               V - sas_angle
  ***********************************************************/

  // 1. Solve a2 = b2 + c2 − 2bc cosA
  //    Given: r_side, l_side, sas_angle
  //    Find:  frame_length, lsa (angle), rsa (angle)
  double frame_length, lsa, rsa;
  frame_length = sqrt(pow(r_side, 2.0) + pow(l_side, 2.0) -
                      2.0 * r_side * l_side * cos(sas_angle));

  if (r_side == l_side) {
    // rsa = lsa
    rsa = lsa = (2.0 * PI_ - sas_angle) / 2.0;
  } else if (r_side > l_side) {
    // l_side is shorter and lsa is the smaller angle
    // ( sin(lsa) / l_side ) = ( sin(sas_angle) / frame_length )
    lsa = asin(l_side * sin(sas_angle) / frame_length);
    rsa = PI_ - sas_angle - lsa; // Sum angles = PI rad
  } else {
    // r_side is shorter and rsa is the smaller angle
    // ( sin(rsa) / r_side ) = (sin(sas_angle) / frame_length )
    rsa = asin(r_side * sin(sas_angle) / frame_length);
    lsa = PI_ - sas_angle - rsa;
  }

  //   RCLCPP_DEBUG(this->get_logger(), "Given:");
  //   RCLCPP_DEBUG(this->get_logger(), "r_side=%f, l_side=%f, sas_angle=%f",
  //   r_side,
  //                l_side, sas_angle);
  //   RCLCPP_DEBUG(this->get_logger(), "Found:");
  //   RCLCPP_DEBUG(this->get_logger(), "rsa=%f, lsa=%f, frame_length=%f", rsa,
  //   lsa,
  //                frame_length);

  // 2. Solve the right triangle (l_side, frame_length - y, x (or h))
  //    Given: l_side, rsa, right angle where h intersects frame
  //    Find:  y, x, h_angle
  double h_angle, x, y;
  // (y + frame_length/2.0) = l_side * cos(rsa)
  y = l_side * cos(rsa) - frame_length / 2.0;                    // ?????
  x = sqrt(pow(l_side, 2.0) - pow(frame_length / 2.0 + y, 2.0)); // pithagoras

  // Note: See diagram above for the rsa, lha, and h_angle angles
  if (r_side == l_side) {
    h_angle = sas_angle / 2.0;
  } else if (r_side > l_side) {
    h_angle = normalize_angle(PI_ / 2.0 - lsa);
  } else {
    h_angle = normalize_angle(PI_ / 2.0 - rsa); // normalize
  }

  //   RCLCPP_DEBUG(this->get_logger(), "Given:");
  //   RCLCPP_DEBUG(this->get_logger(), "l_side=%f, rsa=%f, right angle",
  //   l_side,
  //                rsa);
  //   RCLCPP_DEBUG(this->get_logger(), "Found:");
  //   RCLCPP_DEBUG(this->get_logger(), "x=%f, y=%f, h_angle=%f", x, y,
  //   h_angle);

  // 3. Solve the SAS trinagle
  //    Given: l_side + rsa OR r_side + lsa, frame_length / 2.0
  //    Find:  b_angle, yaw = h_angle-b_angle
  double bisect, yaw = 0.0, b_angle = 0.0;

  if (r_side == l_side) {
    yaw = 0.0;
  } else if (l_side > r_side) {
    // SAS triangle: l_side, rsa, frame_length/2.0
    bisect = sqrt(pow(l_side, 2.0) + pow(frame_length / 2.0, 2.0) -
                  2.0 * l_side * (frame_length / 2.0) * cos(rsa));
    b_angle = asin((frame_length / 2.0) * sin(rsa) / bisect);
  } else {
    // SAS triangle: r_side, lsa, frame_length/2.0
    bisect = sqrt(pow(r_side, 2.0) + pow(frame_length / 2.0, 2.0) -
                  2.0 * r_side * (frame_length / 2.0) * cos(lsa));
    b_angle = asin((frame_length / 2.0) * sin(lsa) / bisect);
  }
  //   RCLCPP_DEBUG(this->get_logger(), "h_angle=%f, b_angle=%f", h_angle,
  //   b_angle);
  yaw = h_angle - b_angle;

  /***********************************************************
                frame_length
  \------------|--y-->.------------------/
   \rsa        |pi/2  .             lsa/
    \          |     .               /
     \         |     .             /               +x
      \        |    .            /
       \    h/x|    .bisect    /                    ^
        \      |   .         /                      |
   l_side\     |   .--ba   /r_side                 -|-
          \    |---ha   \/                        / | \
           \   |w .  \ /                    +yaw /  |  \ -yaw
            \  |a.   /                           V  |  V
             \ |y. /                      -y <------|------> +y
              \|./                        (in laser link frame)
               V - sas_angle
  ***********************************************************/

  // Note: See diagram for signs of y and yaw
  y = abs(y) * ((l_side > r_side) ? -1 : 1);
  yaw = abs(yaw) * ((l_side > r_side) ? 1 : -1);

  /* NOTE:
     1. x and y are for positioning crate_frame, yaw is not used in TF
     2. yaw is for the robot to face crate_frame, and then rotate back

     Essentially the TransformStamped's rotation will be ignored
  */

  // Returning x_offset, y_offset, yaw
  //   RCLCPP_DEBUG(this->get_logger(),
  //                "Solve SAS triangle return: x=%f, y=%f, yaw=%f", x, y, yaw);
  RCLCPP_INFO(
      this->get_logger(),
      "`cart_frame` TF from `robot_front_laser_base_link`: x=%f, y=%f, yaw=%f",
      x, y, yaw);
  return std::make_tuple(x, y, yaw);
}

/*******************************
Cart Approach

I.   Notes
     - facing straight in means the ranges of the two
       inner reflective plate edges are equal
     - when facing straight in, the robot yaw is the angle
       of the cart's lengthwise axis
     - when facing straight in, the robot's distance from
       the midpoint between the plates can be gotten from
       solving the isosceles triangle, being the height of
       the triangle
     - when the robot corrects for base2laser, the ranges
       to the reflective plate edges will stop being equal
     - when the robot rotates to where the ray indices to
       the two reflective plate edges their ranges have to
       be equal

II.  Algorithm
     1 stop at pose `loading_position`. should point more
       or less toward the cart
     2 identify the plates, segment, and find the edges and <--|
       distances to the two inner edges (has to be dynamic)    |
     3 move forward and turning toward the longer range        |
       until the two edge ranges are equal                     |
     4 stop, broadcast "laser_origin_offset" and use `move`    |
       to reach it                                             |
     5 rotate until the edge indices are equidistant from      |
       index front (~541). the two edge ranges have to be      |
       approximately equal. if not, back up and start over ----|
       at 2. if yes, robot is facing straight in. continue ----|
     6 broadcast "cart_frame_front_midpoint" and            <--|
       "cart_frame_centerpoint"
     7 use `move` to reach "cart_frame_front_midpoint"
     8 use `move` to reach "cart_frame_centerpoint"

III. ROS2 elements required
     - scan listener
     - odom listener (optional)
     - amcl_pose listener (optional)
     - cmd_vel publisher
     - transform listener

 *******************************/

/* ***************** Test sandbox    ***************** */

void CartApproach::publish_laser_origin_offset() {
  geometry_msgs::msg::TransformStamped ts_msg_base_to_laser =
      tf_stamped_from_frame_to_frame_3d(
          "robot_base_footprint", "robot_front_laser_base_link", tf_buffer_);

  geometry_msgs::msg::TransformStamped ts_msg_left =
      tf_stamped_from_root_frame_to_composition_frame_3d(
          "map", "robot_base_footprint", tf_buffer_);

  geometry_msgs::msg::TransformStamped ts_msg_right =
      tf_stamped_from_composition_frame_to_target_frame_2d(
          this->get_clock()->now(), "robot_base_footprint",
          "laser_origin_offset", ts_msg_base_to_laser.transform.translation.x,
          0.0);

  static_tf_broadcaster_->sendTransform(tf_stamped_from_composition(
      "map", ts_msg_left, "laser_origin_offset", ts_msg_right));

  // wait for 3 seconds (2s were enough) for the TFs to register
  rclcpp::sleep_for(3s);
}

bool CartApproach::go_to_frame(
    std::string origin_frame_id, std::string target_frame_id,
    MotionDirection dir, double min_lin_speed, double max_lin_speed,
    double min_ang_speed, double max_ang_speed, double lin_tolerance,
    double ang_tolerance, std::shared_ptr<tf2_ros::Buffer> tf_buff,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub) {
  RCLCPP_DEBUG(this->get_logger(), "Entering go_to_frame");

  bool done = false;
  geometry_msgs::msg::TransformStamped tf_msg;

  rclcpp::Rate loop_rate(10); // 10 Hz

  while (rclcpp::ok()) {
    try {
      tf_msg = tf_buff->lookupTransform(origin_frame_id, target_frame_id,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "(go_to_frame) Could not transform %s to %s: %s",
                   origin_frame_id.c_str(), target_frame_id.c_str(), ex.what());
      break;
    }
    // move the robot toward target frame
    double error_distance = std::hypotf(tf_msg.transform.translation.x,
                                        tf_msg.transform.translation.y);

    // TODO:
    // TODO:
    // TODO:
    //
    // When moving backward, what should be the sign of the angle?
    // Backing up with the joystick feels counterintuitive, so
    // should test and study with backing up to `loading_poscition`
    // and `face_shipping_position`
    if (dir == MotionDirection::BACKWARD)
      error_distance *= -1.0;

    // RCLCPP_DEBUG(this->get_logger(),
    //              "(go_to_frame) TF Listener: error_distance = %f m",
    //              error_distance);

    double error_yaw_align = yaw_from_quaternion(
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z, tf_msg.transform.rotation.w);

    double error_yaw_dir =
        atan2(tf_msg.transform.translation.y, tf_msg.transform.translation.x);
    // TODO: subtract robot yaw in world frame (odom or amcl_pose)
    // TODO: for amcl_pose, need to autolocalize accurately first (rotations in
    // place)

    // RCLCPP_DEBUG(this->get_logger(),
    //              "(go_to_frame) TF Listener: error_yaw_align = %f rad",
    //              error_yaw_align);

    geometry_msgs::msg::Twist vel_msg;

    static const double kp_yaw = 1.0;
    static const double kp_distance = 1.0;
    double base_speed;

    // TODO:
    // TODO:
    // TODO:
    //
    // While there is still linear distance, angular.z should point
    // toward the origin of the target frame. When the linear is
    // within tolerance, angular.z should align with the target frame.

    // Note:
    // To use the error_yaw_dir first to keep heading toward the goal position
    // it is best to have both linear.x and angular.z non-zero, otherwise they
    // will need to be applied angular.z first and then linear.x. This is
    // problematic because this motion is done in a loop and will require a
    // mess of boolean variables or a full state machine.

    // State machine at a rate of 10 Hz might do the job nicely and won't
    // require iterative tuning, which is impractical with the slow turnaround
    // of the dev platform.

    // States:
    // STEER_DIR (angular.z): point toward the target position
    // LINEAR (linear.x): move forward or backward as directed
    // ALIGN_YAW (angular.z): match the rotation of the target frame
    if (abs(error_yaw_align) > ang_tolerance) { // correct angular first
      vel_msg.linear.x = 0.0;
      base_speed = kp_yaw * (-error_yaw_align);
      base_speed = clip_speed(base_speed, min_ang_speed, max_ang_speed);
      vel_msg.angular.z = base_speed;
    } else if (abs(error_distance) > lin_tolerance) {
      base_speed = kp_distance * error_distance;
      base_speed = clip_speed(base_speed, min_lin_speed, max_lin_speed);
      vel_msg.linear.x = base_speed;
      vel_msg.angular.z = 0.0;
    } else {
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      done = true;
    }

    RCLCPP_DEBUG(this->get_logger(), "dist: %f, align: %f, dir: %f, x=%f, z=%f",
                 error_distance, error_yaw_align, error_yaw_dir,
                 vel_msg.linear.x, vel_msg.angular.z);

    if (!done)
      vel_pub->publish(vel_msg);
    else
      break;

    loop_rate.sleep();
  }

  RCLCPP_DEBUG(this->get_logger(), "Exiting go_to_frame");

  // TODO: use tf_msg to report the last transform

  return done;
}

/**
 * @brief Clips a speed within bounds.
 * @param value Speed to clip.
 * @param min Lower (upper) end of range.
 * @param max Upper (lower) end of range.
 * @return The clipped speed.
 * Takes abs(min) and abs(max) and applies them relative to
 * the sign of the speed to be clipped.
 */
double CartApproach::clip_speed(double value, double min, double max) {
  bool positive = value > 0.0;
  min = abs(min);
  max = abs(max);

  if (positive) {
    value = (value > max) ? max : value;
    value = (value < min) ? min : value;
  } else { // negative
    value = (value < -max) ? max : value;
    value = (value > -min) ? min : value;
  }
  return value;
}

/*
bool CartApproach::go_to_frame(
    std::string origin_frame_id, std::string target_frame_id,
    MotionDirection dir, double max_lin_speed, double max_ang_speed,
    double lin_tolerance, double ang_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf_buff,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub)
*/
void CartApproach::test_cart_approach() {
  // TODO
  // Broadcast "laser_origin_offset"
  // Move to that frame to tune the parameters of go_to_frame

  this->test_timer_->cancel();

  // Test 1: go_to_frame() FORWARD to "laser_origin_offset"
  //   publish_laser_origin_offset();
  //   RCLCPP_DEBUG(this->get_logger(), "Calling go_to_frame");
  //   bool done = go_to_frame("robot_base_footprint", "laser_origin_offset",
  //                           MotionDirection::FORWARD, 0.05, 0.1, 0.08, 0.3,
  //                           0.01, 0.017, tf_buffer_, vel_pub_);
  //   RCLCPP_INFO(this->get_logger(), "Finished test. Success: %d",
  //               static_cast<int>(done));

  // Test 2: rotate()
  while (!have_odom_)
    ;
  rotate(PI_, 0.3, 0.05, RotationFrame::ROBOT);
  rotate(-2.0 * PI_, 0.3, 0.05, RotationFrame::ROBOT);
  rotate(PI_, 0.3, 0.05, RotationFrame::ROBOT);
}

/* ***************** M    ***************** */
/* *****************  A   ***************** */
/* *****************   I  ***************** */
/* *****************    N ***************** */

/*
 * @brief main
 * Uses a MultiThreadedExecutor running a multi-
 * callback node with a single callback group.
 * Also sets the log severity level for the node.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CartApproach>();
  auto logger = node->get_logger();

  // Set the log level
  std::map<int, std::string> levels = {
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG, "DEBUG"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO, "INFO"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN, "WARN"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR, "ERROR"},
      {RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL, "FATAL"}};

  // Set the log severity level here
  int level = RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG;

  if (rcutils_logging_set_logger_level(logger.get_name(), level) !=
      RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to set logger level '%s' for %s.",
                 (levels[level]).c_str(), node->get_name());
  } else {
    RCLCPP_INFO(logger, "Successfully set logger level '%s' for %s.",
                (levels[level]).c_str(), node->get_name());
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
