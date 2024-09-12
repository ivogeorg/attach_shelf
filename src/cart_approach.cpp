/**
 * @file approach_service_server.cpp
 * @brief Implements a final approach service for the RB1 robot.
 * @author Ivo Georgiev
 * @version 0.9
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <tuple>

#include "attach_shelf/srv/pick_up_cart.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
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
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define PI_ 3.14159265359
#define DEG2RAD (PI_ / 180.0)
#define RAD2DEG (180.0 / PI_)

using namespace std::chrono_literals;
using PickUpCart = attach_shelf::srv::PickUpCart;
using LaserScan = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;

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
private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  CustomSubscriptionOptions topic_pub_sub_options_;

  rclcpp::Service<PickUpCart>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_up_pub_;

  bool have_scan_;
  bool have_odom_;
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry last_odom_;
  double last_yaw_;

  const double REFLECTIVE_INTENSITY_VALUE = 8000;     // for reflective points
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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; // TODO: Consider StaticTL
  rclcpp::TimerBase::SharedPtr listener_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  const double LINEAR_TOLERANCE = 0.08;  // m
  const double ANGULAR_TOLERANCE = 0.01; // rad
  const double LINEAR_BASE = 0.1;        // m/s
  const double ANGULAR_BASE = 0.05;       // rad/s

  /**
   * @class MotionDirection
   * @brief An enum type for the directions of motion.
   * @see method move() uses as parameter
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

public:
  CartApproach();
  ~CartApproach() = default;

private:
  /**
   * @brief Callback for the sensor_msgs::msg::LaserScan subsctiption.
   * @param msg The lastest LaserScan message posted on the /scan topic
   */
  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = *msg;
    have_scan_ = true;
  }

  /**
   * @brief Callback for the nav_msgs::msg::Odometry subsctiption.
   * @param msg The lastest Odometry message posted on the /odom topic
   */
  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg;
    last_yaw_ = yaw_from_quaternion(
        last_odom_.pose.pose.orientation.x, last_odom_.pose.pose.orientation.y,
        last_odom_.pose.pose.orientation.z, last_odom_.pose.pose.orientation.w);
    have_odom_ = true;
  }

  /**
   * @brief Normalizes an angle between PI and -PI
   * @param angle Angle to normalize
   * @return Normalized angle
   */
  inline double normalize_angle(double angle) {
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
  inline double yaw_from_quaternion(double x, double y, double z, double w) {
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
  }

  void service_callback(const std::shared_ptr<PickUpCart::Request> request,
                        const std::shared_ptr<PickUpCart::Response> response);
  std::vector<std::vector<int>> segment(std::vector<int> &v,
                                        const int threshold);
  void listener_cb();
  void broadcaster_cb();
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);
  void move(double dist_m, MotionDirection dir, double speed);
  void turn(double angle_rad, double speed);
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
CartApproach::CartApproach()
    : Node("cart_pick_up_server"),
      cb_group_{
          this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)},
      topic_pub_sub_options_{cb_group_},
      srv_{this->create_service<PickUpCart>(
          "cart_pick_up",
          std::bind(&CartApproach::service_callback, this, _1, _2),
          rmw_qos_profile_services_default, cb_group_)},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10,
          std::bind(&CartApproach::laser_scan_callback, this, _1),
          topic_pub_sub_options_)},
      odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10,
          std::bind(&CartApproach::odometry_callback, this, _1),
          topic_pub_sub_options_)},
      vel_pub_{
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1)},
      elev_up_pub_{
          this->create_publisher<std_msgs::msg::String>("/elevator_up", 1)},
      have_scan_{false}, have_odom_{false}, odom_frame_{"odom"},
      laser_frame_{"robot_front_laser_base_link"}, cart_frame_{"cart_frame"},
      broadcast_odom_cart_{false}, listen_to_odom_laser_{false},
      listen_to_robot_base_cart_{false},
      tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)},
      listener_timer_{this->create_wall_timer(
          100ms, std::bind(&CartApproach::listener_cb, this),
          cb_group_)},
      tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
      broadcaster_timer_{this->create_wall_timer(
          100ms, std::bind(&CartApproach::broadcaster_cb, this),
          cb_group_)} {
  // TODO: Change to LINEAR_CORRECTION if included, either here or in
  // service_cb()
  RCLCPP_INFO(this->get_logger(), "Server of /pick_up_cart service started");
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Top frame of the robot, parent to the base link of the robot, and usually child of the world frame.";
  this->declare_parameter<std::string>("odom_frame_id", "odom", param_desc);
  get_parameter("odom_frame_id", odom_frame_);
  RCLCPP_INFO(this->get_logger(), "odom_frame_id is: %s", odom_frame_.c_str());
}

/**
 * @brief The callback for the /cart_pick_up service.
 * It is called after the pre-approach is complete and
 * implements the final approach and the attachment of
 * the shelf/crate to the RB1 robot.
 * @param request Service request (pick_up_cart)
 * @param response Service response (cart_picked_up)
 */
void CartApproach::service_callback(
    const std::shared_ptr<PickUpCart::Request> request,
    const std::shared_ptr<PickUpCart::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Recived request with attach_to_service='%s'",
              request->pick_up_cart ? "true" : "false");

  // Wait for /scan and /odom callbacks to be called
  while (!have_scan_ || !have_odom_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for data");
    rclcpp::sleep_for(100ms);
  }

  // 1. Detect reflective plates.
  std::vector<int> reflective_point_indices;
  for (int i = 0; i < static_cast<int>(last_laser_.intensities.size()); ++i)
    // if (last_laser_.intensities[i] == REFLECTIVE_INTENSITY_VALUE)
    if (last_laser_.intensities[i] > REFLECTIVE_INTENSITY_THRESHOLD)
      reflective_point_indices.push_back(i);

  // 1.1 If none, return with complete=False
  if (reflective_point_indices.size() == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Did not detect any cart reflective plates");
    response->cart_picked_up = false;
    return;
  }

  // 1.2 Segment the set
  std::vector<std::vector<int>> reflective_vector_set;
  reflective_vector_set =
      segment(reflective_point_indices, POINT_DIST_THRESHOLD);

  unsigned int num_reflective_plates =
      static_cast<unsigned int>(reflective_vector_set.size());

  RCLCPP_DEBUG(this->get_logger(), "Segmented reflective points into %d sets",
               num_reflective_plates);

  // 1.3 If only one, return with complete=False
  if (num_reflective_plates < 2) {
    RCLCPP_ERROR(this->get_logger(),
                 "Did not detect two cart reflective plates");
    response->cart_picked_up = false;
    return;
  }

  // 2. Add `cart_frame`
  // Chain the TF `odom`->`robot_front_laser_base_link` and
  // `robot_front_laser_base_link`->`cart_frame` (constructed)
  // to get `odom`->`cart_frame`

  // 2.1 Pick the two innermost points, one from each set
  // Note: The segments are sorted, externally and internally
  int left_ix, right_ix;
  left_ix = reflective_vector_set[0][num_reflective_plates - 1];
  right_ix = reflective_vector_set[1][0];

  RCLCPP_DEBUG(this->get_logger(), "left_ix = %d, right_ix = %d", left_ix,
               right_ix);

  // 2.2 Define a SAS triangle
  double left_range, right_range, sas_angle;
  left_range = last_laser_.ranges[left_ix];
  right_range = last_laser_.ranges[right_ix];
  sas_angle = (right_ix - left_ix) * last_laser_.angle_increment;

  RCLCPP_DEBUG(this->get_logger(),
               "left_range = %f, right_range = %f, angle = %f", left_range,
               right_range, sas_angle);

  // 2.3 Solve SAS triangle
  // To get x, y, and yaw of `cart_frame`
  // yaw will be used in TF listener for approach
  double x_offset, y_offset, yaw_correction;
  std::tie(x_offset, y_offset, yaw_correction) =
      solve_sas_triangle(left_range, right_range, sas_angle);

  // 2.4 Get TF odom_laser_t_
  // `odom`->`robot_front_laser_base_link`
  listen_to_odom_laser_ = true; // Let the listner look up TF
  RCLCPP_INFO(this->get_logger(),
              "Listening for `odom`->`robot_front_laser_base_link`");
  rclcpp::sleep_for(3s); // Wait for the listener

  // Wait for odom_laser_t_ to be assigned by listener
  while (odom_laser_t_.header.frame_id.compare(odom_frame_) != 0)
    ;

  RCLCPP_INFO(this->get_logger(),
              "Acquired TF `odom`->`robot_front_laser_base_link`");

  RCLCPP_DEBUG(this->get_logger(), "odom_laser_t_.header.stamp=%d sec",
               odom_laser_t_.header.stamp.sec);
  RCLCPP_DEBUG(this->get_logger(), "odom_laser_t_.header.frame_id='%s'",
               odom_laser_t_.header.frame_id.c_str());
  RCLCPP_DEBUG(this->get_logger(), "odom_laser_t_.child_frame_id='%s'",
               odom_laser_t_.child_frame_id.c_str());

  // Got odom_laser_t_, no need to listen for it any more
  listen_to_odom_laser_ = false;

  // 2.5 Chain transforms
  // Express the offsets in a new TransformStamped
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = odom_laser_t_.header.stamp;
  t.header.frame_id = laser_frame_; // `robot_front_laser_base_link`
  t.child_frame_id = cart_frame_;   // `cart_frame`
  t.transform.translation.x = x_offset;
  t.transform.translation.y = y_offset;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf2::Transform tf_odom_laser;
  // Extract translation and rotation from TransformStamped
  tf_odom_laser.setOrigin(tf2::Vector3(odom_laser_t_.transform.translation.x,
                                       odom_laser_t_.transform.translation.y,
                                       odom_laser_t_.transform.translation.z));
  tf_odom_laser.setRotation(tf2::Quaternion(
      odom_laser_t_.transform.rotation.x, odom_laser_t_.transform.rotation.y,
      odom_laser_t_.transform.rotation.z, odom_laser_t_.transform.rotation.w));

  tf2::Transform tf_laser_cart;
  // Extract translation and rotation from TransformStamped
  tf_laser_cart.setOrigin(tf2::Vector3(t.transform.translation.x,
                                       t.transform.translation.y,
                                       t.transform.translation.z));
  tf_laser_cart.setRotation(
      tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                      t.transform.rotation.z, t.transform.rotation.w));

  // Chain the transforms to get TF `odom`->`cart_frame`
  tf2::Transform tf_odom_cart = tf_odom_laser * tf_laser_cart;

  // Fill in the header of TransformStamped odom_cart_t_
  odom_cart_t_.header.stamp = this->get_clock()->now();
  odom_cart_t_.header.frame_id = odom_frame_; // `odom`
  odom_cart_t_.child_frame_id = cart_frame_;  // `cart_frame`

  // Directly assign the transform components to the TransformStamped
  odom_cart_t_.transform.translation.x = tf_odom_cart.getOrigin().x();
  odom_cart_t_.transform.translation.y = tf_odom_cart.getOrigin().y();
  odom_cart_t_.transform.translation.z = tf_odom_cart.getOrigin().z();

  odom_cart_t_.transform.rotation.x = tf_odom_cart.getRotation().x();
  odom_cart_t_.transform.rotation.y = tf_odom_cart.getRotation().y();
  odom_cart_t_.transform.rotation.z = tf_odom_cart.getRotation().z();
  odom_cart_t_.transform.rotation.w = tf_odom_cart.getRotation().w();

  // 2.6 Broadcast `cart_frame` TF
  broadcast_odom_cart_ = true; // start the broadcaster

  // 2.7 If final approach not requested, return complete=true
  // Note: The requirements are logically incomplete, so assuming
  // the service should return `complete=false` since it is not
  // completing the final approach
  if (!request->pick_up_cart) {
    RCLCPP_INFO(this->get_logger(),
                "Broadasting `cart_frame`. Final approach not requested");
    response->cart_picked_up = false;
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Broadasting `cart_frame`");
  }

  // 3. Move the frame `robot_base_link` to `cart_frame`, facing straight in

  // 3.0 Linear correction of 0.21
  RCLCPP_INFO(this->get_logger(), "Correcting for laser link");
  move(BASE_LINK_TO_LASER_LINK, MotionDirection::FORWARD, LINEAR_BASE * 0.5);

  // 3.1 Turn toward `cart_frame`
  // Note: The robot consistently veers to the left of `cart_frame` (when
  // starting from left).
  //       Apply this correction to see if turning farther will stop the target
  //       misses.
  const double DRIFT_TURN_CORRECTION = 1.8;
  RCLCPP_INFO(this->get_logger(), "Facing shelf (incl. drift correction)");
  turn(yaw_correction * DRIFT_TURN_CORRECTION, ANGULAR_BASE);

  //   rclcpp::sleep_for(3s); // Wait for `cart_frame` TF to start broadcasting

  // 3.2 Listen for TF `robot_base_link`->`cart_frame`
  RCLCPP_INFO(this->get_logger(), "Approaching shelf");
  listen_to_robot_base_cart_ = true;

  // 3.3 Wait for TF `base_cart_t_`
  // Listener will set to false when transform recorded
  while (listen_to_robot_base_cart_)
    ;

  // 3.4 Stop broadcasting `cart_frame`
  broadcast_odom_cart_ = false;

  // 3.5 Stop listening and broadcasting
  listener_timer_->cancel();
  broadcaster_timer_->cancel();

  // 3.6 Approach shelf using `base_cart_t_`
  double x_approach = base_cart_t_.transform.translation.x;
  double y_approach = base_cart_t_.transform.translation.y;
  double dist_approach = sqrt(pow(x_approach, 2.0) + pow(y_approach, 2.0));

  move(dist_approach, MotionDirection::FORWARD, LINEAR_BASE * 0.75);

  // 3.7 Straighten out
  RCLCPP_INFO(this->get_logger(), "Straightening out");
  turn(-yaw_correction, ANGULAR_BASE);

  // 4. Move the robot 30 cm forward and stop
  RCLCPP_INFO(this->get_logger(), "Moving under shelf");
  move(0.3, MotionDirection::FORWARD, LINEAR_BASE * 0.5);

  // 5. Lift the elevator to attach to the cart/shelf
  RCLCPP_INFO(this->get_logger(), "Attaching to shelf");
  std_msgs::msg::String msg;
  msg.data = 1;
  elev_up_pub_->publish(msg);

  // 6. Return success of final approach
  RCLCPP_INFO(this->get_logger(), "Final approach completed");
  response->cart_picked_up = true;
}

/**
 * @brief Simple linear motion
 * A blocking call, not a fall-through, so don't use in
 * callbacks. Because it relaculates the distance at
 * each loop, the tolerance can be smaller.
 */
void CartApproach::move(double dist_m, MotionDirection dir,
                                 double speed) {

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
  double last_angle = last_yaw_;
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

    double temp_yaw = last_yaw_;
    double delta_angle = normalize_angle(temp_yaw - last_angle);

    turn_angle += delta_angle;
    last_angle = temp_yaw;
  }
  // reached goal angle within tolerance, stop turning
  //   RCLCPP_DEBUG(this->get_logger(), "Resulting yaw %f", last_yaw_);
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
std::vector<std::vector<int>>
CartApproach::segment(std::vector<int> &v, const int threshold) {
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
 * @brief Transform listener callback
 * Used in creating `cart_frame` TF and first stage of the final approach
 */
void CartApproach::listener_cb() {
  // TODO

  // 1. ODOM->LASER
  if (listen_to_odom_laser_) {
    // listen for TF `odom`->`robot_front_laser_base_link`
    std::string parent_frame = odom_frame_;
    std::string child_frame = laser_frame_;
    // RCLCPP_DEBUG(this->get_logger(), "Listening for `%s`->`%s`",
    //              parent_frame.c_str(), child_frame.c_str());
    try {
      odom_laser_t_ = tf_buffer_->lookupTransform(parent_frame, child_frame,
                                                  tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   parent_frame.c_str(), child_frame.c_str(), ex.what());
      return;
    }

    // 2. BASE->CART
  } else if (listen_to_robot_base_cart_) {
    // NEW STRATEGY:
    // 1. Yaw toward `cart_frame` in service_cb()
    // 2. Use lookupTransform() to reach `cart_frame` in listener_cb()
    // 3. Rotate to 0 deg world coordinates in service_cb()

    // listen for TF `robot_base_link`->`cart_frame`
    std::string parent_frame = "robot_base_link";
    std::string child_frame = cart_frame_;
    RCLCPP_DEBUG(this->get_logger(), "Listening for `%s`->`%s`",
                 parent_frame.c_str(), child_frame.c_str());
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(parent_frame, child_frame,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                   parent_frame.c_str(), child_frame.c_str(), ex.what());
      return;
    }

    base_cart_t_ = t;
    listen_to_robot_base_cart_ = false;

    // // move the robot toward cart_frame
    // double error_distance = sqrt(pow(t.transform.translation.x, 2) +
    //                              pow(t.transform.translation.y, 2));
    // RCLCPP_DEBUG(this->get_logger(), "listener_cb: delta_x = %f m",
    //              t.transform.translation.x);
    // RCLCPP_DEBUG(this->get_logger(), "listener_cb: error_distance = %f m",
    //              error_distance);

    // // double error_yaw =
    // //     atan2(t.transform.translation.y, t.transform.translation.x);
    // double error_yaw =
    //     yaw_from_quaternion(t.transform.rotation.x, t.transform.rotation.y,
    //                         t.transform.rotation.z, t.transform.rotation.w);
    // RCLCPP_DEBUG(this->get_logger(), "listener_cb: error_yaw = %f rad",
    //              error_yaw);

    // geometry_msgs::msg::Twist msg;

    // // static const double kp_yaw = 0.05;
    // // msg.angular.z = (abs(error_yaw) < ANGULAR_TOLERANCE) ? 0.0 : kp_yaw *
    // // error_yaw;

    // static const double kp_distance = 0.75;
    // msg.linear.x = (error_distance < LINEAR_TOLERANCE * 2.0)
    //                    ? 0.0
    //                    : kp_distance * error_distance;

    // if (/*error_yaw < ANGULAR_TOLERANCE &&*/ error_distance <
    //     LINEAR_TOLERANCE * 2.0) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Moved robot within tolerance of `cart_frame`. Stopping");
    //   listen_to_robot_base_cart_ = false;
    // } else {
    //   RCLCPP_DEBUG(this->get_logger(),
    //                "Moving robot toward `cart_frame` (x=%f, z=%f)",
    //                msg.linear.x, msg.angular.z);
    //   vel_pub_->publish(msg);
    // }
  }
}

/**
 * @brief Transform broadcaster callback
 * Used to broadcast `cart_frame` TF
 */
void CartApproach::broadcaster_cb() {
  if (broadcast_odom_cart_) {
    // broadcast TF `odom`->`cart_frame`
    // RCLCPP_DEBUG(this->get_logger(), "Publishing cart_frame");
    odom_cart_t_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(odom_cart_t_);
  } // else no-op
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

/*
 * @brief main
 * Uses a MultiThreadedExecutor running a multi-
 * callback node with a single callback group.
 * Also sets the log severity level for the node.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CartApproach>();
  auto logger = rclcpp::get_logger(node->get_name());

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
     3 more forward and turning toward the longer range        |
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


 *******************************/