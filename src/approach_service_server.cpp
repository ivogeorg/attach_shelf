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

#include "attach_shelf/srv/go_to_loading.hpp"
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
using GoToLoading = attach_shelf::srv::GoToLoading;
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
 * @class ApproachServiceServer
 * @brief A multi-callback node implementing the final approach service.
 */
class ApproachServiceServer : public rclcpp::Node {
private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  CustomSubscriptionOptions topic_pub_sub_options_;

  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elev_up_pub_;

  bool have_scan_;
  bool have_odom_;
  sensor_msgs::msg::LaserScan last_laser_;
  nav_msgs::msg::Odometry last_odom_;

  const double REFLECTIVE_INTENSITY_VALUE = 8000; // for reflective points
  const int POINT_DIST_THRESHOLD = 10;            // for point set segmentation

  std::string odom_frame_;
  std::string laser_frame_;
  std::string cart_frame_;

  bool broadcast_odom_cart_;  // connecting cart_frame to TF tree
  bool listen_to_odom_laser_; // odom_laser_t_ serves as basis for odom_cart_t_
  bool listen_to_robot_base_cart_; // laser_cart_t_ serves in the final approach

  geometry_msgs::msg::TransformStamped odom_cart_t_;
  geometry_msgs::msg::TransformStamped odom_laser_t_;
  geometry_msgs::msg::TransformStamped laser_cart_t_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; // TODO: Consider StaticTL
  rclcpp::TimerBase::SharedPtr listener_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr broadcaster_timer_;

  const double LINEAR_TOLERANCE = 0.04;  // m
  const double ANGULAR_TOLERANCE = 0.07; // rad
  const double LINEAR_BASE = 0.075;      // m/s
  const double ANGULAR_BASE = 0.04;      // rad/s

  /**
   * @class ApproachStage
   * @brief An enum type for the stages of the final approach.
   */
  enum class ApproachStage {
    LINEAR_CORRECTION,
    INIT_ROTATION,
    LINEAR_APPROACH,
    FINAL_ROTATION
  };
  ApproachStage approach_stage_;

  double yaw_correction_;

  /**
   * @class MotionDirection
   * @brief An enum type for the directions of motion.
   * @see method move() uses as parameter
   */
  enum class MotionDirection { FORWARD, BACKWARD };

public:
  ApproachServiceServer();
  ~ApproachServiceServer() = default;

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

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response);
  std::vector<std::vector<int>> segment(std::vector<int> &v,
                                        const int threshold);
  void listener_cb();
  void broadcaster_cb();
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);
  void move(double dist_m, MotionDirection dir);
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
ApproachServiceServer::ApproachServiceServer()
    : Node("approach_service_server_node"),
      cb_group_{
          this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)},
      topic_pub_sub_options_{cb_group_},
      srv_{this->create_service<GoToLoading>(
          "approach_shelf",
          std::bind(&ApproachServiceServer::service_callback, this, _1, _2),
          rmw_qos_profile_services_default, cb_group_)},
      scan_sub_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10,
          std::bind(&ApproachServiceServer::laser_scan_callback, this, _1),
          topic_pub_sub_options_)},
      odom_sub_{this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", 10,
          std::bind(&ApproachServiceServer::odometry_callback, this, _1),
          topic_pub_sub_options_)},
      vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 1)},
      elev_up_pub_{
          this->create_publisher<std_msgs::msg::String>("/elevator_up", 1)},
      have_scan_{false}, have_odom_{false}, odom_frame_{"odom"},
      laser_frame_{"robot_front_laser_base_link"}, cart_frame_{"cart_frame"},
      broadcast_odom_cart_{false}, listen_to_odom_laser_{false},
      listen_to_robot_base_cart_{false},
      tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)},
      listener_timer_{this->create_wall_timer(
          100ms, std::bind(&ApproachServiceServer::listener_cb, this),
          cb_group_)},
      tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
      broadcaster_timer_{this->create_wall_timer(
          100ms, std::bind(&ApproachServiceServer::broadcaster_cb, this),
          cb_group_)},
      approach_stage_{ApproachStage::LINEAR_CORRECTION} {
  // TODO: Change to LINEAR_CORRECTION if included, either here or in
  // service_cb()
  RCLCPP_INFO(this->get_logger(), "Server of /approach_shelf service started");
}

/**
 * @brief The callback for the /approach_shelf service.
 * It is called after the pre-approach is complete and
 * implements the final approach and the attachment of
 * the shelf/crate to the RB1 robot.
 * @param request Service request (attach_to_shelf)
 * @param response Service response (complete)
 */
void ApproachServiceServer::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Recived request with attach_to_service='%s'",
              request->attach_to_shelf ? "true" : "false");

  // Wait for /scan and /odom callbacks to be called
  while (!have_scan_ || !have_odom_) {
    RCLCPP_INFO(this->get_logger(), "Waiting for data");
    rclcpp::sleep_for(100ms);
  }

  // 1. Detect reflective plates.
  std::vector<int> reflective_point_indices;
  for (int i = 0; i < static_cast<int>(last_laser_.intensities.size()); ++i)
    if (last_laser_.intensities[i] == REFLECTIVE_INTENSITY_VALUE)
      reflective_point_indices.push_back(i);

  // 1.1 If none, return with complete=False
  if (reflective_point_indices.size() == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Did not detect any cart reflective plates");
    response->complete = false;
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
    response->complete = false;
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
  double x_offset, y_offset;
  std::tie(x_offset, y_offset, yaw_correction_) =
      solve_sas_triangle(left_range, right_range, sas_angle);

  // 2.4 Get TF odom_laser_t_
  // `odom`->`robot_front_laser_base_link`
  listen_to_odom_laser_ = true; // Let the listner look up TF
  rclcpp::sleep_for(3s);        // Wait for the listener

  // Wait for odom_laser_t_ to be assigned by listener
  while (odom_laser_t_.header.frame_id.compare(odom_frame_) != 0)
    ;

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
  if (!request->attach_to_shelf) {
    RCLCPP_INFO(this->get_logger(),
                "Broadasting `cart_frame`. Final approach not requested");
    response->complete = false;
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Broadasting `cart_frame`");
  }

  // 3. Move the frame `robot_base_link` to `cart_frame`, facing straight in
  rclcpp::sleep_for(3s); // Wait for `cart_frame` TF to start broadcasting

  // 3.1 Listen for TF `robot_base_link`->`cart_frame`
  RCLCPP_INFO(this->get_logger(), "Approaching shelf");
  listen_to_robot_base_cart_ = true;

  // 3.2 Wait for robot to move to `cart_frame`
  while (listen_to_robot_base_cart_)
    ;

  // 3.3 Stop broadcasting `cart_frame`
  broadcast_odom_cart_ = false;

  // 3.4 Stop listening and broadcasting
  listener_timer_->cancel();
  broadcaster_timer_->cancel();

  // 4. Move the robot 30 cm forward and stop
  RCLCPP_INFO(this->get_logger(), "Moving under shelf");
  move(0.3, MotionDirection::FORWARD);

  // 5. Lift the elevator to attach to the cart/shelf
  RCLCPP_INFO(this->get_logger(), "Attaching to shelf");
  std_msgs::msg::String msg;
  msg.data = 1;
  elev_up_pub_->publish(msg);

  // 6. Return success of final approach
  RCLCPP_INFO(this->get_logger(), "Final approach completed");
  response->complete = true;
}

/**
 * @brief Simple linear motion
 */
void ApproachServiceServer::move(double dist_m, MotionDirection dir) {

  // Use straight /odom)
  geometry_msgs::msg::Twist twist;
  twist.linear.x =
      (dir == MotionDirection::FORWARD) ? LINEAR_BASE : -LINEAR_BASE;
  double dist = 0.0, x, y;
  while (abs(dist + LINEAR_TOLERANCE) < dist_m) {
    vel_pub_->publish(twist);
    rclcpp::sleep_for(100ms);

    x = last_odom_.pose.pose.position.x;
    y = last_odom_.pose.pose.position.y;
    dist += sqrt(pow(x, 2.0) + pow(y, 2.0));
  }
}

/**
 * @brief Segments a sorted linear collection of numbers
 * @param v a vector of integers to segment
 * @param threshold the minimum distance between segments
 * @return A vector of vectors each holding a segment
 */
std::vector<std::vector<int>>
ApproachServiceServer::segment(std::vector<int> &v, const int threshold) {
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
void ApproachServiceServer::listener_cb() {
  // TODO

  // 1. ODOM->LASER
  if (listen_to_odom_laser_) {
    // listen for TF `odom`->`robot_front_laser_base_link`
    std::string parent_frame = odom_frame_;
    std::string child_frame = laser_frame_;
    RCLCPP_DEBUG(this->get_logger(), "Listening for `%s`->`%s`",
                 parent_frame.c_str(), child_frame.c_str());
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
    // TODO:
    // Break into sections:
    // - forward the length of the horizontal distance between base and laser
    // - rotate toward cart_frame so there is no yaw to correct
    // - forward toward cart_frame
    // - rotate to perpendicular to cart_frame

    // NOTE: These have to be fall-through

    // TODO: Should factor in the result of "degrees" argument,
    //       that is, the robot yaw at the beginning of the 
    //       approach should be taken into consideration to 
    //       make sure the robot approaches without bumping into
    //       the cart and also moves straight in underneath it

    switch (approach_stage_) {
    case ApproachStage::LINEAR_CORRECTION:
        // get `robot_base_link` to move forward the length
        // of the x offset of `robot_front_laser_base_link`
        break;
    case ApproachStage::INIT_ROTATION:
        // get robot to face along a straight line to 
        // `cart_frame` using yaw_correction_
        break;
    case ApproachStage::LINEAR_APPROACH:
        // get `robot_base_link` to coincide with `cart_frame`
        // (translational x and y only) by using TF between them
        break;
    case ApproachStage::FINAL_ROTATION:
        // get the robot to face straight in along the length
        // of the shelf (should be -yaw_correction_)
        break;
    }


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

    // move the robot toward cart_frame
    double error_distance = sqrt(pow(t.transform.translation.x, 2) +
                                 pow(t.transform.translation.y, 2));

    // double error_yaw =
    //     atan2(t.transform.translation.y, t.transform.translation.x);

    geometry_msgs::msg::Twist msg;

    // static const double kp_yaw = 0.05;
    // DEBUG
    // msg.angular.z = (error_yaw < ANGULAR_TOLERANCE) ? 0.0 : kp_yaw *
    // error_yaw; end DEBUG

    static const double kp_distance = 0.5;
    msg.linear.x = (error_distance < LINEAR_TOLERANCE)
                       ? 0.0
                       : kp_distance * error_distance;

    // DEBUG
    // clamp
    if (msg.linear.x < 0.075)
      msg.linear.x = 0.075;
    // end DEBUG

    if (/*error_yaw < ANGULAR_TOLERANCE &&*/ error_distance <
        LINEAR_TOLERANCE) {
      RCLCPP_INFO(this->get_logger(),
                  "Moved robot within tolerance of `cart_frame`. Stopping");
      listen_to_robot_base_cart_ = false;
    } else {
      RCLCPP_DEBUG(this->get_logger(),
                   "Moving robot toward `cart_frame` (x=%f, z=%f)",
                   msg.linear.x, msg.angular.z);
      vel_pub_->publish(msg);
    }
  }
}

/**
 * @brief Transform broadcaster callback
 * Used to broadcast `cart_frame` TF
 */
void ApproachServiceServer::broadcaster_cb() {
  if (broadcast_odom_cart_) {
    // broadcast TF `odom`->`cart_frame`
    RCLCPP_DEBUG(this->get_logger(), "Publishing cart_frame");
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
ApproachServiceServer::solve_sas_triangle(double l_side, double r_side,
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

  RCLCPP_DEBUG(this->get_logger(), "Given:");
  RCLCPP_DEBUG(this->get_logger(), "r_side=%f, l_side=%f, sas_angle=%f", r_side,
               l_side, sas_angle);
  RCLCPP_DEBUG(this->get_logger(), "Found:");
  RCLCPP_DEBUG(this->get_logger(), "rsa=%f, lsa=%f, frame_length=%f", rsa, lsa,
               frame_length);

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

  RCLCPP_DEBUG(this->get_logger(), "Given:");
  RCLCPP_DEBUG(this->get_logger(), "l_side=%f, rsa=%f, right angle", l_side,
               rsa);
  RCLCPP_DEBUG(this->get_logger(), "Found:");
  RCLCPP_DEBUG(this->get_logger(), "x=%f, y=%f, h_angle=%f", x, y, h_angle);

  // Note: See diagram above for sign of y
  if (l_side < r_side)
    y *= -1;

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
  RCLCPP_DEBUG(this->get_logger(), "h_angle=%f, b_angle=%f", h_angle, b_angle);
  yaw = h_angle - b_angle;

  // Note: See diagram for sign of initial yaw
  if (r_side > l_side)
    yaw *= -1;

  /* NOTE:
     1. x and y are for positioning crate_frame, yaw is not used in TF
     2. yaw is for the robot to face crate_frame, and then rotate back

     Essentially the TransformStamped's rotation will be ignored
  */

  // TODO: Test thoroughly

  // Returning x_offset, y_offset, yaw
  RCLCPP_DEBUG(this->get_logger(), "Returning: x=%f, y=%f, yaw=%f", x, y, yaw);
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

  auto node = std::make_shared<ApproachServiceServer>();
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

  return 0;
}