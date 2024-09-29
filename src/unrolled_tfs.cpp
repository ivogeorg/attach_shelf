/**
 * @file approach_service_server.cpp
 * @brief Implements a final approach service for the RB1 robot.
 * @author Ivo Georgiev
 * @version 0.3
 */

#include "attach_shelf/tf_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <tuple>
#include <utility>
#include <vector>

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
#include "tf2_ros/transform_broadcaster.h"
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
  rclcpp::TimerBase::SharedPtr init_pose_pub_timer_;

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
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
   * @class GoToFrameStages
   * @brief An enum type for the stages of motion toward a frame.
   * @see method go_to_frame() uses in motion loop
   */
  enum class GoToFrameStages { STEER_DIR, GO_STRAIGHT, ALIGN_YAW, STOP };

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

  // SIMULATOR
  std::vector<double> init_position_ = {0.020047, -0.020043, -0.019467,
                                        1.000000};
  // yaw = -0.0389291

  // LAB
  //   std::vector<double> init_position_ = {0.054368, 0.060027, -0.000433,
  //                                         1.000000};
  // yaw = -0.0073978
  /*
          {"init_position", {0.054368, 0.060027, -0.000433, 1.0000}},
  */
  const int cov_x_ix = 0;
  const int cov_y_ix = 7;
  const int cov_z_ix = 35;
  const double COV_THRESHOLD = 0.025;

  // callbacks
  inline void
  laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  inline void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  inline void amcl_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void broadcaster_cb();

  // broadcaster utilities
  void init_broadcaster_tfs();

  // navigation
  void publish_initial_pose();
  void precise_autolocalization();
  bool set_cart_approach_guidance();
  void send_tf_cart_front_midpoint(double x, double y, double yaw);
  void send_tf_cart_centerpoint(double x, double y, double yaw);

  // navigation utilities
  inline double get_current_yaw();

  /**
   * @brief Rotational-only robot motion, based on `twist.angular.z`
   * messages
   * @param rad Degrees to turn
   * @param speed Speed of rotation in rad/s
   * @param angular_tolerance Determines the accuracy of rotation
   * @param frame World or robot frame
   * Contains an internal loop and blocks until rotation complete. Publishes
   * `geometry_msgs::msg::Twist` messages to topic `cmd_vel` or equivalent.
   * Approaches the target angle depending on the sign so as never
   * overshoot.
   */
  void rotate(double rad, double speed, double angular_tolerance,
              RotationFrame frame = RotationFrame::ROBOT);

  bool
  go_to_frame(std::string origin_frame_id, std::string target_frame_id,
              MotionDirection dir, double min_lin_speed, double max_lin_speed,
              double min_ang_speed, double max_ang_speed, double lin_tolerance,
              double ang_tolerance, std::shared_ptr<tf2_ros::Buffer> tf_buff,
              rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub);

  // cart services
  bool cart_pick_up_cb();
  bool cart_set_down_cb();

  // cart service utilities
  std::vector<std::vector<int>> segment(std::vector<int> &v,
                                        const int threshold);
  std::tuple<double, double, double>
  solve_sas_triangle(double left_side, double right_side, double sas_angle);
  bool get_reflective_plate_edges(int &left_ix, int &right_ix,
                                  double &left_range, double &right_range);
  bool cart_lift();
  bool cart_drop();

  // misc utilities
  double clip_speed(double value, double min, double max);
  inline double normalize_angle(double angle);
  inline double yaw_from_quaternion(double x, double y, double z, double w);

  // testing
  const std::string tf_name_init_pos_{"tf_init_pos"};
  const std::string tf_name_load_pos_{"tf_load_pos"};
  const std::string tf_name_face_ship_pos_{"tf_face_ship_pos"};
  const std::string tf_name_ship_pos_{"tf_ship_pos"};
  const std::string root_frame_{"map"};

  // TODO: Parametrize!!!
  const std::map<std::string, std::tuple<double, double, double, double>>
      // SIMULATOR:
      poses = {
          {"init_position", {0.020047, -0.020043, -0.019467, 1.000000}},
          {"loading_position", {5.653875, -0.186439, -0.746498, 0.665388}},
          {"face_shipping_position", {2.552175, -0.092728, 0.715685, 0.698423}},
          {"shipping_position", {2.577595, 0.901998, 0.698087, 0.716013}}};

  // LAB:
  //   poses = {
  //       {"init_position", {0.054368, 0.060027, -0.000433, 1.0000}},
  //       {"loading_position", {4.472548, -0.187311, -0.694903, 0.719103}},
  //       {"face_shipping_position", {1.879749, 0.141987, 0.656541, 0.754290}},
  //       {"shipping_position", {1.878660, 1.114233, 0.706215, 0.707997}}};

  geometry_msgs::msg::TransformStamped tf_stamped_load_pos_;
  geometry_msgs::msg::TransformStamped tf_stamped_face_ship_pos_;
  geometry_msgs::msg::TransformStamped tf_stamped_ship_pos_;

  rclcpp::TimerBase::SharedPtr test_timer_;

  void test_incidence_tfs();
  void publish_laser_origin_offset();
  void send_transforms_from_poses();
};

/**
 * @brief Sole constructor initializing all ROS2 members.
 * The options are initialized with the callback group so
 * that topic subscribers can be properly added to it.
 */
CartApproach::CartApproach()
    : Node("unrolled_tfs_test_node"),
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

      // TODO: Parametrize!!!
      // SIMULATOR
      cmd_vel_topic_name_{"/diffbot_base_controller/cmd_vel_unstamped"},
      // LAB
      //   cmd_vel_topic_name_{"cmd_vel"},

      vel_pub_{this->create_publisher<geometry_msgs::msg::Twist>(
          cmd_vel_topic_name_, 1)},
      elev_up_pub_{
          this->create_publisher<std_msgs::msg::String>("/elevator_up", 1)},
      elev_down_pub_{
          this->create_publisher<std_msgs::msg::String>("/elevator_down", 1)},
      //   initialpose_pub_{
      //       this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      //           "/initialpose", rclcpp::QoS(1).transient_local())},
      initialpose_pub_{
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "/initialpose", 10)},
      have_scan_{false}, have_odom_{false}, have_amcl_pose_{false},
      odom_frame_{"odom"}, laser_frame_{"robot_front_laser_base_link"},
      cart_frame_{"cart_frame"}, tf_buffer_{std::make_shared<tf2_ros::Buffer>(
                                     this->get_clock())},
      tf_listener_{std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)},
      static_tf_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)},
      tf_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(this)},
      broadcaster_timer_{this->create_wall_timer(
          100ms, std::bind(&CartApproach::broadcaster_cb, this), cb_group_)},
      test_timer_{this->create_wall_timer(
          100ms, std::bind(&CartApproach::test_incidence_tfs, this),
          cb_group_)} {
  RCLCPP_INFO(this->get_logger(), "Cart approach testing sandbox started");
  init_broadcaster_tfs();
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
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose) {
  //   RCLCPP_DEBUG(
  //       this->get_logger(),
  //       "amcl_pose:: frame_id: %s, pos_x: %f, pos_y: %f, ori_z: %f, ori_w: %f
  //       Cov "
  //       "[x=%f, y=%f, z=%f]",
  //       last_amcl_pose_.header.frame_id.c_str(),
  //       last_amcl_pose_.pose.pose.position.x,
  //       last_amcl_pose_.pose.pose.position.y,
  //       last_amcl_pose_.pose.pose.orientation.z,
  //       last_amcl_pose_.pose.pose.orientation.w,
  //       last_amcl_pose_.pose.covariance[cov_x_ix],
  //       last_amcl_pose_.pose.covariance[cov_y_ix],
  //       last_amcl_pose_.pose.covariance[cov_z_ix]);

  last_amcl_pose_ = *amcl_pose;
  last_yaw_amcl_ = yaw_from_quaternion(
      amcl_pose->pose.pose.orientation.x, amcl_pose->pose.pose.orientation.y,
      amcl_pose->pose.pose.orientation.z, amcl_pose->pose.pose.orientation.w);
  have_amcl_pose_ = true;
}

void CartApproach::init_broadcaster_tfs() {
  // Nothing here
}

void CartApproach::broadcaster_cb() {

  if (!have_scan_) {
    RCLCPP_DEBUG(this->get_logger(),
                 "(broadcaster_cb) Waiting for laser data...");
    return;
  }

  int left_ix = -1, right_ix = -1;
  double left_range = -10.0, right_range = -10.0;
  if (!get_reflective_plate_edges(left_ix, right_ix, left_range, right_range)) {
    RCLCPP_DEBUG(this->get_logger(), "Could not detect two reflective plates");
  } else {
    std::vector<int> tf_vector = {left_ix, right_ix};

    geometry_msgs::msg::TransformStamped ts_msg;
    int front_ix = 541;
    double left_theta, right_theta, mid_theta, left_x, left_y, right_x, right_y,
        mid_x, mid_y, left_yaw, right_yaw, mid_yaw;

    // left edge
    left_theta = (front_ix - left_ix) * last_laser_.angle_increment;
    left_x = left_range * cos(left_theta); // cos(-t) = cos(t)
    left_y = left_range * sin(left_theta); // sin(-t) = -sin(t)

    // Due to the 180-deg roll of robot_front_laser_base_link
    left_y *= -1;
    left_yaw = -left_theta;

    RCLCPP_DEBUG(
        this->get_logger(),
        "(broadcaster_cb) TF \"left_edge\" at range %f and rel coords (x=%f, "
        "y=%f, yaw=%f)",
        left_range, left_x, left_y, left_yaw);
    ts_msg = tf_stamped_from_relative_coordinates_3d(
        this->get_clock()->now(), "map", "robot_front_laser_base_link",
        "left_edge", left_x, left_y, 0.0, PI_, 0.0, left_yaw, tf_buffer_);
    tf_broadcaster_->sendTransform(ts_msg);

    // right edge
    right_theta = (front_ix - right_ix) * last_laser_.angle_increment;
    right_x = right_range * cos(right_theta); // cos(-t) = cos(t)
    right_y = right_range * sin(right_theta); // sin(-t) = -sin(t)

    // Due to the 180-deg roll of robot_front_laser_base_link
    right_y *= -1;
    right_yaw = -right_theta;

    RCLCPP_DEBUG(
        this->get_logger(),
        "(broadcaster_cb) TF \"right_edge\" at range %f and rel coords (x=%f, "
        "y=%f, yaw=%f)",
        right_range, right_x, right_y, right_yaw);
    ts_msg = tf_stamped_from_relative_coordinates_3d(
        this->get_clock()->now(), "map", "robot_front_laser_base_link",
        "right_edge", right_x, right_y, 0.0, PI_, 0.0, right_yaw, tf_buffer_);
    tf_broadcaster_->sendTransform(ts_msg);

    //     // midpoint
    //     mid_theta = (front_ix - left_ix - (right_ix - left_ix) / 2.0) *
    //     last_laser_.angle_increment;

    //     mid_x = (left_range * cos(left_theta) + right_range *
    //     cos(right_theta)) / 2.0; mid_y = (left_range * sin(left_theta) +
    //     right_range * sin(right_theta)) / 2.0;

    //     // Due to the 180-deg roll of robot_front_laser_base_link
    //     mid_y *= -1;
    //     mid_yaw = -mid_theta;

    //     RCLCPP_DEBUG(this->get_logger(),
    //                  "(broadcaster_cb) TF \"midpoint\" at rel coords (x=%f, "
    //                  "y=%f, yaw=%f)",
    //                  mid_x, mid_y, mid_yaw);
    //     ts_msg = tf_stamped_from_relative_coordinates(
    //         this->get_clock()->now(), "map", "robot_front_laser_base_link",
    //         "midpoint", mid_x, mid_y, mid_yaw, tf_buffer_);
    //     tf_broadcaster_->sendTransform(ts_msg);
    //   }

    // midpoint with correct yaw
    mid_theta = (front_ix - left_ix - (right_ix - left_ix) / 2.0) *
                last_laser_.angle_increment;

    mid_x =
        (left_range * cos(left_theta) + right_range * cos(right_theta)) / 2.0;
    mid_y =
        (left_range * sin(left_theta) + right_range * sin(right_theta)) / 2.0;

    // Due to the 180-deg roll of robot_front_laser_base_link
    mid_y *= -1;
    // mid_yaw = -mid_theta;
    // mid_yaw = -(mid_theta - PI_ / 2.0);
    mid_yaw = -(-PI_ / 2.0 - get_current_yaw() - mid_theta);

    RCLCPP_DEBUG(this->get_logger(),
                 "(broadcaster_cb) TF \"midpoint\" at rel coords (x=%f, "
                 "y=%f, yaw=%f)",
                 mid_x, mid_y, mid_yaw);
    ts_msg = tf_stamped_from_relative_coordinates_3d(
        this->get_clock()->now(), "map", "robot_front_laser_base_link",
        "midpoint", mid_x, mid_y, 0.0, PI_, 0.0, mid_yaw, tf_buffer_);
    tf_broadcaster_->sendTransform(ts_msg);
  }
}

/**
 * @brief Used to send a pose to nav2 topic /initialpose
 * Needs to be started before AMCL for robustness
 */
void CartApproach::publish_initial_pose() {

  // TODO: Parametrize!!!
  geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
  initialpose.header.stamp = this->get_clock()->now();
  initialpose.header.frame_id = "map";
  initialpose.pose.pose.position.x = init_position_[0];
  initialpose.pose.pose.position.y = init_position_[1];
  initialpose.pose.pose.orientation.z = init_position_[2];
  initialpose.pose.pose.orientation.w = init_position_[3];

  rclcpp::Rate pub_rate(10);
  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  RCLCPP_DEBUG(this->get_logger(),
               "Publishing initial pose and waiting for valid /amcl_pose msg");
  while (last_amcl_pose_ == pose) {
    initialpose_pub_->publish(initialpose);
    pub_rate.sleep();
  }
}

/**
 * @brief Used for robot to autonomously and accurately localize itself.
 */
void CartApproach::precise_autolocalization() {
  //   rclcpp::Rate pub_rate(10);
  //   geometry_msgs::msg::PoseWithCovarianceStamped pose;

  //   RCLCPP_DEBUG(this->get_logger(), "Waiting for valid /amcl_pose msg");
  //   while (last_amcl_pose_ == pose) {
  //     pub_rate.sleep();
  //   }

  //   RCLCPP_DEBUG(this->get_logger(), "Localizing robot...");

  double loc_speed = 1.0;

  // if message is empty or any of the covariances is high
  while ((last_amcl_pose_.header.frame_id == "" &&
          last_amcl_pose_.pose.pose.position.x == 0.0 &&
          last_amcl_pose_.pose.pose.position.y == 0.0 &&
          last_amcl_pose_.pose.pose.orientation.z == 0.0 &&
          last_amcl_pose_.pose.pose.orientation.w == 1.0 &&
          last_amcl_pose_.pose.covariance[cov_x_ix] == 0.0 &&
          last_amcl_pose_.pose.covariance[cov_y_ix] == 0.0 &&
          last_amcl_pose_.pose.covariance[cov_z_ix] == 0.0) ||
         last_amcl_pose_.pose.covariance[cov_x_ix] > COV_THRESHOLD ||
         last_amcl_pose_.pose.covariance[cov_y_ix] > COV_THRESHOLD ||
         last_amcl_pose_.pose.covariance[cov_z_ix] > COV_THRESHOLD) {
    RCLCPP_DEBUG(this->get_logger(), "Rotating for precise localization");
    rotate(PI_, loc_speed, 0.05, RotationFrame::ROBOT);
    rotate(-2.0 * PI_, loc_speed, 0.05, RotationFrame::ROBOT);
    rotate(PI_, loc_speed, 0.05, RotationFrame::ROBOT);
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Robot autolocalized at pos_x: %f, pos_y: %f, ori_z: %f, ori_w: %f Cov "
      "[x=%f, y=%f, z=%f]",
      last_amcl_pose_.pose.pose.position.x,
      last_amcl_pose_.pose.pose.position.y,
      last_amcl_pose_.pose.pose.orientation.z,
      last_amcl_pose_.pose.pose.orientation.w,
      last_amcl_pose_.pose.covariance[cov_x_ix],
      last_amcl_pose_.pose.covariance[cov_y_ix],
      last_amcl_pose_.pose.covariance[cov_z_ix]);
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
               "(rotate) Angular difference from goal: %.2f deg",
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

  return last_yaw_amcl_;
  //   return last_yaw_odom_;
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

  // Returning x_offset, y_offset, yaw
  //   RCLCPP_DEBUG(this->get_logger(),
  //                "Solve SAS triangle return: x=%f, y=%f, yaw=%f", x, y, yaw);
  RCLCPP_DEBUG(this->get_logger(), "(solve_sas_triangle) x=%f, y=%f, yaw=%f", x,
               y, yaw);
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

void CartApproach::send_tf_cart_front_midpoint(double x, double y, double yaw) {
  static_tf_broadcaster_->sendTransform(tf_stamped_from_relative_coordinates(
      this->get_clock()->now(), "map", "robot_front_laser_base_link",
      "tf_cart_front_midpoint", x, y, yaw, tf_buffer_));

  rclcpp::sleep_for(3s);
}

// SIMULATOR
const double CART_CENTERPOINT_DEPTH = 0.45;
// LAB
// const double CART_CENTERPOINT_DEPTH = 0.30;

void CartApproach::send_tf_cart_centerpoint(double x, double y, double yaw) {
  static_tf_broadcaster_->sendTransform(tf_stamped_from_relative_coordinates(
      this->get_clock()->now(), "map", "tf_cart_front_midpoint",
      "tf_cart_centerpoint", x, y, yaw, tf_buffer_));

  rclcpp::sleep_for(3s);
}

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

void CartApproach::send_transforms_from_poses() {
  geometry_msgs::msg::PoseStamped init_pos = make_pose(
      this->get_clock()->now(), root_frame_, poses.at("init_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(init_pos, tf_name_init_pos_));

  geometry_msgs::msg::PoseStamped load_pos = make_pose(
      this->get_clock()->now(), root_frame_, poses.at("loading_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(load_pos, tf_name_load_pos_));

  geometry_msgs::msg::PoseStamped face_ship_pos =
      make_pose(this->get_clock()->now(), root_frame_,
                poses.at("face_shipping_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(face_ship_pos, tf_name_face_ship_pos_));

  geometry_msgs::msg::PoseStamped ship_pos = make_pose(
      this->get_clock()->now(), root_frame_, poses.at("shipping_position"));

  static_tf_broadcaster_->sendTransform(
      tf_stamped_from_pose_stamped(ship_pos, tf_name_ship_pos_));

  // wait for 3 seconds (2s were enough) for the TFs to register
  rclcpp::sleep_for(3s);
}

// TODO: Paremetrize !!!
// - Conditions for adding angular correction when moving linearly, incl. not to
// correct (for slow motions, e.g. under cart)
// - Loop rate
// - Tolerances
// - Speeds
bool CartApproach::go_to_frame(
    std::string origin_frame_id, std::string target_frame_id,
    MotionDirection dir, double min_lin_speed, double max_lin_speed,
    double min_ang_speed, double max_ang_speed, double lin_tolerance,
    double ang_tolerance, std::shared_ptr<tf2_ros::Buffer> tf_buff,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub) {
  RCLCPP_DEBUG(this->get_logger(), "Entering go_to_frame");

  // motion loop, looking up TF to target and adjusting motion
  rclcpp::Rate loop_rate(10); // 10 Hz
  bool done = false;
  geometry_msgs::msg::TransformStamped tf_msg;
  GoToFrameStages stage = GoToFrameStages::STEER_DIR;
  geometry_msgs::msg::Twist vel_msg;

  while (rclcpp::ok()) {
    // get transform from origin to target frame
    try {
      tf_msg = tf_buff->lookupTransform(origin_frame_id, target_frame_id,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "(go_to_frame) Could not transform %s to %s: %s",
                   origin_frame_id.c_str(), target_frame_id.c_str(), ex.what());
      break;
    }

    // define errors based on transform

    // linear distance to target
    double error_distance = std::hypotf(tf_msg.transform.translation.x,
                                        tf_msg.transform.translation.y);

    // heading toward target
    double error_yaw_dir =
        atan2(tf_msg.transform.translation.y, tf_msg.transform.translation.x);

    if (dir == MotionDirection::BACKWARD) {
      error_distance *= -1.0;

      // Subtract abs from 180 so ang_tolerance still works
      // Flip sign
      error_yaw_dir =
          (PI_ - abs(error_yaw_dir)) * ((error_yaw_dir > 0) ? -1.0 : 1.0);
    }

    // yaw alignment with target
    double error_yaw_align = normalize_angle(yaw_from_quaternion(
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z, tf_msg.transform.rotation.w));

    static const double kp_yaw = 1.0;
    static const double kp_distance = 1.0;
    double base_speed;

    // motion loop
    switch (stage) {
    case GoToFrameStages::STEER_DIR:
      if (abs(error_yaw_dir) > ang_tolerance) { // correct heading first
        vel_msg.linear.x = 0.0;
        base_speed = kp_yaw * error_yaw_dir;
        base_speed = clip_speed(base_speed, min_ang_speed, max_ang_speed);
        vel_msg.angular.z = base_speed;
      } else {
        vel_msg.angular.z = 0.0;
        stage = GoToFrameStages::GO_STRAIGHT;
      }
      break;
    case GoToFrameStages::GO_STRAIGHT:
      // Send back to STEER_DIR if heading is off
      // NOTE: This can be problematic, especially near the linear goal!
      //   if (abs(error_yaw_dir) > ang_tolerance) { // correct heading first
      //     stage = GoToFrameStages::STEER_DIR;
      //   } else
      if (abs(error_distance) > lin_tolerance) { // then go straight
        base_speed = kp_distance * error_distance;
        base_speed = clip_speed(base_speed, min_lin_speed, max_lin_speed);
        vel_msg.linear.x = base_speed;

        // adjust heading if big error and far enough from target
        if (abs(error_distance) > (2.0 * lin_tolerance) &&
            abs(error_yaw_dir) > ang_tolerance) {
          base_speed = (kp_yaw / 2.0) * error_yaw_dir;
          vel_msg.angular.z = base_speed;
        } else {
          vel_msg.angular.z = 0.0;
        }
      } else {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        stage = GoToFrameStages::ALIGN_YAW;
      }
      break;
    case GoToFrameStages::ALIGN_YAW:
      if (abs(error_yaw_align) > ang_tolerance) { // then align
        vel_msg.linear.x = 0.0;
        base_speed = kp_yaw * error_yaw_align;
        base_speed = clip_speed(base_speed, min_ang_speed, max_ang_speed);
        vel_msg.angular.z = base_speed;
      } else {
        vel_msg.angular.z = 0.0;
        stage = GoToFrameStages::STOP;
      }
      break;
    case GoToFrameStages::STOP:
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      done = true;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "(go_to_frame) Unknown motion stage");
      return false;
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "stage %d: dir: %f, dist: %f, align: %f, x=%f, z=%f",
                 static_cast<int>(stage), error_yaw_dir, error_distance,
                 error_yaw_align, vel_msg.linear.x, vel_msg.angular.z);

    // publish geometry_msgs::msg::Twist to topic /cmd_vel
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
  min = abs(min);
  max = abs(max);
  double speed = abs(value);

  if (speed > max)
    speed = max;
  if (speed < min)
    speed = min;

  return (value > 0) ? speed : -speed;
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
 * @brief Detects the reflective plates by intensity.
 * Detects the reflective plates by segmenting them by intensity. Assuming
 * two sets. If not, return failure. Otherwise, identifies the inner edges
 * and assigns their indices and ranges.
 * @return Success of failure
 */
bool CartApproach::get_reflective_plate_edges(int &left_ix, int &right_ix,
                                              double &left_range,
                                              double &right_range) {
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
    return false;
  }

  // 2. Segment the set
  std::vector<std::vector<int>> reflective_vector_set;
  reflective_vector_set =
      segment(reflective_point_indices, POINT_DIST_THRESHOLD);

  unsigned int num_reflective_plates =
      static_cast<unsigned int>(reflective_vector_set.size());

  RCLCPP_DEBUG(this->get_logger(), "Segmented reflective points into %d sets",
               num_reflective_plates);

  // 3. If only one, return with complete=False
  if (num_reflective_plates < 2) {
    RCLCPP_ERROR(this->get_logger(),
                 "Did not detect two cart reflective plates");
    return false;
  }

  // 4. Inner edges ray indices
  // Note: The segments are sorted, externally and internally
  left_ix = reflective_vector_set[0][reflective_vector_set[0].size() - 1];
  right_ix = reflective_vector_set[1][0];

  // 5. Inner edges ranges
  left_range = last_laser_.ranges[left_ix];
  right_range = last_laser_.ranges[right_ix];

  return true;
}

/**
 * @brief Publishes to the /elevator_up topic to lift the cart.
 * @return Success or failure
 * There isn't any apparent way to know if the elevator is up or not.
 */
bool CartApproach::cart_lift() {
  std_msgs::msg::String msg;
  msg.data = "";

  rclcpp::Rate pub_rate(1); // Hz
  int pub_counter = 0;
  const int PUB_TIMES = 5;
  while (rclcpp::ok()) {
    elev_up_pub_->publish(msg);
    if (++pub_counter == PUB_TIMES)
      break;
    pub_rate.sleep();
  }
  // TODO: Any way to check?
  return true;
}

/**
 * @brief Publishes to the /elevator_down topic to drop the cart.
 * @return Success or failure
 * There isn't any apparent way to know if the elevator is up or not.
 */
bool CartApproach::cart_drop() {
  std_msgs::msg::String msg;
  msg.data = "";

  rclcpp::Rate pub_rate(1); // Hz
  int pub_counter = 0;
  const int PUB_TIMES = 5;
  while (rclcpp::ok()) {
    elev_down_pub_->publish(msg);
    if (++pub_counter == PUB_TIMES)
      break;
    pub_rate.sleep();
  }
  // TODO: Any way to check?
  return true;
}

const double EDGE_RANGE_DIFF_TOLERANCE = 0.02;
const double PLATE_RANGE_MIN = 0.65;
const double PLATE_RANGE_MAX = 0.95;

const int FRONT_IX = 541;
/**
 * @brief Algorithm to set guidance for the cart approach.
 * Uses the difference in the ranges to the two inner edges of the
 * reflective plates to calculate where the "tf_cart_from_midpoint"
 * should be, after which the "tf_cart_centerpoint". Sends the TFs.
 * @return Success or failure
 */

// TODO: Parametrize the distance between tf_cart_front_midpoint and
// tf_cart_centerpoint
//       Should be related to local_costmap/fooprint
//       Lab: ~0.27 (crate is more square than the one in the sim)
//       Sim: ~0.30 (not so important because of the implementation of
//       /elevator_up subscriber)
bool CartApproach::set_cart_approach_guidance() {
  /*
      TODO
      ====
      Loop to face straight in
      Send TF "tf_cart_front_midpoint" and "tf_cart_centerpoint"
  */

  int left_edge_ix = 0, right_edge_ix = 0;
  double left_edge_range = 0.0, right_edge_range = 0.0;
  bool done = false;

  if ((done = get_reflective_plate_edges(left_edge_ix, right_edge_ix,
                                         left_edge_range, right_edge_range))) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Identified reflective plates: left edge %f (%d), right edge %f (%d)",
        left_edge_range, left_edge_ix, right_edge_range, right_edge_ix);
    // TODO: Verify this gives the correct direction (+ to the left, - to the
    // right)
    // double angle_rotate_to_face_cart =
    //     ((FRONT_IX - left_edge_ix) - (right_edge_ix - FRONT_IX)) *
    //     last_laser_.angle_increment;
    // rotate(angle_rotate_to_face_cart, 0.1, 0.05, RotationFrame::ROBOT);
    // done = get_reflective_plate_edges(left_edge_ix, right_edge_ix,
    //                                   left_edge_range, right_edge_range);
    // RCLCPP_DEBUG(
    //     this->get_logger(),
    //     "Identified reflective plates: left edge %f (%d), right edge %f
    //     (%d)", left_edge_range, left_edge_ix, right_edge_range,
    //     right_edge_ix);
    double x, y, yaw;
    std::tie(x, y, yaw) = solve_sas_triangle(left_edge_range, right_edge_range,
                                             (right_edge_ix - left_edge_ix) *
                                                 last_laser_.angle_increment);
    RCLCPP_INFO(this->get_logger(),
                "Sending TF \"tf_cart_front_midpoint\" with relative "
                "coordinates {x: %f, y: %f, yaw: %f}",
                x, y, yaw);
    // Note: The 'yaw' returned by 'solve_sas_triangle' is the angle
    //       between the height from the laser origin to the line
    //       between the plates and the line from the laser origin to
    //       the midpoint of the line. It needs to be negated to set
    //       the TF to "point straight in"
    // send_tf_cart_front_midpoint(x, y, -yaw);

    // TODO: The 'y' seems to be off, almost the other way
    send_tf_cart_front_midpoint(x, y, -yaw);

    RCLCPP_INFO(this->get_logger(),
                "Sending TF \"tf_cart_centerpoint\" with relative "
                "coordinates {x: %f, y: %f, yaw: %f}",
                CART_CENTERPOINT_DEPTH, 0.0, 0.0);
    send_tf_cart_centerpoint(CART_CENTERPOINT_DEPTH, 0.0, 0.0);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not identified 2 reflective plates");
  }

  //   rclcpp::Rate loop_rate(10);
  //   while (rclcpp::ok()) {
  //     if ((done = get_reflective_plate_edges(
  //              left_edge_ix, right_edge_ix, left_edge_range,
  //              right_edge_range))) {
  //       RCLCPP_DEBUG(
  //           this->get_logger(),
  //           "Identified reflective plates: left edge %f (%d), right edge %f
  //           (%d)", left_edge_range, left_edge_ix, right_edge_range,
  //           right_edge_ix);
  //       if (abs(left_edge_range - right_edge_range) <=
  //           EDGE_RANGE_DIFF_TOLERANCE) {
  //         RCLCPP_INFO(this->get_logger(), "Edge ranges equidistant!");
  //         break;
  //       }
  //     } else {
  //       RCLCPP_ERROR(this->get_logger(),
  //                    "Could not identified 2 reflective plates");
  //     }
  //     loop_rate.sleep();
  //   }

  // NOTE:
  // All we need is a loop on the near equality of the edge ranges with
  // repositioning attempts

  // Then solve the SAS triangle and send the "tf_cart_*" TF frames

  // NOTE:
  // The most important principle is to pass the laser through the lengthwise
  // centerline of the cart and stop (ranges to edges equal within tolerance),
  // move forward to compensate for laser offset and then rotate to get the
  // two edge indices equidistant to front ix (541) (the robot will be facing
  // straight in)
  // This can be done with rotation toward the farther edge or rotation toward
  // the closer edge and backing up.
  // For the simulator, the robot should stay between 0.65 and 0.95 from the
  // plate (need to solve the SAS triangle for more accurate values)

  /*
      Loop until edge ranges are equal
      Correct for laser origin offset
      Turn until edge range indices equidistant from front index (541)
      Send TFs
  */
  //   double left_edge_range = 0.0, right_edge_range = 1.1;
  //   int left_edge_ix, right_edge_ix;
  //   const double EDGE_RANGE_DIFF_TOLERANCE = 0.02;
  //   const double MIN_PLATE_RANGE = 0.65;
  //   while (abs(left_edge_range - right_edge_range) >
  //   EDGE_RANGE_DIFF_TOLERANCE) {
  //     // identify plates, segment, find inner edge indices and ranges
  //     // if any range is under a threshold, back up, turning toward longer
  //     // (calculate using indices) NOTE: there is an obstacle right behind
  //     the
  //     // robot, so it can't back up much recommended speeds (sim): x=0.1,
  //     z=0.5
  //     // whichever range is larger
  //     //   rotate robot to half-profile to cart (calculate using indices)
  //     //   move slowly until ranges within tolerance
  //     // send static TF to "tf_laser_origin"
  //     // move to "tf_laser_origin" (go_to_frame())
  //     // rotate until edge indices equidistant from front index (541)

  //     // NOTE: can monitor distances going forward or backward, so:

  //     //
  //   }
  return done;
}

/**
 * @brief Main service callback for picking up the cart.
 * Assuming the robot is at the loading position and facing approximately
 * the front side of the cart, the service sends a static TF of the
 * robot's exact position, uses the reflective plates to center the robot
 * to face straight in, publishes TFs for the cart's front midpoint and
 * the cart's centerpoint, uses these TFs to approach and go under the
 * cart, picks up the cart, and backs up to the TF it first sent.
 * @return Success of failure
 */
bool CartApproach::cart_pick_up_cb() {
  if (set_cart_approach_guidance()) {

    RCLCPP_INFO(this->get_logger(), "Approaching cart...");
    if (go_to_frame("robot_base_footprint", "tf_cart_front_midpoint",
                    MotionDirection::FORWARD, 0.01, 0.08, 0.25, 0.4, 0.05, 0.05,
                    tf_buffer_, vel_pub_)) {
      RCLCPP_INFO(this->get_logger(), "Completed!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not complete! Aborting!");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Going under cart...");
    if (go_to_frame("robot_base_footprint", "tf_cart_centerpoint",
                    MotionDirection::FORWARD, 0.01, 0.08, 0.25, 0.4, 0.05, 0.05,
                    tf_buffer_, vel_pub_)) {
      RCLCPP_INFO(this->get_logger(), "Completed!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not complete! Aborting!");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Attaching to cart...");
    if (cart_lift()) {
      RCLCPP_INFO(this->get_logger(), "Completed!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not complete! Aborting!");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Backing up with cart...");
    if (go_to_frame("robot_base_footprint", "tf_load_pos",
                    MotionDirection::BACKWARD, 0.01, 0.08, 0.25, 0.4, 0.05,
                    0.05, tf_buffer_, vel_pub_)) {
      RCLCPP_INFO(this->get_logger(), "Completed!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not complete! Aborting!");
      return false;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not set cart approach guidance! Aborting!");
    return false;
  }

  return true;
}

/*
bool CartApproach::go_to_frame(
    std::string origin_frame_id, std::string target_frame_id,
    MotionDirection dir, double max_lin_speed, double max_ang_speed,
    double lin_tolerance, double ang_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf_buff,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub)
*/
void CartApproach::test_incidence_tfs() {
  // TODO
  // Broadcast "laser_origin_offset"
  // Move to that frame to tune the parameters of go_to_frame

  RCLCPP_DEBUG(this->get_logger(), "test_incidence_tfs");
  this->test_timer_->cancel();
  RCLCPP_DEBUG(this->get_logger(), "Cancelled timer");

  publish_initial_pose();
  precise_autolocalization();

  // give time to nav2 stack to start and `map` frame to be published by AMCL
  rclcpp::sleep_for(10s);

  RCLCPP_INFO(this->get_logger(), "Publishing pose frames");
  send_transforms_from_poses();

  // TODO
  /*
  geometry_msgs::msg::TransformStamped tf_stamped_from_relative_coordinates(
      builtin_interfaces::msg::Time stamp, std::string root_frame_id,
      std::string origin_frame_id, std::string target_frame_id,
      double origin_to_target_x, double origin_to_target_y,
      double origin_to_target_yaw, std::shared_ptr<tf2_ros::Buffer> tf_buffer)

  */

  // NOTE: Following moved to broadcaster_cb for dynamic broadcast

  //   std::vector<std::pair<std::string, std::tuple<double, double, double>>>
  //   v; v.push_back(std::make_pair("tf_1", std::make_tuple(1.0, 1.0, 0.0)));
  //   v.push_back(std::make_pair("tf_2", std::make_tuple(1.0, -1.0, 0.0)));
  //   v.push_back(std::make_pair("tf_3", std::make_tuple(2.0, 1.0, PI_
  //   / 2.0))); v.push_back(std::make_pair("tf_4", std::make_tuple(2.0, -1.0,
  //   -PI_ / 2.0)));

  //   geometry_msgs::msg::TransformStamped ts_msg;
  //   for (auto &tf : v) {
  //     double x, y, yaw;
  //     std::tie(x, y, yaw) = tf.second;
  //     ts_msg = tf_stamped_from_relative_coordinates(
  //         this->get_clock()->now(), "map", "robot_front_laser_base_link",
  //         tf.first, x, y, yaw, tf_buffer_);
  //     static_tf_broadcaster_->sendTransform(ts_msg);
  //   }
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
