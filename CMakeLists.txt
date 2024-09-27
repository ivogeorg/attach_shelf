cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/GoToLoading.srv"
	"srv/PickUpCart.srv"
	# DEPENDENCIES sensor_msgs geometry_msgs
)

# Pre-approach
add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node rclcpp geometry_msgs sensor_msgs nav_msgs)

# Pre-approach 2
add_executable(pre_approach_v2_node src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2_node rclcpp geometry_msgs sensor_msgs nav_msgs)

# Laser poker
add_executable(laser_scanner_poker_node src/laser_scanner_poker.cpp)
ament_target_dependencies(laser_scanner_poker_node rclcpp sensor_msgs)

# Intensity poker
add_executable(intensity_poker_node src/intensity_poker.cpp)
ament_target_dependencies(intensity_poker_node rclcpp sensor_msgs)

# Approach service server
add_executable(approach_shelf_service_server_node src/approach_service_server.cpp)
ament_target_dependencies(approach_shelf_service_server_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros std_msgs)

# Cart pick up
add_executable(cart_pick_up_service_server_node src/cart_pick_up.cpp)
ament_target_dependencies(cart_pick_up_service_server_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros std_msgs)

# TF utils
add_executable(cart_approach_test_node src/cart_approach.cpp src/tf_utils.cpp)
target_include_directories(cart_approach_test_node PUBLIC include)
ament_target_dependencies(cart_approach_test_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros std_msgs)

# Cart front midpoint test
add_executable(cart_front_midpoint_test_node src/tf_relative_to_laser.cpp src/tf_utils.cpp)
target_include_directories(cart_front_midpoint_test_node PUBLIC include)
ament_target_dependencies(cart_front_midpoint_test_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros std_msgs)

# Relative TFs test
add_executable(tf_relative_to_laser_test_node src/tf_relative_to_laser.cpp src/tf_utils.cpp)
target_include_directories(tf_relative_to_laser_test_node PUBLIC include)
ament_target_dependencies(tf_relative_to_laser_test_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros std_msgs)

# Static-publish TFs
add_executable(static_tf_broadcaster_from_pose src/static_publish_frames.cpp src/tf_utils.cpp)
target_include_directories(static_tf_broadcaster_from_pose PUBLIC include)
ament_target_dependencies(static_tf_broadcaster_from_pose rclcpp geometry_msgs tf2 tf2_ros)

# Get the typesupport target name
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against the generated interface library
# NOTE: CMake, no keywords (PRIVATE, PUBLIC) or all keywords (can't mix)
target_link_libraries(pre_approach_v2_node ${typesupport_target})
target_link_libraries(approach_shelf_service_server_node ${typesupport_target})
target_link_libraries(cart_pick_up_service_server_node ${typesupport_target})

install(TARGETS
			pre_approach_node
			pre_approach_v2_node
			laser_scanner_poker_node
			intensity_poker_node
			approach_shelf_service_server_node
			cart_pick_up_service_server_node
			static_tf_broadcaster_from_pose
			cart_approach_test_node
			tf_relative_to_laser_test_node
			cart_front_midpoint_test_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
			launch
			config
			rviz
	DESTINATION share/${PROJECT_NAME}
)


ament_package()
