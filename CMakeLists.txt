cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
	# DEPENDENCIES sensor_msgs geometry_msgs
)

add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node rclcpp geometry_msgs sensor_msgs nav_msgs)

add_executable(pre_approach_v2_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_v2_node rclcpp geometry_msgs sensor_msgs nav_msgs)

add_executable(laser_scanner_poker_node src/laser_scanner_poker.cpp)
ament_target_dependencies(laser_scanner_poker_node rclcpp sensor_msgs)

add_executable(intensity_poker_node src/intensity_poker.cpp)
ament_target_dependencies(intensity_poker_node rclcpp sensor_msgs)

add_executable(approach_shelf_service_server_node src/approach_service_server.cpp)
ament_target_dependencies(approach_shelf_service_server_node rclcpp geometry_msgs nav_msgs sensor_msgs)

# Get the typesupport target name
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against the generated interface library
# NOTE: CMake, no keywords (PRIVATE, PUBLIC) or all keywords (can't mix)
target_link_libraries(approach_shelf_service_server_node ${typesupport_target})

install(TARGETS
			pre_approach_node
			pre_approach_v2_node
			laser_scanner_poker_node
			intensity_poker_node
			approach_shelf_service_server_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
			launch
	DESTINATION share/${PROJECT_NAME}
)


ament_package()
