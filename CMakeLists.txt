cmake_minimum_required(VERSION 3.8)
project(rtabmap_drone_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# examples/offboard/offboard_control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs sensor_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

add_executable(ros_odometry_to_vehicle_odometry src/ros_odometry_to_vehicle_odometry.cpp)
ament_target_dependencies(ros_odometry_to_vehicle_odometry rclcpp px4_msgs px4_ros_com tf2_ros tf2_eigen nav_msgs)
install(TARGETS ros_odometry_to_vehicle_odometry DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME}
)

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



ament_package()
