cmake_minimum_required(VERSION 3.8)
project(odometry_transformer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(tf2_ros REQUIRED)

# add_executable(odometry_transformer src/odometry_transformer.cpp)
# add_executable(px4_to_pose src/px4_to_pose.cpp)
# ament_target_dependencies(odometry_transformer rclcpp nav_msgs px4_ros_com px4_msgs geometry_msgs)
# ament_target_dependencies(px4_to_pose rclcpp px4_ros_com px4_msgs geometry_msgs)

add_executable(px4_tf_broadcaster src/px4_tf_broadcaster.cpp)
ament_target_dependencies(px4_tf_broadcaster rclcpp px4_msgs px4_ros_com sensor_msgs nav_msgs tf2_ros geometry_msgs)

install(TARGETS px4_tf_broadcaster 
    DESTINATION lib/${PROJECT_NAME})

# install(TARGETS odometry_transformer
#     DESTINATION lib/${PROJECT_NAME})

# install(TARGETS px4_to_pose 
#     DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
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