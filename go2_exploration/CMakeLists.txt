cmake_minimum_required(VERSION 3.8)
project(go2_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(go2_exploration_interfaces REQUIRED)
find_package(unitree_go2_nav_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(go2Exploration src/explore.cpp)
ament_target_dependencies(go2Exploration
  rclcpp rclcpp_action std_srvs geometry_msgs tf2 tf2_geometry_msgs nav2_msgs sensor_msgs nav_msgs unitree_go2_nav_interfaces go2_exploration_interfaces visualization_msgs
)

install(TARGETS
  go2Exploration
  DESTINATION lib/${PROJECT_NAME}
)

# May need later
# install(DIRECTORY
#   launch config
#   DESTINATION share/${PROJECT_NAME}
# )


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
