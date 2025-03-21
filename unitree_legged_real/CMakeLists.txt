cmake_minimum_required(VERSION 3.5)
project(unitree_legged_real)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Standard packages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Point Cloud Library
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Custom High-Level Kinematics Library
find_package(unitree_kinematics REQUIRED)

message(STATUS "-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(EXTRA_LIBS -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/include
)

install(
 DIRECTORY ${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib/
 DESTINATION lib
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk-master/lib)

add_executable(ros2_udp src/ros2_udp.cpp)
target_link_libraries(ros2_udp ${EXTRA_LIBS})
ament_target_dependencies(ros2_udp rclcpp ros2_unitree_legged_msgs)

add_executable(udp_high src/udp_high.cpp)
target_link_libraries(udp_high ${EXTRA_LIBS})
ament_target_dependencies(udp_high rclcpp ros2_unitree_legged_msgs)

add_executable(jsp_high src/jsp_high.cpp)
target_link_libraries(jsp_high ${EXTRA_LIBS})
ament_target_dependencies(jsp_high rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(jsp_low src/jsp_low.cpp)
target_link_libraries(jsp_low ${EXTRA_LIBS})
ament_target_dependencies(jsp_low rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(udp_low src/udp_low.cpp)
target_link_libraries(udp_low ${EXTRA_LIBS})
ament_target_dependencies(udp_low rclcpp ros2_unitree_legged_msgs sensor_msgs)

add_executable(ros2_walk_example src/ros2_walk_example.cpp)
target_link_libraries(ros2_walk_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_example rclcpp ros2_unitree_legged_msgs)

add_executable(ros2_walk_experiments src/ros2_walk_experiments.cpp)
target_link_libraries(ros2_walk_experiments ${EXTRA_LIBS})
ament_target_dependencies(ros2_walk_experiments rclcpp ros2_unitree_legged_msgs)

# Add executable for custom waypoint navigation node
add_executable(unitree_waypoint src/unitree_waypoint.cpp)
target_link_libraries(unitree_waypoint unitree_kinematics::unitree_kinematics ${EXTRA_LIBS})
ament_target_dependencies(unitree_waypoint rclcpp std_msgs std_srvs tf2 tf2_ros ros2_unitree_legged_msgs)

# Add executable for occupancy grid publisher node
add_executable(occupancy_grid_publisher src/occupancy_grid_publisher.cpp)
target_link_libraries(occupancy_grid_publisher ${EXTRA_LIBS})
target_link_libraries(occupancy_grid_publisher ${PCL_LIBRARIES}) # If using PCL
ament_target_dependencies(occupancy_grid_publisher rclcpp nav_msgs sensor_msgs pcl_conversions)

# Add executable for costmap publisher node
#add_executable(costmap_publisher src/costmap_publisher.cpp)
#target_link_libraries(costmap_publisher ${EXTRA_LIBS})
#target_link_libraries(costmap_publisher ${PCL_LIBRARIES}) # If using PCL
#ament_target_dependencies(costmap_publisher rclcpp nav_msgs nav2_msgs sensor_msgs pcl_conversions)

# Add executable for hello world publisher
# add_executable(hello_world_publisher src/ros2_test.cpp)
# target_link_libraries(hello_world_publisher ${EXTRA_LIBS})
# ament_target_dependencies(hello_world_publisher std_msgs rclcpp)

add_executable(ros2_position_example src/ros2_position_example.cpp)
target_link_libraries(ros2_position_example ${EXTRA_LIBS})
ament_target_dependencies(ros2_position_example rclcpp ros2_unitree_legged_msgs)

add_executable(custom_gait src/custom_gait.cpp)
target_link_libraries(custom_gait ${EXTRA_LIBS})
ament_target_dependencies(custom_gait rclcpp ros2_unitree_legged_msgs)

install(TARGETS
    ros2_udp
    udp_high
    jsp_high
    ros2_walk_example
    ros2_walk_experiments
    unitree_waypoint
    ros2_position_example
    custom_gait
    udp_low
    jsp_low
    occupancy_grid_publisher
    # costmap_publisher
    # hello_world_publisher
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
