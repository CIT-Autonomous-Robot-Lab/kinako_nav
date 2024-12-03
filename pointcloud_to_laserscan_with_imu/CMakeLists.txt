cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan_with_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

include_directories(
  include
)

add_executable(pointcloud_to_laserscan_with_imu_node src/pointcloud_to_laserscan_with_imu_node.cpp)

ament_target_dependencies(pointcloud_to_laserscan_with_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  message_filters
)

install(TARGETS
  pointcloud_to_laserscan_with_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()