cmake_minimum_required(VERSION 3.5)
project(imu_motion_tracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(imu_motion_tracker src/imu_motion_tracker.cpp)
ament_target_dependencies(imu_motion_tracker
  rclcpp
  geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  imu_motion_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  meshes/ 
  DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()

