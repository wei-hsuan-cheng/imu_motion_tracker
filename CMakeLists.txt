cmake_minimum_required(VERSION 3.8)
project(imu_motion_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(vision_msgs REQUIRED) 

# Dependencies list
set(dependencies
    ament_cmake
    geometry_msgs
    visualization_msgs
    rcl_interfaces
    rclcpp
    rcutils
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_msgs
    vision_msgs
    Eigen3 
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# --------------------
# Find Liquid-DSP manually
# --------------------
find_path(LIQUID_DSP_INCLUDE_DIR liquid/liquid.h)
if(NOT LIQUID_DSP_INCLUDE_DIR)
  message(FATAL_ERROR "Liquid DSP include directory not found. Please install Liquid-DSP or specify its path.")
endif()
include_directories(${LIQUID_DSP_INCLUDE_DIR})

find_library(LIQUID_DSP_LIBRARY NAMES liquid)
if(NOT LIQUID_DSP_LIBRARY)
  message(FATAL_ERROR "Liquid DSP library not found. Please install Liquid-DSP or specify its path.")
endif()

# Source files
set(IMU_MOTION_TRACKER_SOURCES 
    src/imu_motion_tracker.cpp
    include/filters/BandPassFilter.cpp
)

# Add executable
add_executable(imu_motion_tracker ${IMU_MOTION_TRACKER_SOURCES})

# Link libraries (link Liquid-DSP library as well)
target_link_libraries(imu_motion_tracker ${LIQUID_DSP_LIBRARY})

# Add dependencies
ament_target_dependencies(imu_motion_tracker ${dependencies})

# Define installation rules
install(TARGETS
  imu_motion_tracker

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY 
  meshes/ 
  DESTINATION share/${PROJECT_NAME}/meshes)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()