cmake_minimum_required(VERSION 3.5)
project(urban_road_filter)

# Find dependencies
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Ensure this is present!
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Set the C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}  # Ensure this is present!
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

# Add executable
add_executable(urban_road_filter_node src/main.cpp src/lidar_segmentation.cpp src/z_zero_method.cpp)

# Link libraries
ament_target_dependencies(urban_road_filter_node
  rclcpp
  visualization_msgs
  sensor_msgs # Ensure this is present!
  pcl_conversions
  pcl_ros
)

install(TARGETS
  urban_road_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
