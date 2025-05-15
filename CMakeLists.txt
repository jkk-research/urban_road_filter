cmake_minimum_required(VERSION 3.8)
project(urban_road_filter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add compile options
add_compile_options(-Wall -Wextra -O2)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create the executable
add_executable(urban_road_filter 
  src/lidar_segmentation.cpp 
  src/main.cpp 
  src/star_shaped_search.cpp 
  src/x_zero_method.cpp 
  src/z_zero_method.cpp 
  src/blind_spots.cpp
)

# Link the executable
ament_target_dependencies(urban_road_filter 
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(urban_road_filter ${PCL_LIBRARIES})

# Install the executable
install(TARGETS
  urban_road_filter
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})


ament_package()