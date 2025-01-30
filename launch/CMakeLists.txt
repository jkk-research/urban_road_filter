cmake_minimum_required(VERSION 3.8)
project(urban_road_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(dynamic_reconfigure REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lidar_road src/lidar_segmentation.cpp src/main.cpp src/star_shaped_search.cpp 
                          src/x_zero_method.cpp src/z_zero_method.cpp src/blind_spots.cpp)
ament_target_dependencies(lidar_road rclcpp sensor_msgs pcl_conversions pcl_ros dynamic_reconfigure)

install(TARGETS lidar_road
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()