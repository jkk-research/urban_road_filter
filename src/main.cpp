#include "urban_road_filter/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create detector node
  auto node = std::make_shared<Detector>();
  
  // Spin the node
  rclcpp::spin(node);
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}