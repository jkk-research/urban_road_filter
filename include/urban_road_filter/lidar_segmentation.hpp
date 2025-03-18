#ifndef URBAN_ROAD_FILTER_LIDAR_SEGMENTATION_HPP_
#define URBAN_ROAD_FILTER_LIDAR_SEGMENTATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "urban_road_filter/data_structures.hpp"

class Detector : public rclcpp::Node {
public:
    explicit Detector(const rclcpp::NodeOptions& options);

    void filtered(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void beam_init();
    int partition(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);
    void quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);
    void starShapedSearch(std::vector<Point2D>& array2D);
    void xZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray);
    void zZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray);
    void blindSpots(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray, std::vector<float>& maxDistance);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_road;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_high;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_box;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pobroad;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
};

#endif  //