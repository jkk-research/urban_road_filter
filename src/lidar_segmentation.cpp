#include "urban_road_filter/lidar_segmentation.hpp"
#include "urban_road_filter/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"

namespace params {
    std::string fixedFrame;
    std::string topicName;
    bool x_zero_method, z_zero_method, star_shaped_method;
    bool blind_spots;
    int xDirection;
    float interval;
    float curbHeight;
    int curbPoints;
    float beamZone;
    float angleFilter1;
    float angleFilter2;
    float angleFilter3;
    float min_X, max_X, min_Y, max_Y, min_Z, max_Z;
    float kdev_param;
    float kdist_param;
    bool starbeam_filter;
    int dmin_param;
    bool polysimp_allow;
    bool zavg_allow;
    float polysimp;
    float polyz;
} // namespace params

Detector::Detector(const rclcpp::NodeOptions& options)
    : Node("detector", options)
{
    // Declare and get parameters (example)
    this->declare_parameter<std::string>("topic_name", "/points_raw");
    params::topicName = this->get_parameter("topic_name").as_string();

    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params::topicName,
        10,  // QoS
        std::bind(&Detector::filtered, this, std::placeholders::_1));

    pub_road = this->create_publisher<sensor_msgs::msg::PointCloud2>("road", 1);
    pub_high = this->create_publisher<sensor_msgs::msg::PointCloud2>("curb", 1);
    pub_box = this->create_publisher<sensor_msgs::msg::PointCloud2>("roi", 1);
    pub_pobroad = this->create_publisher<sensor_msgs::msg::PointCloud2>("road_probably", 1);
    pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("road_marker", 1);

    Detector::beam_init();

    RCLCPP_INFO(this->get_logger(), "Detector node ready");
}

void Detector::filtered(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    // Example: publish the point cloud to one of the topics
    // You would perform your filtering logic here
    pub_road->publish(*cloud_msg);
}

void Detector::beam_init() {
    // Your beam initialization code here
}

int Detector::partition(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high) {
    float pivot = array3D[arc][high].alpha;
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++) {
        if (array3D[arc][j].alpha < pivot) {
            i++;
            std::swap(array3D[arc][i], array3D[arc][j]);
        }
    }
    std::swap(array3D[arc][i + 1], array3D[arc][high]);
    return (i + 1);
}

void Detector::quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high) {
    if (low < high) {
        int pi = partition(array3D, arc, low, high);
        quickSort(array3D, arc, low, pi - 1);
        quickSort(array3D, arc, pi + 1, high);
    }
}

void Detector::starShapedSearch(std::vector<Point2D>& array2D) {
    // Your code for starShapedSearch
}

void Detector::xZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray) {
    // Your code for xZeroMethod
}

void Detector::zZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray) {
    // Your code for zZeroMethod
}

void Detector::blindSpots(std::vector<std::vector<Point3D>>& array3D, int index, std::vector<int>& indexArray, std::vector<float>& maxDistance) {
    // Your code for blindSpots
}
