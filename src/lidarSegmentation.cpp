#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>
#include <ros/ros.h>
// marker 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
//#include <lidar_filters_pkg/MyParamsConfig.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarSegmentation");
    ROS_INFO("Initializing %s", ros::this_node::getName().c_str());
    ros::spin();
    return 0;
}