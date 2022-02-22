#include "urban_road_filter/DataStructures.hpp"

/*A paraméterek beállítása.*/
void paramsCallback(lidar_filters_pkg::LidarFiltersConfig &config, uint32_t level){
    fixedFrame = config.fixed_frame;
    topicName = config.topic_name;
    x_zero_method = config.x_zero_method;
    y_zero_method = config.z_zero_method;
    star_shaped_method  = config.star_shaped_method ;
    blind_spots = config.blind_spots;
    xDirection = config.xDirection;
    interval = config.interval;
    curbHeight = config.curb_height;
    curbPoints = config.curb_points;
    beamZone = config.beamZone;
    angleFilter1 = config.cylinder_deg_x;
    angleFilter2 = config.cylinder_deg_z;
    angleFilter3 = config.sector_deg;
    min_X = config.min_x;
    max_X = config.max_x;
    min_Y = config.min_y;
    max_Y = config.max_y;
    min_Z = config.min_z;
    max_Z = config.max_z;
    dmin_param = config.dmin_param;
    kdev_param = config.kdev_param;
    kdist_param = config.kdist_param;
    polysimp_allow = config.simple_poly_allow;
    polysimp = config.poly_s_param;
    zavg_allow = config.poly_z_avg_allow;
    polyz = config.poly_z_manual;
    ROS_INFO("Updated params %s", ros::this_node::getName().c_str());
}

/*MAIN*/
int main(int argc, char **argv)
{
    /*Az ROS inicializálása.*/
    ros::init(argc, argv, "urban_road_filt");
    ROS_INFO("Initializing %s", ros::this_node::getName().c_str());

    /*A GUI felülethez szükséges sorok.*/
    dynamic_reconfigure::Server<lidar_filters_pkg::LidarFiltersConfig> server;
    dynamic_reconfigure::Server<lidar_filters_pkg::LidarFiltersConfig>::CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    /*NodeHandle*/
    ros::NodeHandle nh;
    Detector detector(&nh);

    ros::spin();
    return 0;
}