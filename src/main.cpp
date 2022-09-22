#include "urban_road_filter/data_structures.hpp"

/*parameter setup*/
void paramsCallback(urban_road_filter::LidarFiltersConfig &config, uint32_t level){
    params::fixedFrame = config.fixed_frame;
    params::topicName = config.topic_name;
    params::x_zero_method = config.x_zero_method;
    params::z_zero_method = config.z_zero_method;
    params::star_shaped_method  = config.star_shaped_method ;
    params::blind_spots = config.blind_spots;
    params::xDirection = config.xDirection;
    params::interval = config.interval;
    params::curbHeight = config.curb_height;
    params::curbPoints = config.curb_points;
    params::beamZone = config.beamZone;
    params::angleFilter1 = config.cylinder_deg_x;
    params::angleFilter2 = config.cylinder_deg_z;
    params::angleFilter3 = config.curb_slope_deg;
    params::min_X = config.min_x;
    params::max_X = config.max_x;
    params::min_Y = config.min_y;
    params::max_Y = config.max_y;
    params::min_Z = config.min_z;
    params::max_Z = config.max_z;
    params::kdev_param = config.kdev_param;
    params::kdist_param = config.kdist_param;
    params::starbeam_filter = config.starbeam_filter;
    params::dmin_param = config.dmin_param;
    params::polysimp_allow = config.simple_poly_allow;
    params::polysimp = config.poly_s_param;
    params::zavg_allow = config.poly_z_avg_allow;
    params::polyz = config.poly_z_manual;
    ROS_INFO("Updated params %s", ros::this_node::getName().c_str());
}

/*MAIN*/
int main(int argc, char **argv)
{

    /*initializing ROS*/
    ros::init(argc, argv, "urban_road_filt");
    ROS_INFO("Initializing %s", ros::this_node::getName().c_str());

    /*code needed for GUI*/
    dynamic_reconfigure::Server<urban_road_filter::LidarFiltersConfig> server;
    dynamic_reconfigure::Server<urban_road_filter::LidarFiltersConfig>::CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    /*NodeHandle*/
    ros::NodeHandle nh;
    Detector detector(&nh);

    ros::spin();
    return 0;
}