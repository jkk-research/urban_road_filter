#pragma once

/*Basic includes.*/
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>
#include <memory>
#include <functional>

/*Includes for ROS 2.*/
#include "rclcpp/rclcpp.hpp"

/*Includes for Markers.*/
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/*Includes for PCL.*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_ros/transforms.hpp>

/*ROS 2 specific message types*/
#include "sensor_msgs/msg/point_cloud2.hpp"

/*ramer-douglas-peucker*/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

#include "std_msgs/msg/float32_multi_array.hpp"

using namespace boost::assign;

typedef boost::geometry::model::d2::point_xy<float> xy;

struct Point2D{
    pcl::PointXYZI p;
    float d;
    float alpha;
    short isCurbPoint;
};

struct Point3D:public Point2D{
    float newY;
};

struct polar    //polar-coordinate struct for the points
{
    int id;     //original ID of point (from input cloud)
    float r;    //radial coordinate
    float fi;   //angular coordinate (ccw angle from x-axis)
};

struct box      //struct for detection beams
{
    std::vector<polar> p; //points within the beam's area
    //box *l, *r;           //pointer to adjacent beams (currently not used)
    bool yx;              //whether it is aligned more with the y-axis (than the x-axis)
    float o, d;           //internal parameters (trigonometry)
};

namespace params{
  extern std::string fixedFrame;                               /* Fixed Frame.*/
  extern std::string topicName;                                /* subscribed topic.*/
  extern bool x_zero_method, z_zero_method, star_shaped_method ; /*Methods of roadside detection*/
  extern bool blind_spots;                                     /*Vakfolt javító algoritmus.*/
  extern int xDirection;                                       /*A vakfolt levágás milyen irányú.*/
  extern float interval;                                       /*A LIDAR vertikális szögfelbontásának, elfogadott intervalluma.*/
  extern float curbHeight;                                     /*Becsült minimum szegély magasság.*/
  extern int curbPoints;                                       /*A pontok becsült száma, a szegélyen.*/
  extern float beamZone;                                       /*A vizsgált sugárzóna mérete.*/
  extern float angleFilter1;                                   /*X = 0 érték mellett, három pont által bezárt szög.*/
  extern float angleFilter2;                                   /*Z = 0 érték mellett, két vektor által bezárt szög.*/
  extern float angleFilter3;                                   /*Csaplár László kódjához szükséges. Sugár irányú határérték (fokban).*/
  extern float min_X, max_X, min_Y, max_Y, min_Z, max_Z;       /*A vizsgált terület méretei.*/
  extern float kdev_param;                //(see below)
  extern float kdist_param;               //(see below)
  extern bool starbeam_filter;            //Toggle usage of rectangular beams for starshaped algorithm instead of the whole sector (containing the beam)
  extern int dmin_param;                  //(see below)
  extern bool polysimp_allow;                           /*polygon-eygszerűsítés engedélyezése*/
  extern bool zavg_allow;                               /*egyszerűsített polygon z-koordinátái átlagból (engedély)*/
  extern float polysimp;                                 /*polygon-egyszerűsítési tényező (Ramer-Douglas-Peucker)*/
  extern float polyz;                                   /*manuálisan megadott z-koordináta (polygon)*/
};
/*For pointcloud filtering*/
template <typename PointT>
class FilteringCondition : public pcl::ConditionBase<PointT>
{
public:
  typedef std::shared_ptr<FilteringCondition<PointT>> Ptr;
  typedef std::shared_ptr<const FilteringCondition<PointT>> ConstPtr;
  typedef std::function<bool(const PointT&)> FunctorT;

  FilteringCondition(FunctorT evaluator): 
    pcl::ConditionBase<PointT>(),_evaluator( evaluator ) 
  {}

  virtual bool evaluate (const PointT &point) const {
    // just delegate ALL the work to the injected std::function
    return _evaluator(point);
  }
private:
  FunctorT _evaluator;
};

class Detector : public rclcpp::Node {
public:
    Detector();
    void initialize();

    int partition(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);
    void quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);
    void filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud);
    void starShapedSearch(std::vector<Point2D>& array2D);
    void beam_init();
    void xZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, int* indexArray);
    void zZeroMethod(std::vector<std::vector<Point3D>>& array3D, int index, int* indexArray);
    void blindSpots(std::vector<std::vector<Point3D>>& array3D, int index, int* indexArray, float* maxDistance);
    
    // Additional methods for ROS 2
    void declare_and_get_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameter_changed(const std::vector<rclcpp::Parameter> &parameters);
    void update_global_params();

private:
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Parameter declarations
    std::string fixed_frame_;
    std::string topic_name_;
    bool x_zero_method_;
    bool z_zero_method_;
    bool star_shaped_method_;
    bool blind_spots_;
    bool xDirection_;
    double interval_;
    double curb_height_;
    int curb_points_;
    int beamZone_;
    double cylinder_deg_x_;
    double cylinder_deg_z_;
    double curb_slope_deg_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;
    double kdev_param_;
    double kdist_param_;
    bool starbeam_filter_;
    int dmin_param_;
    bool simple_poly_allow_;
    double poly_s_param_;
    bool poly_z_avg_allow_;
    double poly_z_manual_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_road;        
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_high;        
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_box;         
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pobroad;    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_stats;  // Statistics publisher

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    
    // Callback function for point cloud processing
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    // For Ramer-Douglas-Peucker algorithm
    boost::geometry::model::linestring<xy> line;
    boost::geometry::model::linestring<xy> simplified;
};
