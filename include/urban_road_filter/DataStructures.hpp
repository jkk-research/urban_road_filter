#pragma once

/*Basic includes.*/
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>

/*Includes for ROS.*/
#include <ros/ros.h>

/*Includes for Markers.*/
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*Includes for GUI.*/
#include <dynamic_reconfigure/server.h>
#include <urban_road_filter/LidarFiltersConfig.h>

/*Includes for PCL.*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

/*ramer-douglas-peucker*/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

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
extern int dmin_param;                 //(see below)
extern float kdev_param;               //(see below)
extern float kdist_param;              //(see below)
extern bool polysimp_allow;                           /*polygon-eygszerűsítés engedélyezése*/
extern bool zavg_allow;                               /*egyszerűsített polygon z-koordinátái átlagból (engedély)*/
extern float polysimp;                                 /*polygon-egyszerűsítési tényező (Ramer-Douglas-Peucker)*/
extern float polyz;                                   /*manuálisan megadott z-koordináta (polygon)*/


class Detector{
    public:
    Detector(ros::NodeHandle* nh);

    int partition(std::vector<std::vector<Point3D>>& array3D, int arc,int low, int high);

    void quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);

    //void filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void filter(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void starShapedSearch(std::vector<Point2D>& array2D);

    void xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);

    void zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);
    
    private:
    ros::Publisher pub_road;        
    ros::Publisher pub_high;        
    ros::Publisher pub_box;         
    ros::Publisher pub_pobroad;    
    ros::Publisher pub_marker;      

    ros::Subscriber sub;

    boost::geometry::model::linestring<xy> line;
    boost::geometry::model::linestring<xy> simplified;
};