#pragma once

/*Basic includes.*/
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>
#include <memory>

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
#include <pcl/filters/conditional_removal.h>

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

class Detector{
    public:
    Detector(ros::NodeHandle* nh);

    int partition(std::vector<std::vector<Point3D>>& array3D, int arc,int low, int high);

    void quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high);

    void filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void starShapedSearch(std::vector<Point2D>& array2D);

    void beam_init();

    void xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);

    void zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray);

    void blindSpots(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray,float* maxDistance);
    
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
