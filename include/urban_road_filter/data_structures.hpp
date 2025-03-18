#ifndef URBAN_ROAD_FILTER_DATA_STRUCTURES_HPP_
#define URBAN_ROAD_FILTER_DATA_STRUCTURES_HPP_

/*Basic includes.*/
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <vector>
#include <memory>
#include <functional>
#include <string>

/*Includes for ROS 2.*/
#include <rclcpp/rclcpp.hpp>

/*Includes for Markers.*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*Includes for PCL.*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/impl/point_types.hpp>

/*ramer-douglas-peucker*/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

// #include "parameter_handler/ParameterValue.hpp"

using namespace boost::assign;

namespace boost { namespace geometry { namespace traits {
    template <typename PointT>
    struct tag<PointT> {
        typedef boost::geometry::point_tag type;
    };
}}}

namespace boost { namespace geometry { namespace traits {
    template <>
    struct coordinate_type<pcl::PointXYZI> {
        typedef float type;
    };
}}}

namespace boost { namespace geometry { namespace traits {
    template <>
    struct coordinate_system<pcl::PointXYZI> {
        typedef boost::geometry::cs::cartesian type;
    };
}}}

namespace boost { namespace geometry { namespace traits {
    template <>
    struct dimension<pcl::PointXYZI> : boost::mpl::int_<2> {};
}}}

typedef boost::geometry::model::d2::point_xy<float> xy;  // Define xy here!

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

namespace params {
    extern std::string fixedFrame;
    extern std::string topicName;
    extern bool x_zero_method, z_zero_method, star_shaped_method;
    extern bool blind_spots;
    extern int xDirection;
    extern float interval;
    extern float curbHeight;
    extern int curbPoints;
    extern float beamZone;
    extern float angleFilter1;
    extern float angleFilter2;
    extern float angleFilter3;
    extern float min_X, max_X, min_Y, max_Y, min_Z, max_Z;
    extern float kdev_param;
    extern float kdist_param;
    extern bool starbeam_filter;
    extern int dmin_param;
    extern bool polysimp_allow;
    extern bool zavg_allow;
    extern float polysimp;
    extern float polyz;
    extern int curbPoints;
    extern float curbHeight;
}

template <typename PointT>
class FilteringCondition : public pcl::ConditionBase<PointT>
{
public:
  using Ptr = std::shared_ptr<FilteringCondition<PointT>>;
  using ConstPtr = std::shared_ptr<const FilteringCondition<PointT>>;
  using FunctorT = std::function<bool(const PointT&)>;

  FilteringCondition(FunctorT evaluator)
    : pcl::ConditionBase<PointT>(), _evaluator(evaluator)
  {}

  virtual bool evaluate(const PointT &point) const override {
    // just delegate ALL the work to the injected std::function
    return _evaluator(point);
  }

private:
  FunctorT _evaluator;
};

#endif 
