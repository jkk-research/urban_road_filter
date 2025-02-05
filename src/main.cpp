#include "urban_road_filter/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"

class UrbanRoadFilterNode : public rclcpp::Node {
public:
    UrbanRoadFilterNode() : Node("urban_road_filter") {
        RCLCPP_INFO(this->get_logger(), "Initializing %s", this->get_name());

        // Declare ROS2 Parameters
        this->declare_parameter<std::string>("fixed_frame", "base_link");
        this->declare_parameter<std::string>("topic_name", "/lidar_points");
        this->declare_parameter<int>("x_zero_method", 0);
        this->declare_parameter<int>("z_zero_method", 0);
        this->declare_parameter<int>("star_shaped_method", 0);
        this->declare_parameter<bool>("blind_spots", false);
        this->declare_parameter<bool>("xDirection", false);
        this->declare_parameter<double>("interval", 0.1);
        this->declare_parameter<double>("curb_height", 0.2);
        this->declare_parameter<int>("curb_points", 5);
        this->declare_parameter<double>("beamZone", 1.0);
        this->declare_parameter<double>("cylinder_deg_x", 15.0);
        this->declare_parameter<double>("cylinder_deg_z", 15.0);
        this->declare_parameter<double>("curb_slope_deg", 30.0);
        this->declare_parameter<double>("min_x", -5.0);
        this->declare_parameter<double>("max_x", 5.0);
        this->declare_parameter<double>("min_y", -2.0);
        this->declare_parameter<double>("max_y", 2.0);
        this->declare_parameter<double>("min_z", -1.0);
        this->declare_parameter<double>("max_z", 3.0);
        this->declare_parameter<double>("kdev_param", 1.0);
        this->declare_parameter<double>("kdist_param", 1.0);
        this->declare_parameter<bool>("starbeam_filter", false);
        this->declare_parameter<double>("dmin_param", 0.5);
        this->declare_parameter<bool>("simple_poly_allow", false);
        this->declare_parameter<double>("poly_s_param", 0.1);
        this->declare_parameter<bool>("poly_z_avg_allow", false);
        this->declare_parameter<double>("poly_z_manual", 0.0);

        // Set callback for parameter updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&UrbanRoadFilterNode::paramsCallback, this, std::placeholders::_1)
        );

        // Initialize Detector
        detector_ = std::make_shared<Detector>(this);
    }

private:
    rcl_interfaces::msg::SetParametersResult paramsCallback(
        const std::vector<rclcpp::Parameter> &parameters) {

        for (const auto &param : parameters) {
            if (param.get_name() == "fixed_frame") {
                params::fixedFrame = param.as_string();
            } else if (param.get_name() == "topic_name") {
                params::topicName = param.as_string();
            } else if (param.get_name() == "x_zero_method") {
                params::x_zero_method = param.as_int();
            } else if (param.get_name() == "z_zero_method") {
                params::z_zero_method = param.as_int();
            } else if (param.get_name() == "star_shaped_method") {
                params::star_shaped_method = param.as_int();
            } else if (param.get_name() == "blind_spots") {
                params::blind_spots = param.as_bool();
            } else if (param.get_name() == "xDirection") {
                params::xDirection = param.as_bool();
            } else if (param.get_name() == "interval") {
                params::interval = param.as_double();
            } else if (param.get_name() == "curb_height") {
                params::curbHeight = param.as_double();
            } else if (param.get_name() == "curb_points") {
                params::curbPoints = param.as_int();
            } else if (param.get_name() == "beamZone") {
                params::beamZone = param.as_double();
            } else if (param.get_name() == "cylinder_deg_x") {
                params::angleFilter1 = param.as_double();
            } else if (param.get_name() == "cylinder_deg_z") {
                params::angleFilter2 = param.as_double();
            } else if (param.get_name() == "curb_slope_deg") {
                params::angleFilter3 = param.as_double();
            } else if (param.get_name() == "min_x") {
                params::min_X = param.as_double();
            } else if (param.get_name() == "max_x") {
                params::max_X = param.as_double();
            } else if (param.get_name() == "min_y") {
                params::min_Y = param.as_double();
            } else if (param.get_name() == "max_y") {
                params::max_Y = param.as_double();
            } else if (param.get_name() == "min_z") {
                params::min_Z = param.as_double();
            } else if (param.get_name() == "max_z") {
                params::max_Z = param.as_double();
            } else if (param.get_name() == "kdev_param") {
                params::kdev_param = param.as_double();
            } else if (param.get_name() == "kdist_param") {
                params::kdist_param = param.as_double();
            } else if (param.get_name() == "starbeam_filter") {
                params::starbeam_filter = param.as_bool();
            } else if (param.get_name() == "dmin_param") {
                params::dmin_param = param.as_double();
            } else if (param.get_name() == "simple_poly_allow") {
                params::polysimp_allow = param.as_bool();
            } else if (param.get_name() == "poly_s_param") {
                params::polysimp = param.as_double();
            } else if (param.get_name() == "poly_z_avg_allow") {
                params::zavg_allow = param.as_bool();
            } else if (param.get_name() == "poly_z_manual") {
                params::polyz = param.as_double();
            }
        }

        RCLCPP_INFO(this->get_logger(), "Parameters updated dynamically.");
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    std::shared_ptr<Detector> detector_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UrbanRoadFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
