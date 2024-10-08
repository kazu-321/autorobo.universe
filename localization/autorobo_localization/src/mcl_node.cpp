#include "autorobo_localization/mcl_node.hpp"

namespace mcl_node{
    MclNode::MclNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("mcl_node", node_options) {
        pose_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/current_pose", 10);
        line_pub_     = this->create_publisher<visualization_msgs::msg::Marker>("/localization/mcl/particle", 10);
        scan_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), 
                        std::bind(&MclNode::scan_callback, this, std::placeholders::_1));
        map_sub_      = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SensorDataQoS(), 
                        std::bind(&MclNode::map_callback, this, std::placeholders::_1));
        init_sub_     = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
                        std::bind(&MclNode::init_callback, this, std::placeholders::_1));
        tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    void MclNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"scan recv");
    }

    void MclNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"map recv");
    }

    void MclNode::init_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"init pose recv");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mcl_node::MclNode)