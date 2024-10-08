#ifndef MCL_NODE_HPP
#define MCL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mcl_node{
    class MclNode : public rclcpp::Node{
    public:
        explicit MclNode(const rclcpp::NodeOptions &node_options);
    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    };
}

#endif