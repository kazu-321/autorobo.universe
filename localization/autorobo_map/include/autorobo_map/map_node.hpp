#ifndef MAP_NODE_HPP
#define MAP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace map_node{
    class MapNode : public rclcpp::Node{
    public:
        explicit MapNode(const rclcpp::NodeOptions &node_options);
    private:
        void publishMap();
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}


#endif // MAP_NODE_HPP