#include "autorobo_map/map_node.hpp"

namespace map_node{
    MapNode::MapNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("map_node", node_options) {
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapNode::publishMap, this));
    }
    void MapNode::publishMap(){
        // (x,y)で(0,0)~(3,0)、(0,0)~(0,5)、(0,5)~(3,5)に直線 解像度は 0.1m/cel
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.origin.position.x = -0.7;
        map.info.origin.position.y = -0.8;
        map.info.resolution = 0.1;
        map.info.width = 3/map.info.resolution;
        map.info.height = 5/map.info.resolution;
        map.data.resize(map.info.width * map.info.height);
        for(int y=0;y<map.info.height;y++){
            for(int x=0;x<map.info.width;x++) {
                int index=map.info.width*y+x;
                if((y==0 && 0<=x && x<=map.info.width) || (x==0 && 0<=y && y<=map.info.height) || (y==map.info.height-1 && 0<=x && x<=map.info.width)){
                    map.data[index] = 100;
                } else {
                    map.data[index] = 0;
                }
            }
        }
        map_publisher_->publish(map);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_node::MapNode)