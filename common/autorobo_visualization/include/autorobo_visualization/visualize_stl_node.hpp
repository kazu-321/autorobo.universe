
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace visualize_stl_node{
    class VisualizeStlNode : public rclcpp::Node{
    public:
        explicit VisualizeStlNode(const rclcpp::NodeOptions &node_options);
    private:
        void timer_callback();
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_maker_;
        visualization_msgs::msg::Marker marker_msg;
        rclcpp::TimerBase::SharedPtr pub_timer_;
    };
}