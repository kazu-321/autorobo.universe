#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace visualize_lines_node{
    class VisualizeLinesNode : public rclcpp::Node{
    public:
        explicit VisualizeLinesNode(const rclcpp::NodeOptions &node_options);
    private:
        void timer_callback();
        std::vector<double> area_x, area_y;
        double z;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_maker_;
        visualization_msgs::msg::Marker marker_template;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}