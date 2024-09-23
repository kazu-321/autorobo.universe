#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class visualization_node : public rclcpp::Node{
public:
    visualization_node(const rclcpp::NodeOptions &node_option) : rclcpp::Node("visualize_lines_node", node_option){

        std::string name = declare_parameter<std::string>("namespace" , "area");

        pub_maker_ = create_publisher<visualization_msgs::msg::Marker>(
            "area/"+name, 0);

        marker_template = visualization_msgs::msg::Marker();

        marker_template.header.frame_id = declare_parameter<std::string>("frame_id" , "object");;
        marker_template.ns = "polygon";
        marker_template.id = declare_parameter<int>("marker.id" , 0);
        marker_template.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_template.action = visualization_msgs::msg::Marker::ADD; 
        marker_template.scale.x = declare_parameter<double>("marker.scale"   , 0.05);
        marker_template.color.r = declare_parameter<double>("marker.color.r" , 1.0);
        marker_template.color.g = declare_parameter<double>("marker.color.g" , 1.0);
        marker_template.color.b = declare_parameter<double>("marker.color.b" , 1.0);
        marker_template.color.a = declare_parameter<double>("marker.color.a" , 1.0);
        marker_template.frame_locked = false;
        const auto period = declare_parameter<int>("publsih_rate_ms" , 1000);
        area_x = declare_parameter<std::vector<double>>("area.x",{0.0,1.0});
        area_y = declare_parameter<std::vector<double>>("area.y",{0.0,1.0});
        z = declare_parameter<double>("area.z", 0.0);
        RCLCPP_INFO_STREAM(get_logger(),  "Initialize Task Done");
        timer_ = create_wall_timer(std::chrono::milliseconds(period), std::bind(&visualization_node::timer_callback, this));
    }
private:
    void timer_callback(){
        visualization_msgs::msg::Marker marker = marker_template;
        marker.header.stamp = now();
        marker.points.clear();
        geometry_msgs::msg::Point p;
        std::pair<int,int> pattern[5] = {{0,0},{0,1},{1,1},{1,0},{0,0}};
        for(std::pair<int,int> index: pattern){
            p.x = area_x[index.first];
            p.y = area_y[index.second];
            p.z = z;
            marker.points.push_back(p);
        }
        pub_maker_->publish(marker);
    }
    std::vector<double> area_x;
    std::vector<double> area_y;
    double z;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_maker_;
    visualization_msgs::msg::Marker marker_template;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<visualization_node>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}