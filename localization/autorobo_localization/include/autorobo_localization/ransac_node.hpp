#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ransac_node{
    class RansacNode : public rclcpp::Node{
    public:
        explicit RansacNode(const rclcpp::NodeOptions &node_options);
    private:
        struct Point {
            double x, y;
        };

        struct Wall{
            Point p1, p2;
        };

        struct Line{
            double a, b;
        };

        RansacNode::Line ransac(std::vector<Point> data);
        std::vector<Line> ransacs(std::vector<Point> data);
        Wall RansacNode::Line2Wall(std::vector<Point> data,Line ab,bool TB)
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        int iter, max_lines;
        double map_resolution;
        double distance_threshold;
    };
}