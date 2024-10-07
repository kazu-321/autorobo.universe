#ifndef PURE_PURSUIT_NODE_HPP
#define PURE_PURSUIT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace pure_pursuit_node{
    class PurePursuitNode : public rclcpp::Node{
    public:
        explicit PurePursuitNode(const rclcpp::NodeOptions &node_options);
    private:
        void timer_callback();
        double get_yaw(const geometry_msgs::msg::Quaternion &q);
        void path_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
        double frequency_, look_ahead_distance_, position_min_speed_, angle_p_, angle_min_speed_;
        bool speed_lookahead_, angle_lookahead_;
        visualization_msgs::msg::Marker path;
        geometry_msgs::msg::Pose current_pose_;
        std::shared_ptr     <tf2_ros::Buffer>                            tf_buffer_;
        std::shared_ptr     <tf2_ros::TransformListener>                 tf_listener_;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr path_sub_;
        rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr loopahead_pub_;
        rclcpp::Publisher   <geometry_msgs::msg::Twist>::SharedPtr       twist_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif