#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace planner_node{
    class PlannerNode : public rclcpp::Node{
    public:
        explicit PlannerNode(const rclcpp::NodeOptions &node_option);

    private:
        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();
        double get_Yaw(const geometry_msgs::msg::Quaternion &q);
        double slow_stop_, resolution_, zero_stop_, v_max, freq, position_tolerance_, angle_tolerance_;
        geometry_msgs::msg::PoseStamped::SharedPtr goal_rcv;
        geometry_msgs::msg::Pose current_pose_;
        rclcpp::Publisher   <visualization_msgs::msg::Marker>::SharedPtr path_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
        rclcpp::Publisher   <geometry_msgs::msg::Twist>::SharedPtr err_pub_;
        std::shared_ptr     <tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr     <tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif