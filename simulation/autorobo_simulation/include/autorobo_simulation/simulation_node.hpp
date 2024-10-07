#ifndef SIMULATION_NODE_HPP
#define SIMULATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "autorobo_msgs/msg/twistring.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>

namespace simulation_node{
    class OmniSim : public rclcpp::Node{
    public:
        explicit OmniSim(const rclcpp::NodeOptions & node_options);
    private:
        struct Wall{
            double x1;
            double y1;
            double x2;
            double y2;
        };
        void velocityCallback(const autorobo_msgs::msg::Twistring::SharedPtr msg);
        void timerCallback();
        void lidarCallback();
        float compute_ray_wall_intersection(float lx, float ly, float angle, Wall wall);
        double get_Yaw(const geometry_msgs::msg::Quaternion &q);

        rclcpp::Subscription<autorobo_msgs::msg::Twistring>::SharedPtr       twistring_subscriber_;
        rclcpp::Publisher   <sensor_msgs::msg::LaserScan>::SharedPtr     lidar_pub_;
        std::unique_ptr     <tf2_ros::TransformBroadcaster>              tf_broadcaster_;
        rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        std::shared_ptr     <tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr     <tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped lidar_tf_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr lidar_timer_;
        geometry_msgs::msg::Twist twist_sum_;
        std::vector<double> walls_raw;
        std::vector<Wall> walls;
        int count_, lidar_freq_, lidar_resolution_;
        double old_x_, old_y_, old_z_, x_, y_, z_, lidar_err_,  angle_max_, angle_min_;
        bool sig_, servo_[2], got_tf;
    };
}

#endif