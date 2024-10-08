// Copyright 2024 Hakoroboken
// Modifications copyright 2024 kazu-321
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autorobo_control/pure_pursuit_node.hpp"

namespace pure_pursuit_node{
    PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("pure_pursuit_node",node_options){
        path_sub_      = this->create_subscription<visualization_msgs::msg::Marker>("/planning/path", 10,
                         std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
        tf_buffer_     = std::make_shared         <tf2_ros::Buffer>                (this->get_clock());
        tf_listener_   = std::make_shared         <tf2_ros::TransformListener>     (*tf_buffer_);
        twist_pub_     = this->create_publisher   <geometry_msgs::msg::Twist>      ("/cmd_vel_nav", 10);
        loopahead_pub_ = this->create_publisher   <geometry_msgs::msg::PoseStamped>("/control/lookahead_pose", 10);
        frequency_     = this->declare_parameter<double>("frequency",          20.0);
        timer_         = this->create_wall_timer(std::chrono::milliseconds((int64_t)(1.0/frequency_*1000)), 
                         std::bind(&PurePursuitNode::timer_callback, this));
        look_ahead_distance_= this->declare_parameter<double>("look_ahead_distance", 0.5);
        position_min_speed_ = this->declare_parameter<double>("position_min_speed", 0.10);
        angle_p_            = this->declare_parameter<double>("angle_p",             1.0);
        angle_min_speed_    = this->declare_parameter<double>("angle_min_speed",    0.05);
        speed_lookahead_    = this->declare_parameter<bool>  ("speed_lookahead",   false);
        angle_lookahead_    = this->declare_parameter<bool>  ("angle_lookahead",   false);
    }

    void PurePursuitNode::timer_callback(){
        geometry_msgs::msg::PoseStamped pose;
        try{
            // Get current pose
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation.x = transform.transform.rotation.x;
            pose.pose.orientation.y = transform.transform.rotation.y;
            pose.pose.orientation.z = transform.transform.rotation.z;
            pose.pose.orientation.w = transform.transform.rotation.w;
            current_pose_ = pose.pose;
        }catch (tf2::TransformException &ex){
            // RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            // Set default pose values
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
        }
        if(path.points.size() == 0){        
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z= 0.0;
            twist_pub_->publish(cmd_vel);
            return;
        }

        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
        position_min_speed_  = this->get_parameter("position_min_speed").as_double();
        angle_p_             = this->get_parameter("angle_p").as_double();
        angle_min_speed_     = this->get_parameter("angle_min_speed").as_double();
        speed_lookahead_     = this->get_parameter("speed_lookahead").as_bool();
        angle_lookahead_     = this->get_parameter("angle_lookahead").as_bool();
        

        double current_x = current_pose_.position.x;
        double current_y = current_pose_.position.y;
        double min_distance = std::numeric_limits<double>::max(); // 最小距離を最大で初期化
        int min_distance_index = 0; // 最小距離のインデックス
        for (size_t i = 0; i < path.points.size(); ++i){
            double dx = path.points[i].x - current_x;
            double dy = path.points[i].y - current_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            if (distance < min_distance){ // 最小距離を更新
                min_distance = distance;
                min_distance_index = i;
            }
        }
        int min_index = min_distance_index;
        double goal_distance=std::sqrt((path.points.back().x-current_x)*(path.points.back().x-current_x)+(path.points.back().y-current_y)*(path.points.back().y-current_y));
        // look ahead distanceが最終地点よりも大きい場合は最終地点を目標とする
        if(goal_distance<look_ahead_distance_) min_index = path.points.size()-1;
        else if(min_distance<look_ahead_distance_){
            for(size_t i=min_index;i<path.points.size();i++){
                double dx = path.points[i].x - current_x;
                double dy = path.points[i].y - current_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                if(distance>look_ahead_distance_){ // look ahead distanceを超えたら終了
                    min_index = i;
                    break;
                }
            }
        }

        // pure pursuit start
        geometry_msgs::msg::Point look_ahead_point = path.points[min_index];
        geometry_msgs::msg::Point delta;
        // Δを計算
        delta.x = look_ahead_point.x - current_pose_.position.x;
        delta.y = look_ahead_point.y - current_pose_.position.y;
        delta.z = look_ahead_point.z - current_pose_.orientation.z;
        double angle_map = std::atan2(delta.y,delta.x);   // 目標地点への角度を計算
        double yaw = get_yaw(current_pose_.orientation);    // 現在の角度を取得
        double angle = angle_map-yaw;  // 現在の角度から目標地点への角度を引く
        double speed = path.colors[min_distance_index].g + position_min_speed_; // 最小距離の速度を取得

        // path_の最後とcurrentの差
        // double diff_goalx = path_.poses.back().position.x - current_x;
        // double diff_goaly = path_.poses.back().position.y - current_y;
        // double diff_goalz = get_yaw(path_.poses.back().orientation) - yaw;

        geometry_msgs::msg::Twist cmd_vel;
        // 速度を計算
        cmd_vel.linear.x = speed * std::cos(angle);
        cmd_vel.linear.y = speed * std::sin(angle);
        cmd_vel.angular.z= 0.0;
        // pure pursuit end

        // angle P start

        double target = path.points[min_distance_index].z;   // 最小距離の角度を取得
        if(angle_lookahead_) target = look_ahead_point.z;
        double diff_angle=target-yaw;
        while(diff_angle >  M_PI) diff_angle-=2*M_PI;   // 角度の差を-π~πにする
        while(diff_angle < -M_PI) diff_angle+=2*M_PI;
        cmd_vel.angular.z = angle_p_*diff_angle;   // 角速度を計算
        if(std::signbit(diff_angle))cmd_vel.angular.z-=angle_min_speed_;    // 角速度が0になるのを防ぐ
        else cmd_vel.angular.z+=angle_min_speed_;
        cmd_vel.angular.z = std::min(1.0, std::max(-1.0, cmd_vel.angular.z)); // 角速度を-1~1にする

        // angle P end

        twist_pub_->publish(cmd_vel);   // 速度をパブリッシュ

        geometry_msgs::msg::PoseStamped lookahead_pose;
        lookahead_pose.header.stamp = this->now();
        lookahead_pose.header.frame_id = "map";
        lookahead_pose.pose.position = look_ahead_point;
        lookahead_pose.pose.orientation.z=sin(look_ahead_point.z/2.0);
        lookahead_pose.pose.orientation.w=cos(look_ahead_point.z/2.0);

        loopahead_pub_->publish(lookahead_pose);    // look ahead pointをパブリッシュ

        // RCLCPP_INFO(this->get_logger(), "angle: map:%lf, curr:%lf, delta:%lf, diff:%lf",angle_map*57.3,yaw*57.3,angle*57.3,diff_angle*57.3);
    }
    double PurePursuitNode::get_yaw(const geometry_msgs::msg::Quaternion &q){return std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));}
    void PurePursuitNode::path_callback(const visualization_msgs::msg::Marker::SharedPtr msg){path=*msg;}
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit_node::PurePursuitNode)