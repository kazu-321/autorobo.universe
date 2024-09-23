#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class pure_pursuit_node : public rclcpp::Node{
public:
    pure_pursuit_node() : Node("controller_node"){
        path_sub_      = this->create_subscription<geometry_msgs::msg::PoseArray>  ("/planning/path", 10,
                         std::bind(&pure_pursuit_node::path_callback, this, std::placeholders::_1));
        tf_buffer_     = std::make_shared         <tf2_ros::Buffer>                (this->get_clock());
        tf_listener_   = std::make_shared         <tf2_ros::TransformListener>     (*tf_buffer_);
        twist_pub_     = this->create_publisher   <geometry_msgs::msg::Twist>      ("/cmd_vel_nav", 10);
        loopahead_pub_ = this->create_publisher   <geometry_msgs::msg::PoseStamped>("/control/lookahead_pose", 10);
        frequency_     = this->declare_parameter<double>("frequency",          20.0);
        timer_         = this->create_wall_timer(std::chrono::milliseconds((int64_t)(1.0/frequency_*1000)), 
                         std::bind(&pure_pursuit_node::timer_callback, this));
        look_ahead_distance_= this->declare_parameter<double>("look_ahead_distance", 0.5);
        position_min_speed_ = this->declare_parameter<double>("position_min_speed", 0.10);
        angle_p_            = this->declare_parameter<double>("angle_p",             1.0);
        angle_min_speed_    = this->declare_parameter<double>("angle_min_speed",    0.05);
        speed_lookahead_    = this->declare_parameter<bool>  ("speed_lookahead",   false);
        angle_lookahead_    = this->declare_parameter<bool>  ("angle_lookahead",   false);
    }

private:
    void timer_callback(){
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
        if(path_.poses.size() == 0){        
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
        for (size_t i = 0; i < path_.poses.size(); ++i){
            double dx = path_.poses[i].position.x - current_x;
            double dy = path_.poses[i].position.y - current_y;
            double distance = std::sqrt(dx*dx + dy*dy);
            if (distance < min_distance){ // 最小距離を更新
                min_distance = distance;
                min_distance_index = i;
            }
        }
        int min_index = min_distance_index;
        double goal_distance=std::sqrt((path_.poses.back().position.x-current_x)*(path_.poses.back().position.x-current_x)+(path_.poses.back().position.y-current_y)*(path_.poses.back().position.y-current_y));
        // look ahead distanceが最終地点よりも大きい場合は最終地点を目標とする
        if(goal_distance<look_ahead_distance_) min_index = path_.poses.size()-1;
        else if(min_distance<look_ahead_distance_){
            for(size_t i=min_index;i<path_.poses.size();i++){
                double dx = path_.poses[i].position.x - current_x;
                double dy = path_.poses[i].position.y - current_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                if(distance>look_ahead_distance_){ // look ahead distanceを超えたら終了
    rclcpp::Publisher   <geometry_msgs::msg::PoseArray>::SharedPtr   path_pub_;
                    min_index = i;
                    break;
                }
            }
        }

        // pure pursuit start
        geometry_msgs::msg::Pose look_ahead_pose = path_.poses[min_index];
        geometry_msgs::msg::Pose delta;
        // Δを計算
        delta.position.x = look_ahead_pose.position.x - current_pose_.position.x;
        delta.position.y = look_ahead_pose.position.y - current_pose_.position.y;
        delta.orientation.z = look_ahead_pose.orientation.z - current_pose_.orientation.z;
        delta.orientation.w = look_ahead_pose.orientation.w - current_pose_.orientation.w;
        double angle_map = std::atan2(delta.position.y,delta.position.x);   // 目標地点への角度を計算
        double yaw = get_yaw(current_pose_.orientation);    // 現在の角度を取得
        double angle = angle_map-yaw;  // 現在の角度から目標地点への角度を引く
        double speed = path_.poses[min_distance_index].position.z + position_min_speed_; // 最小距離の速度を取得
        if(speed_lookahead_) speed = look_ahead_pose.position.z + position_min_speed_;

        // path_の最後とcurrentの差
        double diff_goalx = path_.poses.back().position.x - current_x;
        double diff_goaly = path_.poses.back().position.y - current_y;
        double diff_goalz = get_yaw(path_.poses.back().orientation) - yaw;

        geometry_msgs::msg::Twist cmd_vel;
        // 速度を計算
        cmd_vel.linear.x = speed * std::cos(angle);
        cmd_vel.linear.y = speed * std::sin(angle);
        cmd_vel.angular.z= 0.0;
        // pure pursuit end

        // angle P start

        double target = get_yaw(path_.poses[min_distance_index].orientation);   // 最小距離の角度を取得
        if(angle_lookahead_) target = get_yaw(look_ahead_pose.orientation);
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
        lookahead_pose.pose = look_ahead_pose;
        loopahead_pub_->publish(lookahead_pose);    // look ahead pointをパブリッシュ

        // RCLCPP_INFO(this->get_logger(), "angle: map:%lf, curr:%lf, delta:%lf, diff:%lf",angle_map*57.3,yaw*57.3,angle*57.3,diff_angle*57.3);
    }
    double get_yaw(const geometry_msgs::msg::Quaternion &q){return std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));}
    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){path_=*msg;}
    double frequency_;
    double look_ahead_distance_;
    double position_min_speed_;
    double angle_p_;
    double angle_min_speed_;
    bool speed_lookahead_;
    bool angle_lookahead_;
    geometry_msgs::msg::PoseArray path_;
    geometry_msgs::msg::Pose current_pose_;
    std::shared_ptr     <tf2_ros::Buffer>                            tf_buffer_;
    std::shared_ptr     <tf2_ros::TransformListener>                 tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr   path_sub_;
    rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr loopahead_pub_;
    rclcpp::Publisher   <geometry_msgs::msg::Twist>::SharedPtr       twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pure_pursuit_node>());
    rclcpp::shutdown();
    return 0;
}