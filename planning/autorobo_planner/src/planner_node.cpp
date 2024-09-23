#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PlannerNode : public rclcpp::Node{
public:
    PlannerNode() : Node("planner_node"){
        path_pub_    = this->create_publisher<geometry_msgs::msg::PoseArray>("/planning/path", 10);
        goal_sub_    = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
                       std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));
        goal_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/goal", 10);
        err_pub_     = this->create_publisher<geometry_msgs::msg::Twist>("/planning/error", 10);
        v_max        = this->declare_parameter<double>("v_max",1.0);
        freq         = this->declare_parameter<double>("frequency",10.0);
        position_tolerance_ = this->declare_parameter<double>("position_tolerance", 0.01);
        angle_tolerance_    = this->declare_parameter<double>("angle_tolerance",    0.01);
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_       = this->create_wall_timer(std::chrono::milliseconds((int64_t)(1.0/freq*1000)), 
                       std::bind(&PlannerNode::timer_callback, this));
        slow_stop_   = this->declare_parameter<double>("slow_stop",0.1);
        resolution_  = this->declare_parameter<double>("resolution",0.02);
        zero_stop_   = this->declare_parameter<double>("zero_stop",0.1);
    }
private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Received goal: (%f, %f),%f", msg->pose.position.x, msg->pose.position.y,get_Yaw(msg->pose.orientation)*180/M_PI);
        goal_rcv=msg;
    }
    geometry_msgs::msg::PoseStamped::SharedPtr goal_rcv;

    void timer_callback(){
        if(goal_rcv==nullptr)return;
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id="map";
        goal.header.stamp = this->now();
        goal.pose = goal_rcv->pose;

        if(goal.pose.position.x>1.7)goal.pose.position.x=1.7;
        if(goal.pose.position.x<0.0)goal.pose.position.x=0.0;
        if(goal.pose.position.y>3.5)goal.pose.position.y=3.5;
        if(goal.pose.position.y<0.0)goal.pose.position.y=0.0;

        double z_=get_Yaw(goal.pose.orientation)*180.0/M_PI;
        double max_z=3.0;
        if(z_>max_z)z_=max_z;
        if(z_<-max_z)z_=-max_z;
        goal.pose.orientation.z=std::sin(z_/2.0*M_PI/180.0);
        goal.pose.orientation.w=std::cos(z_/2.0*M_PI/180.0);

        goal_pub_->publish(goal);


        try{
            // Get current pose
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            current_pose_.position.x = transform.transform.translation.x;
            current_pose_.position.y = transform.transform.translation.y;
            current_pose_.position.z = transform.transform.translation.z;
            current_pose_.orientation.x = transform.transform.rotation.x;
            current_pose_.orientation.y = transform.transform.rotation.y;
            current_pose_.orientation.z = transform.transform.rotation.z;
            current_pose_.orientation.w = transform.transform.rotation.w;
        }catch (tf2::TransformException &ex){
            // RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            // Set default pose values
            current_pose_.position.x = 0.0;
            current_pose_.position.y = 0.0;
            current_pose_.position.z = 0.0;
            current_pose_.orientation.x = 0.0;
            current_pose_.orientation.y = 0.0;
            current_pose_.orientation.z = 0.0;
            current_pose_.orientation.w = 1.0;
        }
        
        slow_stop_ = this->get_parameter("slow_stop").as_double();
        zero_stop_ = this->get_parameter("zero_stop").as_double();
        v_max      = this->get_parameter("v_max").as_double();
        
        geometry_msgs::msg::PoseArray path;
        path.header = goal.header;
        double x= goal.pose.position.x - current_pose_.position.x;
        double y= goal.pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(x*x + y*y); // 距離
        double dx = x/distance; // 単位
        double dy = y/distance; // 単位
        double start_yaw = get_Yaw(current_pose_.orientation);
        double goal_yaw = get_Yaw(goal.pose.orientation);
        double delta_yaw = goal_yaw - start_yaw;    // rad
        if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;    // -π < delta_yaw < π
        else if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;

        geometry_msgs::msg::Twist err;
        err.linear.x = x;
        err.linear.y = y;
        err.angular.z= delta_yaw;
        err_pub_->publish(err);

        if(std::abs(x)<position_tolerance_/100.0 && std::abs(y)<position_tolerance_/100.0 && std::abs(delta_yaw)<angle_tolerance_*M_PI/180.0){
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            RCLCPP_INFO(this->get_logger(), "x: %lfcm, y: %lfcm, yaw: %lf°", x*100.0, y*100.0, delta_yaw*180/M_PI);
            geometry_msgs::msg::PoseArray path;
            path.header.stamp = this->now();
            path.header.frame_id = "map";
            path_pub_->publish(path);   // 目標地点に到達したらpathをクリア
            goal_rcv=nullptr;
            return;
        }

        // RCLCPP_INFO(this->get_logger(), "send goal: (%f, %f),%f", goal.pose.position.x, goal.pose.position.y,get_Yaw(goal.pose.orientation)*180/M_PI);
        for (double now = resolution_; now < distance; now+=resolution_){
            geometry_msgs::msg::Pose pose;
            pose.position.x = current_pose_.position.x + now * dx;
            pose.position.y = current_pose_.position.y + now * dy;
            double progress = now / distance;   // 0.0 < progress < 1.0
            double interp_velocity;
            // 進行度に応じて速度を変化させる
            //if(progress > 1.0-zero_stop_)interp_velocity = 0.0;
            //else if(progress > 1.0-slow_stop_)interp_velocity = v_max * ((1.0 - progress) / slow_stop_);
            //else 				interp_velocity = v_max;
            if(now>distance-zero_stop_) interp_velocity = 0.0;
            else if(now>distance-slow_stop_)interp_velocity = v_max * (1.0-(slow_stop_+now-distance)/(slow_stop_-zero_stop_));
            else interp_velocity = v_max;
            pose.position.z=interp_velocity;
            // 進行度に応じてsigmoidでyawを変化させる
            double current_yaw = start_yaw + delta_yaw / (1.0 + std::exp(-7.5 * (progress - 0.5)));
            // double current_yaw = start_yaw + delta_yaw * progress;
            pose.orientation.z = std::sin(current_yaw/2.0);
            pose.orientation.w = std::cos(current_yaw/2.0);
            path.poses.push_back(pose);
        }
        path.poses.push_back(goal.pose); // 最後にゴールを追加
        path_pub_->publish(path);   // pathをpublish
    }
    double get_Yaw(const geometry_msgs::msg::Quaternion &q){return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));}
    double slow_stop_;
    double resolution_;
    double zero_stop_;
    double v_max;
    double freq;
    double position_tolerance_;
    double angle_tolerance_;
    geometry_msgs::msg::Pose current_pose_;
    rclcpp::Publisher   <geometry_msgs::msg::PoseArray>::SharedPtr   path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Publisher   <geometry_msgs::msg::Twist>::SharedPtr err_pub_;
    std::shared_ptr     <tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr     <tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
