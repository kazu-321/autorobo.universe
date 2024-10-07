#include "rclcpp/rclcpp.hpp"
#include "autorobo_msgs/msg/twistring.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>

class OmniSim : public rclcpp::Node {
public:
    OmniSim() : Node("omni_sim_node") ,count_(0),old_x_(0.0),old_y_(0.0),old_z_(0.0),x_(0.0),y_(0.0),z_(0.0) {
        twistring_subscriber_= this->create_subscription<autorobo_msgs::msg::Twistring>("/R1", 10, 
                              std::bind(&OmniSim::velocityCallback, this, std::placeholders::_1));
        lidar_pub_          = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
        tf_broadcaster_     = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_              = this->create_wall_timer(std::chrono::milliseconds(50), 
                              std::bind(&OmniSim::timerCallback, this));
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        old_x_ = this->declare_parameter<double>("start_x", 0.0);
        old_y_ = this->declare_parameter<double>("start_y", 0.0);
        lidar_err_=this->declare_parameter<double>("lidar_err",0.0);
        angle_max_=this->declare_parameter<double>("angle_max",M_PI);
        angle_min_=this->declare_parameter<double>("angle_min",-M_PI);
        lidar_resolution_=this->declare_parameter<int>("lidar_resolution",180);
        lidar_freq_=this->declare_parameter<int>("lidar_freq",10);
        lidar_timer_        = this->create_wall_timer(std::chrono::milliseconds(1/lidar_freq_*1000),
                              std::bind(&OmniSim::lidarCallback, this));
        sig_     =false;
        servo_[0]=false;
        servo_[1]=false;
        got_tf=false;
    }

private:
    void velocityCallback(const autorobo_msgs::msg::Twistring::SharedPtr msg) {
        count_++;
        twist_sum_.linear.x+=msg->twist.linear.x  *1.5;
        twist_sum_.linear.y+=msg->twist.linear.y  *1.5;
        twist_sum_.angular.z+=msg->twist.angular.z*1.5;
        if(msg->cmd=="c")     sig_=true;
        if(msg->cmd=="p")     sig_=false;
        if(msg->cmd=="s 0 0") servo_[0]=false;
        if(msg->cmd=="s 1 0") servo_[1]=false;
        if(msg->cmd=="s 0 1") servo_[0]=true;
        if(msg->cmd=="s 1 1") servo_[1]=true;
    }

    void timerCallback() {
        // TFのブロードキャスト
        geometry_msgs::msg::TransformStamped transformStamped;
        x_=old_x_;
        y_=old_y_;
        z_=old_z_;
        if(count_!=0&&sig_){
            z_+=twist_sum_.angular.z/count_*0.05;
            double angle = std::atan2(twist_sum_.linear.y, twist_sum_.linear.x)+z_;
            double distance = std::sqrt(twist_sum_.linear.x*twist_sum_.linear.x+twist_sum_.linear.y*twist_sum_.linear.y)/count_;
            x_+=distance*std::cos(angle)*0.05;
            y_+=distance*std::sin(angle)*0.05;
        }

        if(x_<-0.2) x_=-0.2;
        if(x_> 1.8) x_= 1.8;
        if(y_<-0.3) y_=-0.3;
        if(y_> 3.7) y_= 3.7;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link_sim";
        transformStamped.transform.translation.x = x_; // 更新されたx位置
        transformStamped.transform.translation.y = y_; // 更新されたy位置
        transformStamped.transform.translation.z = 0.0; // z位置は0

        tf2::Quaternion q;
        while(z_<-M_PI)z_+=M_PI*2;
        while(z_>M_PI)z_-=M_PI*2;
        q.setRPY(0, 0, z_); // 更新された姿勢（ロール、ピッチ、ヨー）
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);

        old_x_ = x_;
        old_y_ = y_;
        old_z_ = z_;
        twist_sum_.linear.x = 0;
        twist_sum_.linear.y = 0;
        twist_sum_.angular.z = 0;
        count_ = 0;
        RCLCPP_INFO(this->get_logger(),"sig: %d, servo l:%d r:%d",sig_,servo_[1],servo_[0]);
        // RCLCPP_INFO(this->get_logger(), "x: %lf , y: %lf , z: %lf", x_, y_, z_);
    }

    void send_scan(){ // get distance from robot to wall
        // lidar_err_=this->get_parameter("lidar_err").as_double();
        // float x=x_+0.7-0.5*std::cos(z_);
        // float y=y_+0.8-0.5*std::sin(z_);
        // float yaw=z_;
        // float right_angle = M_PI/2+std::atan2(y,x);
        // float left_angle  = M_PI/2-std::atan2(5.0-y,x);

        // sensor_msgs::msg::LaserScan scan;
        // scan.header.frame_id="laser_frame";
        // scan.header.stamp=this->get_clock()->now();
        // scan.angle_max=0;
        // scan.angle_min=-M_PI;
        // scan.angle_increment=M_PI/180;
        // scan.range_max=12.0;
        // scan.range_min=0.1;
        // scan.scan_time=0.05;
        // scan.time_increment=scan.scan_time/180.0;
        // // RCLCPP_INFO(this->get_logger(),"right:%f,left%f",right_angle,left_angle);
        // for(int i=0;i<180;i++){
        //     float angle=i*M_PI/180;
        //     if(angle+yaw<left_angle){
        //         // 左側の壁
        //         scan.ranges.push_back((5.0-y)/cosf(angle+yaw));
        //     }else if(angle+yaw>right_angle){
        //         // 右側の壁
        //         scan.ranges.push_back(y/cosf(M_PI-angle-yaw));
        //     }else{
        //         // 前の壁
        //         scan.ranges.push_back(x/cosf(M_PI/2-angle-yaw));
        //     }
        //     scan.ranges.back()+=lidar_err_*(2.0*(static_cast<double>(std::rand())/RAND_MAX)-1.0)/1000.0;
        // }
        // lidar_pub_->publish(scan);

        if(!got_tf){
            try {
                lidar_tf_ = tf_buffer_->lookupTransform("base_link", "laser_frame", tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform base_link to laser_frame: %s", ex.what());
                return;
            }
        }    
        float x=lidar_tf_.transform.translation.x + x_;
        float y=lidar_tf_.transform.translation.y + y_;
        float z= get_Yaw(lidar_tf_.transform.orientation);


        sensor_msgs::msg::LaserScan scan;
        scan.header.frame_id = "laser_frame";
        scan.header.stamp = this->get_clock()->now();
        scan.angle_max = angle_max_;
        scan.angle_min = angle_min_;
        scan.angle_increment =  M_PI / lidar_resolution_;
        scan.range_max = 12.0;
        scan.range_min = 0.1;
        scan.scan_time = 1/lidar_freq_;
        scan.time_increment = 1/lidar_freq_ /lidar_resolution_;

        for(int i=0; i<(angle_max_-angle_min_)/M_PI*lidar_resolution_; i++){
            float angle = scan.angle_min + i * scan.angle_increment + z;
            flota min_dist = scan.range_min;
            for(const auto& wall : walls){

            }
        }
    }
    double get_Yaw(const geometry_msgs::msg::Quaternion &q){return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));}
    rclcpp::Subscription<autorobo_msgs::msg::Twistring>::SharedPtr       twistring_subscriber_;
    rclcpp::Publisher   <sensor_msgs::msg::LaserScan>::SharedPtr     lidar_pub_;
    std::unique_ptr     <tf2_ros::TransformBroadcaster>              tf_broadcaster_;
    rclcpp::Publisher   <geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    geometry_msgs::msg::TransformStamped lidar_tf_;
    visualization_msgs::msg::Markre walls;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_lidar_;
    geometry_msgs::msg::Twist twist_sum_;
    int count_, lidar_freq_, lidar_resolution_;
    double old_x_, old_y_, old_z_, x_, y_, z_, lidar_err_,  angle_max_, angle_min_;
    bool sig_, servo_[2], got_tf;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniSim>());
    rclcpp::shutdown();
    return 0;
}