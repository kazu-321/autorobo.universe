#include "autorobo_simulation/simulation_node.hpp"

namespace simulation_node{
    OmniSim::OmniSim(const rclcpp::NodeOptions & node_options) : rclcpp::Node("omni_sim_node") ,count_(0),old_x_(0.0),old_y_(0.0),old_z_(0.0),x_(0.0),y_(0.0),z_(0.0) {
        twistring_subscriber_= this->create_subscription<autorobo_msgs::msg::Twistring>("/R1", 10, 
                              std::bind(&OmniSim::velocityCallback, this, std::placeholders::_1));
        lidar_pub_          = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
        tf_broadcaster_     = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_              = this->create_wall_timer(std::chrono::milliseconds(50), 
                              std::bind(&OmniSim::timerCallback, this));    
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        old_x_ = this->declare_parameter<double>("start_x", 0.0);
        old_y_ = this->declare_parameter<double>("start_y", 0.0);
        lidar_err_=this->declare_parameter<double>("lidar_err",0.0);
        angle_max_=this->declare_parameter<double>("angle_max",0.0);
        angle_min_=this->declare_parameter<double>("angle_min",-M_PI);
        lidar_resolution_=this->declare_parameter<int>("lidar_resolution",180);
        lidar_freq_=this->declare_parameter<int>("lidar_freq",10);
        lidar_timer_        = this->create_wall_timer(std::chrono::milliseconds(1/lidar_freq_*1000),
                              std::bind(&OmniSim::lidarCallback, this));
        walls_raw = declare_parameter<std::vector<double>>("walls",{-0.7,-0.8, 2.7,-0.8, -0.7,-0.8, -0.7,4.2, -0.7,4.2, 2.7,4.2});
        sig_     =true;
        servo_[0]=false;
        servo_[1]=false;
        got_tf=false;
    
        for(size_t i=0;i<walls_raw.size();i+=4){
            Wall wall;
            wall.x1=walls_raw[i];
            wall.y1=walls_raw[i+1];
            wall.x2=walls_raw[i+2];
            wall.y2=walls_raw[i+3];
            walls.push_back(wall);
        }
    }

    void OmniSim::velocityCallback(const autorobo_msgs::msg::Twistring::SharedPtr msg) {
        count_++;
        twist_sum_.linear.x+=msg->twist.linear.x  *1.5;
        twist_sum_.linear.y+=msg->twist.linear.y  *1.5;
        twist_sum_.angular.z+=msg->twist.angular.z*1.5;
        if(msg->cmd!=""){
            if(msg->cmd=="c")     sig_=true;
            if(msg->cmd=="p")     sig_=false;
            if(msg->cmd=="s 0 0") servo_[0]=false;
            if(msg->cmd=="s 1 0") servo_[1]=false;
            if(msg->cmd=="s 0 1") servo_[0]=true;
            if(msg->cmd=="s 1 1") servo_[1]=true;
            RCLCPP_INFO(this->get_logger(), "電源: %d, 射出ロック解除: 右:%d 左:%d", sig_, servo_[0], servo_[1]);
        }
    }

    void OmniSim::timerCallback() {
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
        // RCLCPP_INFO(this->get_logger(), "x: %lf , y: %lf , z: %lf", x_, y_, z_);
    }

    void OmniSim::lidarCallback(){ // get distance from robot to wall
        if(!got_tf){
            try {
                lidar_tf_ = tf_buffer_->lookupTransform("base_link", "laser_frame", tf2::TimePointZero);
                got_tf=true;
                RCLCPP_INFO(this->get_logger(), "tf: x%f, y%f, z%f",lidar_tf_.transform.translation.x,lidar_tf_.transform.translation.y,get_Yaw(lidar_tf_.transform.rotation));
            }
            catch (tf2::TransformException &ex) {
                return;
            }
        }    
        float x=x_ + lidar_tf_.transform.translation.x*std::cos(z_) + lidar_tf_.transform.translation.y*std::sin(z_);
        float y=y_ + lidar_tf_.transform.translation.x*std::sin(z_) + lidar_tf_.transform.translation.y*std::cos(z_);
        float z=-get_Yaw(lidar_tf_.transform.rotation)+ z_;

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
            float min_dist = std::numeric_limits<float>::infinity();
            for(const auto& wall : walls){
                float dist = compute_ray_wall_intersection(x, y, angle, wall);
                if (dist > scan.range_min && dist < min_dist) {
                    min_dist = dist;
                }
            }
            scan.ranges.push_back(min_dist);
        }
        lidar_pub_->publish(scan);
    }

    float OmniSim::compute_ray_wall_intersection(float lx, float ly, float angle, Wall wall){
        // レイの方向
        float dx = std::cos(angle);
        float dy = std::sin(angle);

        // 壁の端点
        float x1 = wall.x1, y1 = wall.y1;
        float x2 = wall.x2, y2 = wall.y2;

        // 2D線分とレイの交差計算（パラメトリック方程式）
        float denominator = (x2 - x1) * dy - (y2 - y1) * dx;
        if (std::abs(denominator) < 1e-6) {
            return std::numeric_limits<float>::infinity();  // 平行
        }

        float t = ((lx - x1) * dy - (ly - y1) * dx) / denominator;
        float u = ((x2 - x1) * (ly - y1) - (y2 - y1) * (lx - x1)) / denominator;

        if (t >= 0.0 && t <= 1.0 && u > 0.0) {
            return u;  // 交差点までの距離
        }
        return std::numeric_limits<float>::infinity();  // 交差しない
    }

    double OmniSim::get_Yaw(const geometry_msgs::msg::Quaternion &q){return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));}

};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simulation_node::OmniSim)