#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class ransac_node : public rclcpp::Node{
public:
    ransac_node() : Node("ransac") {
        pose_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/current_pose", 10);
        line_pub_     = this->create_publisher<visualization_msgs::msg::Marker>("/ransac/line", 10);
        cloud_pub_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ransac/cloud", 10);
        scan_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), 
                        std::bind(&ransac_node::scan_callback, this, std::placeholders::_1));
        tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        iter = this->declare_parameter("iter", 100);
        distance_threshold = this->declare_parameter("distance_threshold", 0.025);

    }

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

    Line ransac(std::vector<Point> data){
        if(data.size()==0){
            Line err_line;
            err_line.a=0.0;
            err_line.b=0.0;
            return err_line;
        }
        Point best_right,best_left;
        int best_inliers=0;
        iter=this->get_parameter("iter").as_int();
        distance_threshold=this->get_parameter("distance_threshold").as_double();
        Line best_line;
        for(int i=0;i<iter;i++){
            Point p1 = data[rand()%data.size()];
            Point p2 = data[rand()%data.size()];
            double a=(p1.y-p2.y)/(p1.x-p2.x);
            double b=p1.y-a*p1.x;
            int inliers=0;
            for(int j=0;j<data.size();j++){
                double d=fabs(a*data[j].x-data[j].y+b)/sqrt(a*a+1);
                if(d<=distance_threshold){
                    inliers++;
                }
            }
            if(inliers > best_inliers){
                best_inliers=inliers;
                best_line.a=a;
                best_line.b=b;
            }
        }
        return best_line;
    }

    Wall getRL(std::vector<Point> data,Line ab,bool TB=false){
        double a=ab.a;
        double b=ab.b;
        Wall best_wall;
        best_wall.p1.x=-100.0;
        best_wall.p1.y=-100.0;
        best_wall.p2.x= 100.0;
        best_wall.p2.y= 100.0;
        for(int i=0;i<data.size();i++){
            double d=fabs(a*data[i].x-data[i].y+b)/sqrt(a*a+1);
            if(d<=distance_threshold){
                if(TB){
                    if(best_wall.p1.x < data[i].x) best_wall.p1=data[i];
                    if(best_wall.p2.x > data[i].x) best_wall.p2=data[i];
                }else{
                    if(best_wall.p1.y < data[i].y) best_wall.p1=data[i];
                    if(best_wall.p2.y > data[i].y) best_wall.p2=data[i];
                }
            }
        }
        return best_wall;
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto start_time=this->now();
        std::vector<Point> scan_points,robot_filter,side_wall,front_wall;
        for(int i = 0; i < msg->ranges.size(); i++) {
            if(msg->ranges[i] < msg->range_max) {
                Point p;
                p.x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment -M_PI/2)-0.5;
                p.y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment -M_PI/2);
                if(-0.5<=p.x&&p.x<=0.5&&-0.5<=p.y&&p.y<0.5) robot_filter.push_back(p);
                else                                        scan_points.push_back(p);
            }
        }
        rclcpp::Time start = this->now();
        // get front wall
        Line front_ab=ransac(scan_points);
        Wall front=getRL(scan_points,front_ab);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id="base_link";
        marker.header.stamp=start_time;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD; 
        marker.ns="front_wall";
        marker.id=0;
        marker.color.g=1.0;
        marker.color.a=1.0;
        marker.scale.x=distance_threshold;
        marker.scale.y=1.0;
        geometry_msgs::msg::Point pr,pl;
        pr.x=front.p1.x;
        pr.y=front.p1.y;
        pl.x=front.p2.x;
        pl.y=front.p2.y;
        marker.points.push_back(pr);
        marker.points.push_back(pl);
        line_pub_->publish(marker);

        // get side scan
        double angle=M_PI/2-atan2(front.p1.y-front.p2.y,front.p1.x-front.p2.x);
        for(int i=0;i<scan_points.size();i++){
            double d=fabs(front_ab.a*scan_points[i].x-scan_points[i].y+front_ab.b)/sqrt(front_ab.a*front_ab.a+1);
            if(d<=distance_threshold) front_wall.push_back(scan_points[i]);
            else                      side_wall.push_back(scan_points[i]);
        }

        // get side_wall
        Line side_ab=ransac(side_wall);
        if(side_ab.a==0.0&&side_ab.b==0.0) return;
        Wall side=getRL(side_wall,side_ab,true);
        pr.x=side.p1.x;
        pr.y=side.p1.y;
        pl.x=side.p2.x;
        pl.y=side.p2.y;
        marker.ns="side_wall";
        marker.points.clear();
        marker.points.push_back(pr);
        marker.points.push_back(pl);
        line_pub_->publish(marker);

        // frontとsideのabから交点座標を計算
        double x_=-(front_ab.b-side_ab.b)/(front_ab.a-side_ab.a);
        double y_=front_ab.a*x_+front_ab.b;

        double x=x_*cos(angle)-y_*sin(angle);
        double y=x_*sin(angle)+y_*cos(angle);

        x=-x-0.7;
        if(side_ab.b<0.0)y=-y-0.8;
        else             y=-y+4.2;

        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.frame_id="map";
        current_pose.header.stamp=this->now();
        current_pose.pose.position.x=x;
        current_pose.pose.position.y=y;
        current_pose.pose.orientation.z=sin(angle / 2.0);
        current_pose.pose.orientation.w=cos(angle / 2.0);        
        pose_pub_->publish(current_pose);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.rotation.z = sin(angle / 2.0);
        transform.transform.rotation.w = cos(angle / 2.0);
        tf_broadcaster_->sendTransform(transform);

        // RCLCPP_INFO(this->get_logger(),"(%lf , %lf , %lf) , %lf ms",x,y,angle*180/M_PI,(this->now().nanoseconds()-start_time.nanoseconds())/1000.0/1000.0,side_ab.b);

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id="base_link";
        cloud.header.stamp=this->now();
        cloud.height = 1;
        cloud.width = robot_filter.size()+front_wall.size()+side_wall.size();
        cloud.is_dense = false;
        cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

        for (const auto& p : robot_filter) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0;
            *iter_r = 0;
            *iter_g = 0;
            *iter_b = 0;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
        for (const auto& p : front_wall) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0;        
            *iter_r = 255;
            *iter_g = 0;
            *iter_b = 0;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
        for (const auto& p : side_wall) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0;
            *iter_r = 0;
            *iter_g = 0;
            *iter_b = 255;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
        cloud_pub_->publish(cloud);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int iter;
    double map_resolution;
    double distance_threshold;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ransac_node>());
    rclcpp::shutdown();
    return 0;
}


