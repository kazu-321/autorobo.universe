#include "autorobo_localization/ransac_node.hpp"

namespace ransac_node{
    RansacNode::RansacNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node("ransac_node", node_options) {
        pose_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/current_pose", 10);
        line_pub_     = this->create_publisher<visualization_msgs::msg::Marker>("/localization/ransac/line", 10);
        cloud_pub_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/ransac/cloud", 10);
        scan_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), 
                        std::bind(&RansacNode::scan_callback, this, std::placeholders::_1));
        tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        iter = this->declare_parameter("iter", 100);
        distance_threshold = this->declare_parameter("distance_threshold", 0.025);
        max_lines = this->declare_parameter("max_lines", 3);
        walls_raw = declare_parameter<std::vector<double>>("walls",{-0.7,-0.8, 2.7,-0.8, -0.7,-0.8, -0.7,4.2, -0.7,4.2, 2.7,4.2});
    }

    RansacNode::Wall RansacNode::Line2Wall(std::vector<Point> data,Line ab){
        double a=ab.a;
        double b=ab.b;
        Wall best_wall;
        best_wall.p1.x=-100.0;
        best_wall.p1.y=-100.0;
        best_wall.p2.x= 100.0;
        best_wall.p2.y= 100.0;
        for(size_t i=0;i<data.size();i++){
            double d=fabs(a*data[i].x-data[i].y+b)/sqrt(a*a+1);
            if(d<=distance_threshold){
                if(abs(a)<1.0){
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

    RansacNode::Line RansacNode::ransac(){
        if(data.size()==0){
            Line err_line;
            err_line.a=0.0;
            err_line.b=0.0;
            return err_line;
        }
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
            for(size_t j=0;j<data.size();j++){
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
        std::vector<Point> inliers;
        for(size_t i=0;i<data.size();i++){
            double d=fabs(best_line.a*data[i].x-data[i].y+best_line.b)/sqrt(best_line.a*best_line.a+1);
            if(d<=distance_threshold){
                inliers.push_back(data[i]);
                data.erase(data.begin()+i);
                i--;
            }
        }
        filtered_data.push_back(inliers);
        return best_line;
    }

    std::vector<RansacNode::Line> RansacNode::ransacs(){
        std::vector<Line> lines;
        for(int i=0;i<max_lines;i++){
            if(data.size()==0) break;
            Line line=ransac();
            if(line.a==0.0&&line.b==0.0) break;
            lines.push_back(line);
        }
        return lines;
    }
    
    void RansacNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto start_time=this->now();
        std::vector<Point> scan_points;
        filtered_data.clear();
        for(size_t i = 0; i < msg->ranges.size(); i++) {
            if(msg->ranges[i] < msg->range_max) {
                Point p;
                p.x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment -M_PI/2)-0.5;
                p.y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment -M_PI/2);
                if(-0.5<=p.x&&p.x<=0.5&&-0.5<=p.y&&p.y<0.5) continue;
                else                                        scan_points.push_back(p);
            }
        }
        rclcpp::Time start = this->now();
        data=scan_points;
        std::vector<Line> detect_lines=ransacs();
        std::vector<Wall> detect_walls;
        for(auto line : detect_lines){
            Wall wall=Line2Wall(scan_points,line);
            detect_walls.push_back(wall);
        }
        
         // frontとsideのabから交点座標を計算

        double angle=M_PI/2-atan2(detect_walls[0].p1.y-detect_walls[0].p2.y,detect_walls[0].p1.x-detect_walls[0].p2.x);
        double x_=-(detect_lines[0].b-detect_lines[1].b)/(detect_lines[0].a-detect_lines[1].a);
        double y_=detect_lines[0].a*x_+detect_lines[0].b;

        double x=x_*cos(angle)-y_*sin(angle);
        double y=x_*sin(angle)+y_*cos(angle);

        x=-x-0.7;
        if(detect_lines[1].b<0.0)y=-y-0.8;
        else             y=-y+4.2;

        if(x < 0.0) x = 0.0;
        if(x > 3.0) x = 3.0;
        if(y < -0.5)y =-0.5;
        if(y > 5.0) y = 5.0;

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

        int count=0;
        for(auto points : filtered_data){
            count+=points.size();
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id="base_link";
        cloud.header.stamp=this->now();
        cloud.height = 1;
        cloud.width = count;
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
        for(size_t i=0;i<filtered_data.size();i++){
            for(auto point : filtered_data[i]){
                *iter_x = point.x;
                *iter_y = point.y;
                *iter_z = 0.0;
                *iter_r = 0;
                *iter_g = 0;
                *iter_b = 0;
                if(i==0) *iter_r = 255;
                if(i==1) *iter_g = 255;
                if(i==2) *iter_b = 255;
                ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
            }
        }
        cloud_pub_->publish(cloud);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ransac_node::RansacNode)