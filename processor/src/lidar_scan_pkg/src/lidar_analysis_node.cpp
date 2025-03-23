/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

 //ADD MIT LICENSE AS WELL

#include "math.h"
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_msg_pkg/msg/lidar_position.hpp"
#include <px4_msgs/msg/obstacle_distance.hpp>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class SLLidarClient : public rclcpp::Node {
   public:
    SLLidarClient() : Node("sllidar_client") {
        lidar_info_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&SLLidarClient::scanCallback, this, std::placeholders::_1));

        analysis_pub = this->create_publisher<custom_msg_pkg::msg::LidarPosition>("Lidar/analysis", rclcpp::QoS(rclcpp::KeepLast(10)));
        pixhawk_pub = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
    }

   private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int count = scan->scan_time / scan->time_increment;
        RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
        RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        // For testing lidar messages (we can change which points we publish later)
        px4_msgs::msg::ObstacleDistance GCS_msg;
        for (int i = 0; i < count; i+= 45) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            RCLCPP_INFO(this->get_logger(), "angle-distance : [%f, %f]", degree, scan->ranges[i]);
            GCS_msg.distances[i] = scan->ranges[i];
            GCS_msg.increment = scan->angle_increment * 45;
        }

        RCLCPP_INFO(this->get_logger(), "range_size : %d", count);
        pixhawk_pub->publish(GCS_msg);
        publish_analysis(this->analysis_pub, scan, count);
    }

    void publish_analysis(rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr& pub, 
                          const sensor_msgs::msg::LaserScan::SharedPtr scan,
                          int count )
    {

        sensor_msgs::msg::LaserScan current_scan = *scan;
        flaot sum_of_potential_x = 0.0;
        float sum_of_potential_y = 0.0;
        bool avoid = false;

        custom_msg_pkg::msg::LidarPosition msg;
        msg.stop = false;
        
        for(int i = 0; i < count; ++i) {
            float d0 = 25; // ignore values under 20 meters we missed an object and our design was flawed. also a requiremtn to detect objects at 20 meters
            float k = 0.5; //gain / weightedness of potential field
            
            //filter our object whose values are under 20 meters
            if(current_scan.ranges[i] < d0 && current_scan.ranges[i] > current_scan.range_min) {
                msg.stop = true;

                //MEASURED IN RADIANS
                float X_radians = cos(scan->angle_min + scan->angle_increment * i);
                float Y_radians = sin(scan->angle_min + scan->angle_increment * i);
                float potential = -.5 * k * pow( ( (1/current_scan.ranges[i]) - (1/d0) ), 2 );
                sum_of_potential_x += X_radians * potential;
                sum_of_potential_y += Y_radians * potential;
            }
            
        }

        msg.sum_of_potential_x = sum_of_potential_x;
        msg.sum_of_potential_y = sum_of_potential_y;

        //XYZ COORDINATE MAPPING GOES HERE
        
        msg.z.assign(scan->ranges.begin(), scan->ranges.end());
        msg.x.resize(count, 0.0);
        msg.y.resize(count, 0.0);

        pub->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub_;
    rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr analysis_pub;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr pixhawk_pub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
