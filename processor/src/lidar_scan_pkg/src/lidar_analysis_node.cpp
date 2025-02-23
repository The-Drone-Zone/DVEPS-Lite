/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_msg_pkg/msg/lidar_position.hpp"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class SLLidarClient : public rclcpp::Node {
   public:
    SLLidarClient() : Node("sllidar_client") {
        lidar_info_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&SLLidarClient::scanCallback, this, std::placeholders::_1));

        analysis_pub = this->create_publisher<custom_msg_pkg::msg::LidarPosition>("Lidar/analysis", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

   private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int count = scan->scan_time / scan->time_increment;
        RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
        RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        // for (int i = 0; i < count; i++) {
        //     float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //     RCLCPP_INFO(this->get_logger(), "angle-distance : [%f, %f]", degree, scan->ranges[i]);
        // }
        RCLCPP_INFO(this->get_logger(), "range_size : %d", count);
        publish_analysis(this->analysis_pub, scan, count);
    }

    void publish_analysis(rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr& pub, 
                          const sensor_msgs::msg::LaserScan::SharedPtr scan,
                          int count )
    {
        custom_msg_pkg::msg::LidarPosition msg;


        msg.z.assign(scan->ranges.begin(), scan->ranges.end());
        msg.x.resize(count, 0.0);
        msg.y.resize(count, 0.0);

        pub->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub_;
    rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr analysis_pub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
