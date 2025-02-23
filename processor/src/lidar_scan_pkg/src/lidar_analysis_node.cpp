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
        lidar_info_sub_ = this->create_subscription<custom_msg_pkg::msg::LidarPosition>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&SLLidarClient::scanCallback, this, std::placeholders::_1));
    }

   private:
    void scanCallback(const custom_msg_pkg::msg::LidarPosition::SharedPtr scan) {
        int count = scan->laser_scan.scan_time / scan->laser_scan.time_increment;
        RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->laser_scan.header.frame_id.c_str(), count);
        RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->laser_scan.angle_min), RAD2DEG(scan->laser_scan.angle_max));

        for (int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->laser_scan.angle_min + scan->laser_scan.angle_increment * i);
            RCLCPP_INFO(this->get_logger(), "angle-distance : [%f, %f]", degree, scan->laser_scan.ranges[i]);
        }
    }

    rclcpp::Subscription<custom_msg_pkg::msg::LidarPosition>::SharedPtr lidar_info_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
