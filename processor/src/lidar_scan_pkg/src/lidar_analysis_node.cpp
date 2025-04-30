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
#include <deque>

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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub_;
    rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr analysis_pub;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr pixhawk_pub;

    // std::deque<std::vector<float>> scan_history_;
    std::vector<float> scan_history_;
    const size_t HISTORY_SIZE = 5;
    int danger_count_ = 0;


    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int count = scan->scan_time / scan->time_increment;
        // RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
        // RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        // For testing lidar messages (we can change which points we publish later)
        px4_msgs::msg::ObstacleDistance GCS_msg = px4_msgs::msg::ObstacleDistance();
        GCS_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        GCS_msg.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
        GCS_msg.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
        GCS_msg.increment = 1125;  // degrees per measurement
        GCS_msg.min_distance = 100;  // 100 cm = 1 m
        GCS_msg.max_distance = 4000;  // 4000 cm = 40 m
        GCS_msg.angle_offset = 0.0f;

        for (int i = 0; i < count; i+= 45) {
            if (i <= count) {
                float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
                RCLCPP_INFO(this->get_logger(), "angle-distance : [%f, %f]", degree, scan->ranges[i]);
                GCS_msg.distances[i / 45] = scan->ranges[i] * 100; // need to convert distance to cm (if not already)
                GCS_msg.increment = scan->angle_increment * 45 * 10000;
            }
        }

        RCLCPP_INFO(this->get_logger(), "range_size : %d", count);
        pixhawk_pub->publish(GCS_msg);
        publish_analysis(this->analysis_pub, scan, count, scan->scan_time);
    }

    void publish_analysis(rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr& pub, 
                          const sensor_msgs::msg::LaserScan::SharedPtr scan,
                          int count, float time_bewteen_full_scans = 0.1)
    {
        int danger_beam_count = 0;
        custom_msg_pkg::msg::LidarPosition msg;
        msg.stop = false;
        msg.least_range = 100;

        for(int i = 0; i < count; ++i) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            if( !(degree > 0 && degree < 1.0) && !( degree > 121.0 && degree < 124.0 ) && !( degree > 230.0  && degree < 233.0 ) && !(degree > 358.9 && degree < 359.99) ){
                continue;                
            }

            if(scan->ranges[i] < msg.least_range) {
                msg.least_range = scan->ranges[i]; //find the least range value in the scan
            }
        }

        if (scan_history_.size() >= HISTORY_SIZE) {
            scan_history_.erase(scan_history_.begin());
        }
        push_back_max_size(scan_history_, msg.least_range, HISTORY_SIZE);

        if (scan_history_.size() < 2) return;

        float total_velocity = 0.0;
        for(int i = 1; i < scan_history_.size(); ++i) {
            float v = (scan_history_[i - 1] - scan_history_[i]) / time_bewteen_full_scans; //distance over time is velocity
            total_velocity += v;
        }
        float average_velocity = total_velocity / (scan_history_.size() - 1); //minus 1 because we are looking at the intervals bewtween the ranges
        float current_range = scan_history_.back(); //most recent range data

        if(average_velocity > 0.0) {
            float collision_time = current_range / average_velocity; //collision_time = time to collision based on the most recent rnge value and the average velocity of the object
            //RCLCPP_INFO(this->get_logger(), "Collision Time: %f", collision_time);
            if (collision_time < 4.0) { //4.0 because we are going at 5/ms and 4*5 = 20 meters
                msg.stop = true; //Here I am using multiple beams to signal a stop not just 1 beam
            }
        }


        // if (scan_history_.size() < 2) return;

        // //go through each beam in the 360 degree scan
        // for(int i = 0; i < count; ++i){
        //     float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

        //     //Angles outisde the drone hitbox at 20 meters can be ignored.
        //     if( !(degree > 0 && degree < 1.0) && !( degree > 121.0 && degree < 124.0 ) && !( degree > 230.0  && degree < 233.0 ) && !(degree > 358.9 && degree < 359.99) ){
        //         continue;                
        //     }

        //     if(scan->ranges[i] < msg.least_range) {
        //         msg.least_range = scan->ranges[i]; //find the least range value in the scan
        //     }

        //     //least range value pass into message.
        //     std::vector<float> range_over_time;
        //     for (const auto& past_scan : scan_history_) {
        //         if (i < past_scan.size()) {
        //             range_over_time.push_back(past_scan[i]);
        //         }
        //     }

        //     if (range_over_time.size() < 2) continue; // Not enough data to analyze

        //     float total_velocity = 0.0;
        //     for (size_t j = 1; j < range_over_time.size(); ++j) {
        //         float v = (range_over_time[j - 1] - range_over_time[j]) / time_bewteen_full_scans; //distance over time is velocity
        //         total_velocity += v;
        //     }
        //     float average_velocity = total_velocity / (range_over_time.size() - 1); //minus 1 because we are looking at the intervals bewtween the ranges

        //     float current_range = range_over_time.back(); //most recent range data

        //     if (avg_velocity > 0.0) {
        //         float collision_time = current_range / avg_velocity; //collision_time = time to collision based on the most recent rnge value and the average velocity of the object
        //         if (collision_time < 4.0) { //4.0 because we are going at 5/ms and 4*5 = 20 meters
        //             danger_beam_count++; //Here I am using multiple beams to signal a stop not just 1 beam
        //         }
        //     }
        // }

        // if (danger_beam_count >= 3) { //I just chose a randum number 3 as the beam number. 
        //     msg.stop = true;
        // }

        pub->publish(msg);
    }

    void push_back_max_size(std::vector<float>& vec, float value, size_t max_size) {
        if (vec.size() >= max_size) {
            vec.erase(vec.begin());  // Remove the oldest element
        }
        vec.push_back(value);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}