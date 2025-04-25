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
#include <deque>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class SLLidarClient : public rclcpp::Node {
   public:
    SLLidarClient() : Node("sllidar_client") {
        lidar_info_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&SLLidarClient::scanCallback, this, std::placeholders::_1));

        analysis_pub = this->create_publisher<custom_msg_pkg::msg::LidarPosition>("Lidar/analysis", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

   private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub_;
    rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr analysis_pub;

    std::deque<std::vector<float>> scan_history_;
    const size_t HISTORY_SIZE = 5;


    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        int count = scan->scan_time / scan->time_increment;
        // RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
        // RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        std::vector<float> current_ranges = scan->ranges;
        if (scan_history_.size() >= HISTORY_SIZE) {
            scan_history_.pop_front();
        }
        scan_history_.push_back(current_ranges);

        // RCLCPP_INFO(this->get_logger(), "range_size : %d", count);
        publish_analysis(this->analysis_pub, scan, count, scan->scan_time);
    }

    void publish_analysis(rclcpp::Publisher<custom_msg_pkg::msg::LidarPosition>::SharedPtr& pub, 
                          const sensor_msgs::msg::LaserScan::SharedPtr scan,
                          int count, float time_bewteen_full_scans = 0.1)
    {
        int danger_beam_count = 0;
        custom_msg_pkg::msg::LidarPosition msg;
        msg.stop = false;
        //msg.least_val = 50;

        if (scan_history_.size() < 2) return;

        //go through each beam in the 360 degree scan
        for(int i = 0; i < count; ++i){
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if (degree < 0) {
                degree += 360.0;
            }
            // RCLCPP_WARN(this->get_logger(), "DEGREE: %f, Range: %f", degree, scan->ranges[i]); // DEBUG DELETE LATER

            //Angles outisde the drone hitbox at 20 meters can be ignored.
            if( !(degree > 178.5 && degree < 181.5) && !( degree > 60.5 && degree < 64.5 ) && !( degree > 306.5  && degree < 310.5 ) ){
                continue;
            }

            // if(scan->ranges[i] < msg.least_val) {
            //     msg.least_val = scan->ranges[i]; //find the least range value in the scan
            // }

            //least range value pass into message.
            float ZERO = .001;
            std::vector<float> range_over_time;
            for (const auto& past_scan : scan_history_) {
                float value = past_scan[i];
                if (value > ZERO && value < 40.00) {
                    range_over_time.push_back(value);
                }
            }

            if (range_over_time.size() < 3) continue; // Not enough data to analyze

            //DEBUG ONLY DELETE BEFORE PR
            // std::cout << "Values at index " << i << " (degree: " << degree << "): ";
            // for (const auto& r : range_over_time) {
            //     std::cout << r << " ";
            // }
            // std::cout << std::endl;

            float total_velocity = 0.0;
            for (size_t j = 1; j < range_over_time.size(); ++j) {
                float v = (range_over_time[j - 1] - range_over_time[j]) / time_bewteen_full_scans; //distance over time is velocity
                total_velocity += v;
            }
            float avg_velocity = total_velocity / (range_over_time.size() - 1); //minus 1 because we are looking at the intervals bewtween the ranges

            // if (avg_velocity > 2 || total_velocity > 2){
            //     RCLCPP_WARN(this->get_logger(), "total_velocity: %f", total_velocity); // DEBUG DELETE LATER
            //     RCLCPP_WARN(this->get_logger(), "avg_velocity  : %f", avg_velocity); // DEBUG DELETE LATER
            //     std::cout << "Values at index " << i << " (degree: " << degree << "): ";
            //     for (const auto& r : range_over_time) {
            //         std::cout << r << " ";
            //     }
            //     std::cout << std::endl;
            // }

            float current_range = range_over_time.back(); //most recent range data

            if (avg_velocity > 0.0) {
                //RCLCPP_WARN(this->get_logger(), "avg_velocity  : %f", avg_velocity); // DEBUG DELETE LATER
                float collision_time = current_range / avg_velocity; //collision_time = time to collision based on the most recent rnge value and the average velocity of the object
                // RCLCPP_WARN(this->get_logger(), "collision_time  : %f", collision_time); // DEBUG DELETE LATER

                if (collision_time < 4.0) { //4.0 because we are going at 5/ms and 4*5 = 20 meters
                    danger_beam_count++; //Here I am using multiple beams to signal a stop not just 1 beam
                }

                if (danger_beam_count >= 3) { //I just chose a randum number 3 as the beam number.
                   break;
                }
            }
        }

        if (danger_beam_count >= 3) { //I just chose a randum number 3 as the beam number.
            RCLCPP_INFO(this->get_logger(), "DANGER BEAM: STOP STOP STOP");
            msg.stop = true;
        }

        // pub->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLLidarClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




// sensor_msgs::msg::LaserScan current_scan = *scan;
//         float sum_of_potential_x = 0.0;
//         float sum_of_potential_y = 0.0;
//         bool avoid = false;

//         custom_msg_pkg::msg::LidarPosition msg;
//         msg.stop = false;
        
//         for(int i = 0; i < count; ++i) {
//             float d0 = 25; // ignore values under 20 meters we missed an object and our design was flawed. also a requiremtn to detect objects at 20 meters
//             float k = 0.5; //gain / weightedness of potential field
            
//             //filter our object whose values are under 20 meters
//             if(current_scan.ranges[i] < d0 && current_scan.ranges[i] > current_scan.range_min) {
//                 msg.stop = true;

//                 //MEASURED IN RADIANS
//                 float X_radians = cos(scan->angle_min + scan->angle_increment * i);
//                 float Y_radians = sin(scan->angle_min + scan->angle_increment * i);
//                 float potential = -.5 * k * pow( ( (1/current_scan.ranges[i]) - (1/d0) ), 2 );
//                 sum_of_potential_x += X_radians * potential;
//                 sum_of_potential_y += Y_radians * potential;
//             }
            
//         }

//         msg.sum_of_potential_x = sum_of_potential_x;
//         msg.sum_of_potential_y = sum_of_potential_y;

//         //XYZ COORDINATE MAPPING GOES HERE
        
//         msg.z.assign(scan->ranges.begin(), scan->ranges.end());
//         msg.x.resize(count, 0.0);
//         msg.y.resize(count, 0.0);

//         pub->publish(msg);