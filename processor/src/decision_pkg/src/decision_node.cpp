#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_scan_pkg/msg/obstacle.hpp"
#include "camera_scan_pkg/msg/obstacle_array.hpp"
#include "custom_msg_pkg/msg/lidar_position.hpp"

class DecisionController : public rclcpp::Node {
    public:
        long int time_sum = 0;
        long int counter = 0;

        DecisionController() : Node("decision_node") {
            // Create Subscriber to Image Analysis
            image_subscription_ = this->create_subscription<camera_scan_pkg::msg::ObstacleArray>(
                "output_obstacles", 10, std::bind(&DecisionController::imageCallback, this, std::placeholders::_1));
            
            // Create Subscriber to LiDAR Analysis
            lidar_subscription_ = this->create_subscription<custom_msg_pkg::msg::LidarPosition>(
                "output_obstacles", 10, std::bind(&DecisionController::lidarCallback, this, std::placeholders::_1));
        }
    private:
        rclcpp::Subscription<camera_scan_pkg::msg::ObstacleArray>::SharedPtr image_subscription_;
        rclcpp::Subscription<custom_msg_pkg::msg::LidarPosition>::SharedPtr lidar_subscription_;

        void imageCallback(const camera_scan_pkg::msg::ObstacleArray::SharedPtr msg) {
            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "Received %zu analyzed image obstacles", msg->obstacles.size());

            printImageObstacles(msg);

            // Image to LiDAR map function call goes here

            // End time
            auto end = std::chrono::high_resolution_clock::now();
            // Compute duration
            std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds >(end - start);
            // Print analysis time
            RCLCPP_INFO(this->get_logger(), "Decision Time: %ld ms", duration_ms.count());
            time_sum += duration_ms.count();
            counter += 1;
            RCLCPP_INFO(this->get_logger(), "Current Average Decision FPS: %ld fps", 1000 / (time_sum / counter));
        }

        void lidarCallback(const custom_msg_pkg::msg::LidarPosition::SharedPtr msg) {
            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "Received %zu analyzed LiDAR samples", msg->x.size());

            // Image to LiDAR map function call goes here

            // End time
            auto end = std::chrono::high_resolution_clock::now();
            // Compute duration
            std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds >(end - start);
            // Print analysis time
            RCLCPP_INFO(this->get_logger(), "Decision Time: %ld ms", duration_ms.count());
            time_sum += duration_ms.count();
            counter += 1;
            RCLCPP_INFO(this->get_logger(), "Current Average Decision FPS: %ld fps", 1000 / (time_sum / counter));
        }

    void printImageObstacles(const camera_scan_pkg::msg::ObstacleArray::SharedPtr msg) {
        for (camera_scan_pkg::msg::Obstacle obstacle : msg->obstacles) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detect at: (%lf, %lf)", obstacle.x, obstacle.y);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionController>());
    rclcpp::shutdown();
    return 0;
}