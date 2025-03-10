#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

#include "camera_scan_pkg/msg/obstacle.hpp"
#include "camera_scan_pkg/msg/obstacle_array.hpp"
#include "custom_msg_pkg/msg/lidar_position.hpp"
#include "custom_msg_pkg/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"


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
                "Lidar/analysis", 10, std::bind(&DecisionController::lidarCallback, this, std::placeholders::_1));

            command_publisher_ = this->create_publisher<custom_msg_pkg::msg::Command>("DecisionController/command", 10);

            // running_ = true;
            // background_thread_ = std::thread(&DecisionController::background_loop, this);
        }

        // ~DecisionController() {
        //     running_ = false;
        //     cv_.notify_one();  // Wake up thread to exit cleanly
        //     if (background_thread_.joinable()) {
        //         background_thread_.join();
        //     }
        // }


    private:
        rclcpp::Subscription<camera_scan_pkg::msg::ObstacleArray>::SharedPtr image_subscription_;
        rclcpp::Subscription<custom_msg_pkg::msg::LidarPosition>::SharedPtr lidar_subscription_;
        rclcpp::Publisher<custom_msg_pkg::msg::Command>::SharedPtr command_publisher_;

        // std::thread background_thread_;
        // std::mutex mutex_;
        // std::condition_variable cv_;
        // std::atomic<bool> running_;
        // bool keep_turning_ = false;
        bool keep_stop_ = true;


        void imageCallback(const camera_scan_pkg::msg::ObstacleArray::SharedPtr msg) {
            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "Received %zu analyzed image obstacles", msg->obstacles.size());

            //printImageObstacles(msg);
            // std::lock_guard<std::mutex> lock(mutex_);
            if(msg->obstacles.size() > 8) {
                if(keep_stop_)
                {
                    RCLCPP_INFO(this->get_logger(), "STOP STOP STOP");
                    publish_control_command(custom_msg_pkg::msg::Command::STOP);
                    // keep_turning_ = true;
                    // RCLCPP_INFO(this->get_logger(), "Notifying background thread to send TURN command...");
                    // cv_.notify_one();
                    // RCLCPP_INFO(this->get_logger(), "Notifying background thread to send TURN command...");
                    keep_stop_ = false;
                }
            }
            // else 
            // {
            //     keep_turning_ = false;
            // }
            

            // Image to LiDAR map function call goes here

            // End time
            auto end = std::chrono::high_resolution_clock::now();
            // Compute duration
            std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds >(end - start);
            // Print analysis time
            // RCLCPP_INFO(this->get_logger(), "Decision Time: %ld ms", duration_ms.count());
            time_sum += duration_ms.count();
            counter += 1;
            // RCLCPP_INFO(this->get_logger(), "Current Average Decision FPS: %ld fps", 1000 / (time_sum / counter));
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

        // void background_loop() {
        //     RCLCPP_INFO(this->get_logger(), "BACKGROUND LOOP");
        //     while (running_) {
        //         std::unique_lock<std::mutex> lock(mutex_);
        //         RCLCPP_INFO(this->get_logger(), "BACKGROUND LOOP 2");
        //         cv_.wait(lock, [this] { return keep_turning_ || !running_; }); //loop does not run untill told to do so
        //         RCLCPP_INFO(this->get_logger(), "BACKGROUND LOOP 3");
        //         if (!running_) {
        //             break;
        //         }
                
        //         while (keep_turning_ && running_) {
        //             RCLCPP_INFO(this->get_logger(), "TURN TURN TURN");
        //             publish_control_command(custom_msg_pkg::msg::Command::TURN, 90.0);
        //             lock.unlock();
        //             std::this_thread::sleep_for(std::chrono::seconds(1));
        //             lock.lock();
        //         }
        //     }
        // }

        void publish_control_command(const int32_t &in_command, int turn_degree) {
            custom_msg_pkg::msg::Command command_msg;
            command_msg.command = in_command;
            command_msg.turn_deg = turn_degree;
            command_publisher_->publish(command_msg);
        }

        void publish_control_command(const int32_t &in_command) {
            custom_msg_pkg::msg::Command command_msg;
            command_msg.command = in_command;
            command_publisher_->publish(command_msg);
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