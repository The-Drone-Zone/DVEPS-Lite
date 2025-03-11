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
            command_ack_subscrption_ = this->create_subscription<custom_msg_pkg::msg::LidarPosition>(
                "Lidar/analysis", 10, std::bind(&DecisionController::lidarCallback, this, std::placeholders::_1));

            command_publisher_ = this->create_publisher<custom_msg_pkg::msg::Command>("DecisionController/command", 10);
            command_ack_subscriber_ = this->create_subscription<custom_msg_pkg::msg::CommandAck>("DecisionController/command_ack", 10, std::bind(&DecisionController::command_ack_callback, this, std::placeholders::_1));

            find_path_thread = std::thread(&DecisionController::find_path, this);

        }


    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<camera_scan_pkg::msg::ObstacleArray>::SharedPtr image_subscription_;
        rclcpp::Subscription<custom_msg_pkg::msg::LidarPosition>::SharedPtr lidar_subscription_;
        rclcpp::Publisher<custom_msg_pkg::msg::Command>::SharedPtr command_publisher_;
        rclcpp::Subscription<custom_msg_pkg::msg::Command>::SharedPtr command_ack_subscrption_;

        std::thread find_path_thread;


        int decision_mode = 0;
        bool outstanding_ack= false;


        void imageCallback(const camera_scan_pkg::msg::ObstacleArray::SharedPtr msg) {
            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "Received %zu analyzed image obstacles", msg->obstacles.size());

            //printImageObstacles(msg);


            if(msg->obstacles.size() > 8 && decision_mode == 0) {
                
                RCLCPP_INFO(this->get_logger(), "STOP STOP STOP");
                publish_control_command(custom_msg_pkg::msg::Command::STOP);
            }
            

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


        void command_ack_subscrption_(const custom_msg_pkg::msg::CommandAck::SharedPtr msg) {
            if(msg->result == 0) {
                switch(msg->command) {
                    case custom_msg_pkg::msg::Command::STOP:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Stopped");
                        decision_mode = 1;
                        break;
                    case custom_msg_pkg::msg::Command::TURN:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Turned");
                        decision_mode = 2;
                        break;
                    case custom_msg_pkg::msg::Command::FORWARD:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Moved Forward");
                        decision_mode = 3;
                        break;
                    case custom_msg_pkg::msg::Command::DEFAULT:
                        RCLCPP_INFO(this->get_logger(), "Vehicle command acknowledged");
                        decision_mode = 0;
                        break;
                    default:
                        RCLCPP_INFO(this->get_logger(), "Vehicle command acknowledged");
                        break;
                }
                RCLCPP_INFO(this->get_logger(), "Vehicle command acknowledged");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Vehicle command failed");
            }
        }

        void find_path() {
            while(rclcpp::ok()) {
                if(decision_mode == 1) {
                    RCLCPP_INFO(this->get_logger(), "Finding Path / Turnning Drone");
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // for test only making sure drone stops appropriatly. 
                    publish_control_command(custom_msg_pkg::msg::Command::TURN);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //replace maybe with new logic for decision component
                    // decision_mode = 2; commented for test also proabbly should just delte.
                }
                else if(decision_mode == 2) {
                    RCLCPP_INFO(this->get_logger(), "Stopping Turn / Moving Forward");
                    publish_control_command(custom_msg_pkg::msg::Command::FORWARD);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //replace maybe with new logic for decision component
                    decision_mode = 3;
                }
                else if (decision_mode == 3) {
                    RCLCPP_INFO(this->get_logger(), "Resume Normal Operation");
                    publish_control_command(custom_msg_pkg::msg::Command::DEFAULT);
                    decision_mode = 0;
                }
            }
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