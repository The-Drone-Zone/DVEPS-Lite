#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

#include "camera_scan_pkg/msg/obstacle.hpp"
#include "camera_scan_pkg/msg/obstacle_array.hpp"
#include "custom_msg_pkg/msg/lidar_position.hpp"
#include "custom_msg_pkg/msg/command.hpp"
#include "custom_msg_pkg/msg/command_ack.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

enum STATES {
    DEFAULT = 0,
    STOP,
    TURN,
    REROUTING,
    FORWARD
};

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
            command_ack_subscrption_ = this->create_subscription<custom_msg_pkg::msg::CommandAck>("DecisionController/command_ack", 10, std::bind(&DecisionController::command_ack_callback, this, std::placeholders::_1));

            find_path_thread = std::thread(&DecisionController::find_path, this);
            decision_mode.store(STATES::DEFAULT, std::memory_order_release); //DEAFULT==0
            current_state.store(STATES::DEFAULT, std::memory_order_release); //DEAFULT==0

        }


    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<camera_scan_pkg::msg::ObstacleArray>::SharedPtr image_subscription_;
        rclcpp::Subscription<custom_msg_pkg::msg::LidarPosition>::SharedPtr lidar_subscription_;
        rclcpp::Publisher<custom_msg_pkg::msg::Command>::SharedPtr command_publisher_;
        rclcpp::Subscription<custom_msg_pkg::msg::CommandAck>::SharedPtr command_ack_subscrption_;

        std::thread find_path_thread;


        std::atomic<STATES> decision_mode;
        std::atomic<STATES> current_state;
        

        std::atomic<bool> outstanding_ack;

        camera_scan_pkg::msg::ObstacleArray image_obstacles;
        custom_msg_pkg::msg::LidarPosition lidar_samples;
        int turn_count = 0;


        void imageCallback(const camera_scan_pkg::msg::ObstacleArray::SharedPtr msg) {
            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "Received %zu analyzed image obstacles", msg->obstacles.size());

            //printImageObstacles(msg);
            image_obstacles = *msg;

            if(msg->tracked_obstacle && current_state.load(std::memory_order_acquire) == 0 && !outstanding_ack.load(std::memory_order_acquire)) {
                RCLCPP_INFO(this->get_logger(), "STOP STOP STOP");
                publish_control_command(custom_msg_pkg::msg::Command::STOP);
                outstanding_ack.store(true, std::memory_order_release); 
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

            // Image to LiDAR map function call goes here
            if(msg->stop) {
                RCLCPP_INFO(this->get_logger(), "STOPPING");
                //my drone command PR needs to pushed to dev before I pull in changes here.
            }

            //PROCESS XYZ COORDINATE MAPPING HERE MAYBE

            //NEED TO HAVE THE DRONE COMMAND PUSHED IN ORDER TO USE the current location functionality.

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


        // The variable current_state is used to keep track of the current state of the drone.
        // it only gets updated when an acknowledgement is received from the DroneCommander telling the decision_node
        // that the drone has successfully completed the command. ie the last completed command is the current state.
        void command_ack_callback(const custom_msg_pkg::msg::CommandAck::SharedPtr msg) {
            if(msg->result == 0) {
                switch(msg->command) {
                    case custom_msg_pkg::msg::Command::STOP:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Stopped");
                        current_state.store(STATES::STOP, std::memory_order_release); //STOP==1
                        break;
                    case custom_msg_pkg::msg::Command::TURN:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Turned");
                        current_state.store(STATES::TURN, std::memory_order_release); //STOP==2
                        break;
                    case custom_msg_pkg::msg::Command::FORWARD:
                        RCLCPP_INFO(this->get_logger(), "Vehicle Moved Forward");
                        current_state.store(STATES::FORWARD, std::memory_order_release); //STOP==3
                        break;
                    // case custom_msg_pkg::msg::Command::DEFAULT:
                    //     RCLCPP_INFO(this->get_logger(), "Vehicle DEFAULT");
                    //     current_state.store(STATES::DEFAULT, std::memory_order_release); //DEFAULT==0
                    //     break;
                    default:
                        RCLCPP_INFO(this->get_logger(), "Vehicle command acknowledged");
                        break;
                }
                outstanding_ack.store(false, std::memory_order_release);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Vehicle command failed");
            }
        }

        void find_path() {
            while(rclcpp::ok()) {

                
                //the current state is stopped and we need to start turning
                if(current_state.load(std::memory_order_acquire) == STATES::STOP && outstanding_ack.load(std::memory_order_acquire) == false){
                    RCLCPP_INFO(this->get_logger(), "STOP STATE");
                    turn_count++;
                    if(turn_count < 24) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10000)); //replace maybe with new logic for decision component
                        RCLCPP_INFO(this->get_logger(), "PUBLISHED FIRST TURN");
                        publish_control_command(custom_msg_pkg::msg::Command::TURN);
                        outstanding_ack.store(true, std::memory_order_release);
                    }
                    
                }
                else if(current_state.load(std::memory_order_acquire) == STATES::TURN && outstanding_ack.load(std::memory_order_acquire) == false) {
                    RCLCPP_INFO(this->get_logger(), "TURN STATE");
                   current_state.store(STATES::REROUTING, std::memory_order_release);
                }
                else if (current_state.load(std::memory_order_acquire) == STATES::REROUTING && outstanding_ack.load(std::memory_order_acquire) == false) {
                    //chill for a bit
                    RCLCPP_INFO(this->get_logger(), "Rerouting");
                    if(image_obstacles.tracked_obstacle) {
                        turn_count++;
                        RCLCPP_INFO(this->get_logger(), "TURN COUNT, %d ", turn_count);


                        if(turn_count == 11){
                            RCLCPP_INFO(this->get_logger(), "BIG TURN");
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //replace maybe with new logic for decision component
                            publish_control_command(custom_msg_pkg::msg::Command::TURN, -165);
                        }
                        else if (turn_count < 11) {
                            RCLCPP_INFO(this->get_logger(), "LITTLE TURN");
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //replace maybe with new logic for decision component
                            publish_control_command(custom_msg_pkg::msg::Command::TURN);
                        }
                        else if (turn_count < 23){
                            RCLCPP_INFO(this->get_logger(), "OPPOSITE LITTLE TURN");
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); //replace maybe with new logic for decision component
                            publish_control_command(custom_msg_pkg::msg::Command::TURN, -15.0);
                        }
                        else {
                            current_state.store(STATES::STOP, std::memory_order_release); //STOP==1
                        }
                        outstanding_ack.store(true, std::memory_order_release);
                    }


                    else {
                        RCLCPP_INFO(this->get_logger(), "Published first forward ");
                        publish_control_command(custom_msg_pkg::msg::Command::FORWARD);
                        outstanding_ack.store(true, std::memory_order_release);
                        turn_count = 0;
                    }

                }
                else if(current_state.load(std::memory_order_acquire) == STATES::FORWARD && outstanding_ack.load(std::memory_order_acquire) == false) {
                    RCLCPP_INFO(this->get_logger(), "Stopping Turn / Moving Forward");
                    // publish_control_command(custom_msg_pkg::msg::Command::DEFAULT);
                    current_state.store(STATES::DEFAULT, std::memory_order_release); //DEFAULT==0
                    // outstanding_ack.store(true, std::memory_order_release);
                }
            }
        }

        void publish_control_command(const int32_t &in_command, const int32_t &degree = 15) {
            custom_msg_pkg::msg::Command command_msg;
            command_msg.command = in_command;
            command_msg.deg = degree;
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