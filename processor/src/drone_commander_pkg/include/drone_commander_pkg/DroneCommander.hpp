#ifndef DRONE_COMMANDER_H
#define DRONE_COMMANDER_H

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include "custom_msg_pkg/msg/command.hpp"
#include "custom_msg_pkg/msg/command_ack.hpp"

#include <memory>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class PositionControl; // Forward declaration to avoid circular dependency

class DroneCommander : public rclcpp::Node, public std::enable_shared_from_this<DroneCommander> {
public:
    DroneCommander();
    void initPositionControl(std::shared_ptr<DroneCommander> DC);
    
private:
    void checkCommand(const custom_msg_pkg::msg::Command::SharedPtr msg);
    void commanderCallback();
    void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    void publishOffboardCtlMsg();
    void publishTrajectorySetpoint();
    void command_offboard_control_mode();

    int offboard_setpoint_counter_;
    std::shared_ptr<PositionControl> position_control_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<custom_msg_pkg::msg::CommandAck>::SharedPtr command_ack_publisher_;
    
    rclcpp::Subscription<custom_msg_pkg::msg::Command>::SharedPtr decision_command_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;

    bool stop_flag_ = false;
    bool turn_flag_ = false;
    bool forward_flag_ = false;
    bool hover_flag_ = false;
    bool resume_mission_flag = false;
    int last_command_received_ = -1;
};

#endif // DRONE_COMMANDER_H