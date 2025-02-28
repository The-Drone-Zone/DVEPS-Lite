#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>


#include <chrono>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class DroneCommander : public rclcpp::Node
{
    public:
        DroneCommander(): Node("drone_commander")
        {
            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
            decision_command_subscriber_ = this->create_subscription<custom_msg_pkg::msg::Command>("DecisionController/command", 10, std::bind(&DroneCommander::checkCommand, this, std::placeholders::_1));
            
            timer = this->create_wall_timer(100ms, std::bind(&DroneCommander::CommanderCallback, this));
            offboard_setpoint_counter_ = 0;
        }

        void arm()
        {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

            RCLCPP_INFO(this->get_logger(), "Arm command send");
        }

        void disarm()
        {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

            RCLCPP_INFO(this->get_logger(), "Disarm command send");
        }

    private:
        int offboard_setpoint_counter_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Subscription<custom_msg_pkg::msg::Command>::SharedPtr decision_command_subscriber_;

        bool stop_flag_ = false;

        void checkCommand(const custom_msg_pkg::msg::Command::SharedPtr msg) {
            if(msg->command == custom_msg_pkg::msg::Command::STOP) {
                RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
                stop_flag_ = true;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "NO STOP");
            }
        }

        void CommanderCallback()
        {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            if(stop_flag_) {
                RCLCPP_INFO(this->get_logger(), "STOPPING");
            }
            else
            {
                // offboard_control_mode needs to be paired with trajectory_setpoint
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        }

        void publish_offboard_control_mode()
        {
            OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = true;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg);
        }

        void publish_trajectory_setpoint()
        {
            TrajectorySetpoint msg{};
            msg.position = {0.0, 0.0, -5.0};
            msg.yaw = -3.14; // [-PI:PI]
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_publisher_->publish(msg);
        }

        void publish_vehicle_command(uint16_t command, float param1, float param2)
        {
            VehicleCommand msg{};
            msg.param1 = param1;
            msg.param2 = param2;
            msg.command = command;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);
        }
};


int main(int argc, char *argv[])
{
	std::cout << "Starting DroneCommander node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneCommander>());

	rclcpp::shutdown();
	return 0;
}   