#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "custom_msg_pkg/msg/command.hpp"


#include <stdint.h>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <thread>


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

            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&DroneCommander::positionCallback, this, std::placeholders::_1));
            odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&DroneCommander::odometry_callback, this, std::placeholders::_1));


            timer_ = this->create_wall_timer(500ms, std::bind(&DroneCommander::commanderCallback, this));
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
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;

        bool stop_flag_ = false;
        bool hover_flag_ = false;
        bool resume_mission_flag = false;
        int test_count = 0;

        float current_pos_[3] = {0.0, 0.0, 0.0};
        float current_velocity_[3] = {0.0, 0.0, 0.0};


        void checkCommand(const custom_msg_pkg::msg::Command::SharedPtr msg) {
            if(msg->command == custom_msg_pkg::msg::Command::STOP) {
                RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
                stop_flag_ = true;
            }
            else if (msg->command == custom_msg_pkg::msg::Command::TURN) {
                RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "NO STOP");
            }
        }

        void commanderCallback() {
            if(stop_flag_){
                auto pause_msg = px4_msgs::msg::VehicleCommand();
                pause_msg.timestamp = this->get_clock()->now().nanoseconds();
                pause_msg.command = 176;
                pause_msg.param1 = 1;
                pause_msg.param2 = 4;
                pause_msg.param3 = 3;
                pause_msg.target_system = 1;
                pause_msg.target_component = 1;
                pause_msg.source_system = 1;
                pause_msg.source_component = 1;
                pause_msg.from_external = true;
                pause_msg.confirmation = 0;
                vehicle_command_publisher_->publish(pause_msg);

                RCLCPP_INFO(this->get_logger(), "Sent pause command to the vehicle.");
                hover_flag_ = true;
                stop_flag_ = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            if(hover_flag_){
                test_count++;
                RCLCPP_INFO(this->get_logger(), "Test count %d\n", test_count);
            }

            if(hover_flag_ && test_count == 20){
                auto resume_msg = px4_msgs::msg::VehicleCommand();
                resume_msg.timestamp = this->get_clock()->now().nanoseconds();
                resume_msg.command = 176;
                resume_msg.param1 = 1;
                resume_msg.param2 = 4;
                resume_msg.param3 = 4;
                resume_msg.target_system = 1;
                resume_msg.target_component = 1;
                resume_msg.source_system = 1;
                resume_msg.source_component = 1;
                resume_msg.from_external = true;
                resume_msg.confirmation = 0;
                vehicle_command_publisher_->publish(resume_msg);

                RCLCPP_INFO(this->get_logger(), "##### Sent Resume command to the vehicle. #####");
            }
            
        }


        void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {

            current_pos_[0] = msg->x;  // North (m)
            current_pos_[1] = msg->y;  // East (m)
            current_pos_[2] = msg->z;  // Down (m)

            //RCLCPP_INFO(this->get_logger(), "Current position: X: %.2f, Y: %.2f, Z: %.2f", current_pos_[0], current_pos_[1], current_pos_[2]);
        }


        void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            // current_yaw_ = quaternion_to_yaw(msg->q);

            current_velocity_[0] = msg->velocity[0];  // Velocity in the X direction
            current_velocity_[1] = msg->velocity[1];  // Velocity in the Y direction
            current_velocity_[2] = msg->velocity[2];
            RCLCPP_INFO(this->get_logger(), "Current velocity: X: %.2f, Y: %.2f, Z: %.2f", current_velocity_[0], current_velocity_[1], current_velocity_[2]);

        }


        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
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
