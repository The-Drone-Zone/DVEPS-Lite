#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include "custom_msg_pkg/msg/command.hpp"
#include "custom_msg_pkg/msg/command_ack.hpp"



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

            command_ack_publisher_ = this->create_publisher<custom_msg_pkg::msg::CommandAck>("DecisionController/command_ack", 10);
            decision_command_subscriber_ = this->create_subscription<custom_msg_pkg::msg::Command>("DecisionController/command", 10, std::bind(&DroneCommander::checkCommand, this, std::placeholders::_1));

            //This is needed for all telemetry we want from the drone. must follow this way of subscribing. found in px4 documentation
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&DroneCommander::positionCallback, this, std::placeholders::_1));
            odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&DroneCommander::odometry_callback, this, std::placeholders::_1));
            vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack", qos, std::bind(&DroneCommander::vehicle_command_ack_callback, this, std::placeholders::_1));

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
        rclcpp::Publisher<custom_msg_pkg::msg::CommandAck>::SharedPtr command_ack_publisher_;

        rclcpp::Subscription<custom_msg_pkg::msg::Command>::SharedPtr decision_command_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;

        bool stop_flag_ = false;
        bool turn_flag_ = false;
        bool forward_flag_ = false;
        bool hover_flag_ = false;
        bool resume_mission_flag = false;
        int last_command_recieved = -1;

        float current_pos_[3] = {0.0, 0.0, 0.0};
        float current_velocity_[3] = {0.0, 0.0, 0.0};
        float current_yaw_ = 0.0;


        void checkCommand(const custom_msg_pkg::msg::Command::SharedPtr msg) {
            last_command_recieved = msg->command;
            switch(msg->command) {
                case custom_msg_pkg::msg::Command::STOP:
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
                    stop_flag_ = true;
                    turn_flag_ = false;
                    forward_flag_ = false;
                    break;
                case custom_msg_pkg::msg::Command::TURN:
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
                    turn_flag_ = true;
                    command_offboard_control_mode();
                    //no ack so forward will not happen yet
                    break;
                case custom_msg_pkg::msg::Command::FORWARD:
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d", msg->command);
                    turn_flag_ = false; //probably rework the transition to forward so we stop turning exactly when we want too. Maybe
                    forward_flag_ = true;
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "NO STOP");
                    break;
            }
        }

        void commanderCallback() {
            if(stop_flag_){
                //command do set mode, custom mode px4 (1), px4 mode auto mission(4), px4 sub mode hold(3)
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3); //Get rid of magic numbers later

                RCLCPP_INFO(this->get_logger(), "Sent pause command to the vehicle.");
                hover_flag_ = true;
                stop_flag_ = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); //DO NOT DELTE IDK SLEEP IS NESSISARY
            }
            else if (turn_flag_){
                static bool initialized = false;  // Tracks if yaw_angle has been set
                static float yaw_angle;           // Static variable to store yaw
                static bool increasing = true;     // Direction of rotation

                // Initialize yaw_angle only once, using the latest yaw reading
                if (!initialized) {
                    yaw_angle = current_yaw_;
                    RCLCPP_INFO(this->get_logger(), "Current yaw: %.2f degrees", current_yaw_ * (180.0 / M_PI));
                    initialized = true;
                }

                // Publish the yaw setpoint
                auto trajectory_setpoint_msg = TrajectorySetpoint();
                trajectory_setpoint_msg.position[0] = current_pos_[0];  // Hold position
                trajectory_setpoint_msg.position[1] = current_pos_[1];
                trajectory_setpoint_msg.position[2] = current_pos_[2];
                trajectory_setpoint_msg.yaw = yaw_angle;

                publishOffboardCtlMsg();
                trajectory_setpoint_publisher_->publish(trajectory_setpoint_msg);

                RCLCPP_INFO(this->get_logger(), "Yaw Command Sent: %.2f degrees", yaw_angle * (180.0 / M_PI));

                // Define oscillation range
                float max_yaw = current_yaw_ + (M_PI / 2);  // +90 degrees
                float min_yaw = current_yaw_ - (M_PI / 2);  // -90 degrees

                // Adjust yaw for next function call
                if (increasing) {
                    yaw_angle += 5.0 * (M_PI / 180.0);  // Rotate clockwise
                    if (yaw_angle >= max_yaw) {  // Reverse direction at +90°
                        increasing = false;
                    }
                } else {
                    yaw_angle -= 5.0 * (M_PI / 180.0);  // Rotate counterclockwise
                    if (yaw_angle <= min_yaw) {  // Reverse direction at -90°
                        increasing = true;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); //DO NOT DELTE IDK SLEEP IS NESSISARY
            }
            else if (forward_flag_){
                auto trajectory_setpoint_msg = TrajectorySetpoint();
                trajectory_setpoint_msg.position[0] = current_pos_[0] + 5.0;  // Move forward 5 meters
                trajectory_setpoint_msg.position[1] = current_pos_[1];
                trajectory_setpoint_msg.position[2] = current_pos_[2];
                trajectory_setpoint_msg.yaw = current_yaw_;

                publishOffboardCtlMsg();
                trajectory_setpoint_publisher_->publish(trajectory_setpoint_msg);

                RCLCPP_INFO(this->get_logger(), "Forward Command Sent: %.2f meters", trajectory_setpoint_msg.position[0]);
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
            current_yaw_ = atan2(2.0 * (msg->q[3] * msg->q[2] + msg->q[0] * msg->q[1]),
                                 1.0 - 2.0 * (msg->q[1] * msg->q[1] + msg->q[2] * msg->q[2]));

            current_velocity_[0] = msg->velocity[0];  // Velocity in the X direction
            current_velocity_[1] = msg->velocity[1];  // Velocity in the Y direction
            current_velocity_[2] = msg->velocity[2];
            // RCLCPP_INFO(this->get_logger(), "Current velocity: X: %.2f, Y: %.2f, Z: %.2f", current_velocity_[0], current_velocity_[1], current_velocity_[2]);

        }


        void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
            if(msg->result == 0) {
                switch(msg->command) {
                    case VehicleCommand::VEHICLE_CMD_DO_SET_MODE: { //braces are because i declare a varible inside case statement
                        RCLCPP_INFO(this->get_logger(), "Vehicle mode set");
                        custom_msg_pkg::msg::CommandAck msg_ack;
                        msg_ack.command = last_command_recieved;
                        msg_ack.result = 0;
                        command_ack_publisher_->publish(msg_ack);
                        break;
                    }
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


        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0)
        {
            auto msg = px4_msgs::msg::VehicleCommand();
            msg.command = command;
            msg.param1 = param1;
            msg.param2 = param2;
            msg.param3 = param3;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.confirmation = 0;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(msg);
        }

        void publishOffboardCtlMsg() {
            auto offboard_control_mode_msg = OffboardControlMode();
            offboard_control_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_msg.position = true;
            offboard_control_mode_msg.velocity = false;
            offboard_control_mode_msg.acceleration = false;
            offboard_control_mode_msg.attitude = false;
            offboard_control_mode_msg.body_rate = false;
            offboard_control_mode_publisher_->publish(offboard_control_mode_msg);
        }

        void publishTrajectorySetpoint() {
            auto trajectory_setpoint_msg = TrajectorySetpoint();
            trajectory_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_msg.position = {0.0, 0.0, 0.0};
            trajectory_setpoint_msg.yaw = 0.0;
            trajectory_setpoint_publisher_->publish(trajectory_setpoint_msg);
        }


        void command_offboard_control_mode() {
            int offboard_setpoint_counter = 0;
            if (offboard_setpoint_counter == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            }

            //what is inside of these should not matter onyy needed inorder to command the vehicle into this mode.
            publishOffboardCtlMsg();
            publishTrajectorySetpoint();

            if (offboard_setpoint_counter < 11) {
                offboard_setpoint_counter++;
            }
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
