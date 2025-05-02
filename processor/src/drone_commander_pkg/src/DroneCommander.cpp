#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include "custom_msg_pkg/msg/command.hpp"
#include "custom_msg_pkg/msg/command_ack.hpp"


#include <drone_commander_pkg/PosFunctions.hpp>
#include <drone_commander_pkg/ActuatorControl.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <thread>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class DroneCommander : public rclcpp::Node//, public std::enable_shared_from_this<DroneCommander>
{
    public:
        DroneCommander(): Node("drone_commander")
        {
            RCLCPP_INFO(this->get_logger(), "START DRONE COMMANDER");

            //passing the drone commander itself into the PositionControl class. 
            position_control_ = nullptr; 
            uart_ = std::make_shared<UART>();

            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

            command_ack_publisher_ = this->create_publisher<custom_msg_pkg::msg::CommandAck>("DecisionController/command_ack", 10);
            decision_command_subscriber_ = this->create_subscription<custom_msg_pkg::msg::Command>("DecisionController/command", 10, std::bind(&DroneCommander::checkCommand, this, std::placeholders::_1));

            //This is needed for all telemetry we want from the drone. must follow this way of subscribing. found in px4 documentation
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
            vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack", qos, std::bind(&DroneCommander::vehicle_command_ack_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(500ms, std::bind(&DroneCommander::commanderCallback, this));
            offboard_setpoint_counter_ = 0;
        }

        void initPositionControl(std::shared_ptr<DroneCommander> DC)
        {
            //initialized this way because of shared messages and subcribers
            position_control_ = std::make_shared<PositionControl>(DC);
        }

    private:
        int offboard_setpoint_counter_;
        std::shared_ptr<PositionControl> position_control_;
        std::shared_ptr<UART> uart_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Publisher<custom_msg_pkg::msg::CommandAck>::SharedPtr command_ack_publisher_;

        rclcpp::Subscription<custom_msg_pkg::msg::Command>::SharedPtr decision_command_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;

        bool stop_flag_ = false;
        bool turn_flag_ = false;
        bool first_turn = true;
        bool commanded_turn = false;
        bool in_offboard_control_ = false;
        bool forward_flag_ = false;
        bool first_forward = true;
        bool resume_flag_ = false;
        bool hover_flag_ = false;
        bool resume_mission_flag = false;
        bool keep_checking_height = true;
        int last_command_recieved = -1;
        double degree_ = 0.0;
        std::array<float, 3> current_pos = {};
        float position_X_copy = 0.0;

        void checkCommand(const custom_msg_pkg::msg::Command::SharedPtr msg) {
            last_command_recieved = msg->command;
            switch(msg->command) {
                case custom_msg_pkg::msg::Command::STOP:
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d STOP STOP STOP", msg->command);
                    stop_flag_ = true;
                    turn_flag_ = false;
                    forward_flag_ = false;
                    break;
                case custom_msg_pkg::msg::Command::TURN: //HIJACKED TO JUST CONTINUE MISSION RIGHT NOW.
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d TURN TURN TURN", msg->command);
                    turn_flag_ = true;
                    commanded_turn = true;
                    stop_flag_ = false;
                    forward_flag_ = false;
                    if(first_turn){
                        command_offboard_control_mode();
                        first_turn = false;
                    }
                    degree_ = msg->deg;
                    RCLCPP_INFO(this->get_logger(), "degree %f", degree_);
                    //no ack so forward will not happen yet
                    break;
                case custom_msg_pkg::msg::Command::FORWARD:
                    RCLCPP_INFO(this->get_logger(), "Received Command: %d FORWARD FORWARD FORWARD", msg->command);
                    turn_flag_ = false; //probably rework the transition to forward so we stop turning exactly when we want too. Maybe
                    forward_flag_ = true;
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "NO STOP");
                    break;
            }
        }

        void commanderCallback() {    
            
            if(position_control_->checkAnalysisHeight() && keep_checking_height){
                custom_msg_pkg::msg::CommandAck msg_ack;
                msg_ack.command = 0;
                msg_ack.result = 0;
                msg_ack.height_reached = true;
                command_ack_publisher_->publish(msg_ack);
                RCLCPP_INFO(this->get_logger(), "HEIGHT IS GOOD");
                keep_checking_height = false;
            }

            if(stop_flag_){
                //command do set mode, custom mode px4 (1), px4 mode auto mission(4), px4 sub mode hold(3)
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3); //Get rid of magic numbers later

                RCLCPP_INFO(this->get_logger(), "Sent pause command to the vehicle.");
                uart_->sendDistance(10);
                hover_flag_ = true;
                stop_flag_ = false;
            }
            else if (turn_flag_ && in_offboard_control_){
                //publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0); //Get rid of magic numbers later
                RCLCPP_INFO(this->get_logger(), "\n START turn command to the vehicle.");
                    px4_msgs::msg::TrajectorySetpoint msg;

                    if(commanded_turn){
                        msg = position_control_->turnByAngle(degree_, true);
                    }
                    else {
                        msg = position_control_->turnByAngle(degree_);
                    }

                    publishOffboardCtlMsg();
                    trajectory_setpoint_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "SENT turn command to the vehicle.");

                if(commanded_turn){
                    custom_msg_pkg::msg::CommandAck msg_ack;
                    msg_ack.command = 2;
                    msg_ack.result = 0;
                    command_ack_publisher_->publish(msg_ack);
                    commanded_turn = false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500)); //DO NOT DELETE
            }
            else if (forward_flag_){

                if(first_forward){
                    current_pos = position_control_->getLocalPosition();
                    position_X_copy = current_pos[0];
                    current_pos[0] += 5.0;
                }

                publishOffboardCtlMsg();
                trajectory_setpoint_publisher_->publish(position_control_->moveForwardByMeters(current_pos[0]));

                if(first_forward){
                    custom_msg_pkg::msg::CommandAck msg_ack;
                    msg_ack.command = 3;
                    msg_ack.result = 0;
                    command_ack_publisher_->publish(msg_ack);
                    first_forward = false;
                }

                if(position_control_->checkDist(position_X_copy))
                {
                    forward_flag_ = false;
                    resume_flag_ = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(450)); //Under 2hz to stay in offboard mode

                // for(int i = 0; i < 5; ++i){
                //     publishOffboardCtlMsg();
                //     trajectory_setpoint_publisher_->publish(position_control_->moveForward());

                //     custom_msg_pkg::msg::CommandAck msg_ack;
                //     msg_ack.command = 3;
                //     msg_ack.result = 0;
                //     command_ack_publisher_->publish(msg_ack);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(450)); //Under 2hz to stay in offboard mode
                // }

                //publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 4);
            }
            else if (resume_flag_){
                uart_->sendDistance(256); //CHANGE TO CORRECT DISTANCE LATER FOR HORIZONTAL
                RCLCPP_INFO(this->get_logger(), "Sent resume command to the vehicle.");
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 4);
                resume_flag_ = false;
            }
        }


        void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
            if(msg->result == 0) {
                switch(msg->command) {
                    case VehicleCommand::VEHICLE_CMD_DO_SET_MODE: { //braces are because i declare a varible inside case statement
                        RCLCPP_INFO(this->get_logger(), "Vehicle mode set");
                        RCLCPP_INFO(this->get_logger(), "Last command: %d", last_command_recieved);

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

            for(int i = 0; i <= 10; ++i){
                if (offboard_setpoint_counter == 10) {
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    RCLCPP_INFO(this->get_logger(), "offboard_control_mode");
                    in_offboard_control_ = true;
                    break;
                }
    
                //what is inside of these should not matter only needed in order to command the vehicle into this mode.
                publishOffboardCtlMsg();
                publishTrajectorySetpoint();
    
                if (offboard_setpoint_counter < 11) {
                    offboard_setpoint_counter++;
                }
            }
            RCLCPP_INFO(this->get_logger(), "command_offboard_control_mode()");
        }
};


int main(int argc, char *argv[])
{
	std::cout << "Starting DroneCommander node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    auto drone_commander = std::make_shared<DroneCommander>();
    drone_commander->initPositionControl(drone_commander);
	rclcpp::spin(drone_commander);
	rclcpp::shutdown();
	return 0;
}   
