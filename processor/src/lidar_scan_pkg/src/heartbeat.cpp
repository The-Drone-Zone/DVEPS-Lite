#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/vehicle_command.hpp"

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node
{
public:
    HeartbeatNode()
    : Node("heartbeat_node")
    {
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_ = this->create_wall_timer(1s, std::bind(&HeartbeatNode::send_heartbeat, this));

        RCLCPP_INFO(this->get_logger(), "Companion heartbeat node started.");
    }

private:
    void send_heartbeat()
    {
        auto now = this->get_clock()->now();
        px4_msgs::msg::VehicleCommand cmd = px4_msgs::msg::VehicleCommand();
        cmd.timestamp = now.nanoseconds() / 1000;  // microseconds

        cmd.param1 = 0.0;
        cmd.param2 = 0.0;
        cmd.param3 = 0.0;
        cmd.param4 = 0.0;
        cmd.param5 = 0.0;
        cmd.param6 = 0.0;
        cmd.param7 = 0.0;

        // safe command ID — this is a custom, non-actionable command (3000+ are reserved for custom)
        cmd.command = 31000;

        cmd.target_system = 1; // PX4 (typically system ID 1)
        cmd.target_component = 1; // Autopilot (MAV_COMP_ID_AUTOPILOT1)
        cmd.source_system = 2; // Unique ID for Jetson
        cmd.source_component = 196; // OnBoard for Obstacle avoidance (MAV_COMP_ID_OBSTACLE_AVOIDANCE)

        cmd.from_external = true;
        cmd.confirmation = 0;

        publisher_->publish(cmd);
        RCLCPP_DEBUG(this->get_logger(), "Sent dummy vehicle_command as heartbeat");
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatNode>());
    rclcpp::shutdown();
    return 0;
}
