#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSubscriber : public rclcpp::Node {
public:
    LidarSubscriber() : Node("lidar_sub_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarSubscriber::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty()) {
            RCLCPP_INFO(this->get_logger(), "First scan range: %.2f meters", msg->ranges[0]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received an empty scan.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSubscriber>());
    rclcpp::shutdown();
    return 0;
}
