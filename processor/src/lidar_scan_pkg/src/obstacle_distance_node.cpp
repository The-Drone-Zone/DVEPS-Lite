#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include "common/mavlink.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class ObstacleDistancePublisher : public rclcpp::Node {
public:
    ObstacleDistancePublisher()
        : Node("obstacle_distance_publisher")
    {
        lidar_info_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&ObstacleDistancePublisher::send_obstacle_distance, this, std::placeholders::_1));

        serial_port_ = open_serial("/dev/ttyUSB1");
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }
        scan_counter_ = 0;
    }

    ~ObstacleDistancePublisher() {
        if (serial_port_ > 0)
            close(serial_port_);
    }

private:
    int serial_port_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub_;
    size_t scan_counter_;

    int open_serial(const std::string& port) {
        // Open the serial port:
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) return -1;
        fcntl(fd, F_SETFL, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
        // Get current terminal attributes
        struct termios options{};
        tcgetattr(fd, &options);
    
        // Set baud rate for input and output
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
    
        // Enable receiver, set local mode
        options.c_cflag |= (CLOCAL | CREAD);
    
        // Set 8 data bits
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    
        // No parity
        options.c_cflag &= ~PARENB;
    
        // One stop bit
        options.c_cflag &= ~CSTOPB;
    
        // Disable hardware flow control (RTS/CTS)
        options.c_cflag &= ~CRTSCTS;
    
        // Apply the configuration immediately
        tcsetattr(fd, TCSANOW, &options);
    
        return fd;
    }

    void send_obstacle_distance(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Only send every 3 scans
        if (scan_counter_++ % 3 != 0) {
            return;
        }

        int count = scan->scan_time / scan->time_increment;
        RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]", scan->header.frame_id.c_str(), count);
        RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // Horizontal LiDAR Line
        mavlink_obstacle_distance_t horizontal;
        horizontal.time_usec = this->now().nanoseconds() / 1000;
        horizontal.sensor_type = MAV_DISTANCE_SENSOR_LASER;
        horizontal.increment = (360.0 / count) * 1000; // Must be passed as int (even tho docs say float)
        horizontal.min_distance = 100;
        horizontal.max_distance = 50000;
        horizontal.increment_f = NAN;
        horizontal.angle_offset = -3.0f;
        horizontal.frame = MAV_FRAME_BODY_FRD;
        // Bottom left to top right diagonal line (diagonal1)
        mavlink_obstacle_distance_t diagonal1;
        diagonal1.time_usec = this->now().nanoseconds() / 1000;
        diagonal1.sensor_type = MAV_DISTANCE_SENSOR_LASER;
        diagonal1.increment = (360.0 / count) * 1000; // Must be passed as int (even tho docs say float)
        diagonal1.min_distance = 100;
        diagonal1.max_distance = 50000;
        diagonal1.increment_f = NAN;
        diagonal1.angle_offset = 119.0f;
        diagonal1.frame = MAV_FRAME_BODY_FRD;
        // Top left to bottom right diagonal line (diagonal2)
        mavlink_obstacle_distance_t diagonal2;
        diagonal2.time_usec = this->now().nanoseconds() / 1000;
        diagonal2.sensor_type = MAV_DISTANCE_SENSOR_LASER;
        diagonal2.increment = (360.0 / count) * 1000; // Must be passed as int (even tho docs say float)
        diagonal2.min_distance = 100;
        diagonal2.max_distance = 50000;
        diagonal2.increment_f = NAN;
        diagonal2.angle_offset = 228.0f;
        diagonal2.frame = MAV_FRAME_BODY_FRD;

        // Set distances to "no value read" by default
        std::fill_n(horizontal.distances, 72, 0xFFFF);
        std::fill_n(diagonal1.distances, 72, 0xFFFF);
        std::fill_n(diagonal2.distances, 72, 0xFFFF);
        
        int h = 0;
        int d1 = 0;
        int d2 = 0;
        for (int i = 0; i < count; ++i) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            if (degree < 0) {
                degree += 360;
            }
            // RCLCPP_INFO(this->get_logger(), "angle-distance : [%f, %f]", degree, scan->ranges[i]);
            if (degree >= 0 && degree <= 3) {
                horizontal.distances[h] = scan->ranges[i] * 100; // need to convert distance to cm (starts in mm)
                ++h;
            }
            else if (degree >= 119 && degree <= 125.5) {
                diagonal1.distances[d1] = scan->ranges[i] * 100; // need to convert distance to cm (starts in mm)
                ++d1;
            }
            else if (degree >= 228 && degree <= 234.5) {
                diagonal2.distances[d2] = scan->ranges[i] * 100; // need to convert distance to cm (starts in mm)
                ++d2;
            }
        }

        // Send Horizontal Line
        mavlink_msg_obstacle_distance_pack(
            2, 191, &msg,
            horizontal.time_usec,
            horizontal.sensor_type,
            horizontal.distances,
            horizontal.increment,
            horizontal.min_distance,
            horizontal.max_distance,
            horizontal.increment_f,
            horizontal.angle_offset,
            horizontal.frame
        );
        int len = mavlink_msg_to_send_buffer(buffer, &msg);
        int total_written = 0;
        while (total_written < len) {
            int n = write(serial_port_, buffer + total_written, len - total_written);
            if (n == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Horz: Write EAGAIN, retrying...");
                    usleep(1000);  // wait 1 ms
                    continue;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Horz: Write failed: %s", strerror(errno));
                    break;
                }
            }
            total_written += n;
        }
        RCLCPP_INFO(this->get_logger(), "Sent OBSTACLE_DISTANCE Horizontal");

        // Send Diagonal1 Line
        mavlink_msg_obstacle_distance_pack(
            2, 191, &msg,
            diagonal1.time_usec,
            diagonal1.sensor_type,
            diagonal1.distances,
            diagonal1.increment,
            diagonal1.min_distance,
            diagonal1.max_distance,
            diagonal1.increment_f,
            diagonal1.angle_offset,
            diagonal1.frame
        );
        len = mavlink_msg_to_send_buffer(buffer, &msg);
        total_written = 0;
        while (total_written < len) {
            int n = write(serial_port_, buffer + total_written, len - total_written);
            if (n == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Diag1: Write EAGAIN, retrying...");
                    usleep(1000);  // wait 1 ms
                    continue;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Diag1: Write failed: %s", strerror(errno));
                    break;
                }
            }
            total_written += n;
        }
        RCLCPP_INFO(this->get_logger(), "Sent OBSTACLE_DISTANCE Diagonal1");
        // Send Diagonal2 Line
        mavlink_msg_obstacle_distance_pack(
            2, 191, &msg,
            diagonal2.time_usec,
            diagonal2.sensor_type,
            diagonal2.distances,
            diagonal2.increment,
            diagonal2.min_distance,
            diagonal2.max_distance,
            diagonal2.increment_f,
            diagonal2.angle_offset,
            diagonal2.frame
        );
        len = mavlink_msg_to_send_buffer(buffer, &msg);
        total_written = 0;
        while (total_written < len) {
            int n = write(serial_port_, buffer + total_written, len - total_written);
            if (n == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Diag2: Write EAGAIN, retrying...");
                    usleep(1000);  // wait 1 ms
                    continue;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Diag2: Write failed: %s", strerror(errno));
                    break;
                }
            }
            total_written += n;
        }
        RCLCPP_INFO(this->get_logger(), "Sent OBSTACLE_DISTANCE Diagonal2");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDistancePublisher>());
    rclcpp::shutdown();
    return 0;
}
