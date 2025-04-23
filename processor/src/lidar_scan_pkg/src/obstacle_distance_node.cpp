#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include "common/mavlink.h"

class ObstacleDistancePublisher : public rclcpp::Node {
public:
    ObstacleDistancePublisher()
        : Node("obstacle_distance_publisher")
    {
        serial_port_ = open_serial("/dev/ttyUSB0");
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ObstacleDistancePublisher::send_obstacle_distance, this)
        );
    }

    ~ObstacleDistancePublisher() {
        if (serial_port_ > 0)
            close(serial_port_);
    }

private:
    int serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;

    int open_serial(const std::string& port) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) return -1;

        struct termios options{};
        tcgetattr(fd, &options);
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        tcsetattr(fd, TCSANOW, &options);

        return fd;
    }

    void send_obstacle_distance() {
        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        mavlink_obstacle_distance_t dist{};
        dist.time_usec = this->now().nanoseconds() / 1000;
        dist.sensor_type = MAV_DISTANCE_SENSOR_LASER;
        dist.increment = 10;
        dist.min_distance = 300;
        dist.max_distance = 1000;
        dist.increment_f = NAN;
        dist.angle_offset = 0.0f;
        dist.frame = MAV_FRAME_BODY_FRD;
        dist.distance[0] = 450;
        dist.distance[1] = 460;
        dist.distance[2] = 470;
        dist.distance[3] = 0xFFFF;  // No data
        dist.distance[4] = 0xFFFF;  // No data
        dist.distance[5] = 480;
        dist.distance[6] = 490;
        dist.distance[7] = 500;
        dist.distance[8] = 510;
        dist.distance[9] = 520;

        dist.angle_offset = 0.0f;
        dist.increment = 20;

        mavlink_msg_obstacle_distance_encode(1, 200, &msg, &dist);

        int len = mavlink_msg_to_send_buffer(buffer, &msg);
        write(serial_port_, buffer, len);

        RCLCPP_INFO(this->get_logger(), "Sent OBSTACLE_DISTANCE");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDistancePublisher>());
    rclcpp::shutdown();
    return 0;
}
