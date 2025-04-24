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

        mavlink_obstacle_distance_t dist;
        dist.time_usec = this->now().nanoseconds() / 1000;
        dist.sensor_type = MAV_DISTANCE_SENSOR_LASER;
        dist.increment = 5;
        dist.min_distance = 100;
        dist.max_distance = 50000;
        dist.increment_f = NAN;
        dist.angle_offset = 0.0f;
        dist.frame = MAV_FRAME_BODY_FRD;
        for (int i = 0; i < 72; ++i) {
            dist.distances[i] = 40000;
        }
        dist.distances[0] = 20000;
        dist.distances[1] = 18000;
        dist.distances[2] = 15000;
        dist.distances[3] = 15000;  
        dist.distances[4] = 15000; 
        dist.distances[5] = 16000;
        dist.distances[6] = 17000;
        dist.distances[7] = 19000;
        dist.distances[8] = 20000;
        dist.distances[9] = 25000;

        mavlink_msg_obstacle_distance_pack(
            1, 200, &msg,
            dist.time_usec,
            dist.sensor_type,
            dist.distances,
            dist.increment,
            dist.min_distance,
            dist.max_distance,
            dist.increment_f,
            dist.angle_offset,
            dist.frame
        );

        int len = mavlink_msg_to_send_buffer(buffer, &msg);
        tcflush(serial_port_, TCIOFLUSH);
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
