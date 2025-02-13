#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sl_lidar.h>
#include <thread>
#include <atomic>

enum {
    LIDAR_A_SERIES_MINUM_MAJOR_ID      = 0,
    LIDAR_S_SERIES_MINUM_MAJOR_ID       = 5,
    LIDAR_T_SERIES_MINUM_MAJOR_ID       = 8,
};

using namespace sl;

class RPLidarNode : public rclcpp::Node {
public:
    RPLidarNode() : Node("lidar_pub_node"), keep_running_(true) {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate", 1000000); //might need to change but for usb this works
        this->declare_parameter<std::string>("frame_id", "laser_frame");

        // Get parameter values
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int serial_baudrate = this->get_parameter("serial_baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Create publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);

        // Initialize Lidar Driver
        drv_ = *createLidarDriver();
        
        if (!drv_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create RPLIDAR driver.");
            return;
        }

        IChannel* channel = *createSerialPortChannel(serial_port, serial_baudrate);
        if (SL_IS_FAIL(drv_->connect(channel))) {
            delete drv_;
            RCLCPP_ERROR(this->get_logger(), "Error, cannot connect to RPLIDAR on %s.", serial_port.c_str());
            throw std::runtime_error("RPLIDAR initialization failed.");
        }

        if(!getRPLIDARDeviceInfo(drv_)){
            delete drv_;
            RCLCPP_ERROR(this->get_logger(), "Error, cannot get RPLIDAR info %s.", serial_port.c_str());
            throw std::runtime_error("RPLIDAR initialization failed.");
        }

        if (!checkRPLIDARHealth(drv_)) {
            delete drv_;
            RCLCPP_ERROR(this->get_logger(), "RPLIDAR health check failed. Exiting.");
            throw std::runtime_error("RPLIDAR initialization failed.");
        }

        // Create start/stop services
        stop_motor_service_ = this->create_service<std_srvs::srv::Empty>(
            "stop_motor", std::bind(&RPLidarNode::stop_motor_callback, this, std::placeholders::_1, std::placeholders::_2));
        start_motor_service_ = this->create_service<std_srvs::srv::Empty>(
            "start_motor", std::bind(&RPLidarNode::start_motor_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Start the background thread for continuous scanning
        scan_thread_ = std::thread(&RPLidarNode::scan_loop, this);
    }

    ~RPLidarNode() {
        keep_running_ = false;
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
        if (drv_) {
            drv_->setMotorSpeed(0);
            drv_->disconnect();
        }
        delete drv_;
    }

private:
    std::atomic<bool> keep_running_;
    std::thread scan_thread_;
    ILidarDriver * drv_ = NULL;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service_;

    bool getRPLIDARDeviceInfo(ILidarDriver * drv) {
        sl_result     op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        char sn_str[35] = {0}; 
        for (int pos = 0; pos < 16 ;++pos) {
            sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
        }
        char mode_str[16] = {0};
        if((devinfo.model>>4) <= LIDAR_S_SERIES_MINUM_MAJOR_ID){
            sprintf(mode_str,"A%dM%d",(devinfo.model>>4),(devinfo.model&0xf));

        }else if((devinfo.model>>4) <= LIDAR_T_SERIES_MINUM_MAJOR_ID){
            sprintf(mode_str,"S%dM%d",(devinfo.model>>4)-LIDAR_S_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));
        }else{
            sprintf(mode_str,"T%dM%d",(devinfo.model>>4)-LIDAR_T_SERIES_MINUM_MAJOR_ID,(devinfo.model&0xf));

        }
        RCLCPP_INFO(this->get_logger(), "RPLIDAR MODE:%s",mode_str);
        RCLCPP_INFO(this->get_logger(), "RPLIDAR S/N: %s",sn_str);
        RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }


    void scan_loop() {
        while (keep_running_) {
            if (!drv_ || !drv_->isConnected()) {
                RCLCPP_WARN(this->get_logger(), "Lidar is not connected. Retrying...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = sizeof(nodes) / sizeof(nodes[0]);

            rclcpp::Time start_scan_time = this->get_clock()->now();
            sl_result op_result = drv_->grabScanDataHq(nodes, count);
            rclcpp::Time end_scan_time = this->get_clock()->now();
            double scan_duration = (end_scan_time - start_scan_time).seconds();

            if (op_result == SL_RESULT_OK) {
                drv_->ascendScanData(nodes, count);
                RCLCPP_INFO(this->get_logger(), "Scanned Data");
                publish_scan(nodes, count, start_scan_time, scan_duration);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to grab scan data.");
            }
        }
    }

    void publish_scan(sl_lidar_response_measurement_node_hq_t* nodes, size_t node_count, 
                      rclcpp::Time start, double scan_time) {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = start;
        scan_msg.header.frame_id = frame_id_;
        scan_msg.angle_min = 0.0;
        scan_msg.angle_max = 2.0 * M_PI;
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);
        scan_msg.scan_time = scan_time;
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 8.0;

        scan_msg.ranges.resize(node_count);
        scan_msg.intensities.resize(node_count);

        for (size_t i = 0; i < node_count; i++) {
            float distance = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
            scan_msg.ranges[i] = (distance == 0.0) ? std::numeric_limits<float>::infinity() : distance;
            scan_msg.intensities[i] = (float)(nodes[i].quality >> 2);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing scan with %zu points", node_count);
        scan_pub_->publish(scan_msg);
    }

    bool checkRPLIDARHealth(ILidarDriver* drv) {
        sl_lidar_response_device_health_t healthinfo;
        sl_result op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) {
            if (healthinfo.status == SL_LIDAR_STATUS_OK) {
                RCLCPP_INFO(this->get_logger(), "RPLIDAR health status: OK.");
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "RPLIDAR health warning or error.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot retrieve RPLIDAR health.");
            return false;
        }
    }

    void stop_motor_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        if (drv_) {
            RCLCPP_INFO(this->get_logger(), "Stopping motor...");
            drv_->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            drv_->setMotorSpeed(0);
            RCLCPP_INFO(this->get_logger(), "Motor stopped.");
        }
    }

    void start_motor_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                              std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        if (drv_ && drv_->isConnected()) {
            RCLCPP_INFO(this->get_logger(), "Starting motor.");
            drv_->setMotorSpeed();
            drv_->startScan(0, 1);
        } else {
            RCLCPP_WARN(this->get_logger(), "Lidar is not connected.");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPLidarNode>());
    rclcpp::shutdown();
    return 0;
}
