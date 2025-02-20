#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

// cv::VideoCapture cap("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12,
// framerate=30/1 ! nvvidconv ! video/x-raw, format=(string)GRAY8 ! tee name=t ! queue ! appsink t. ! queue !
// nvoverlaysink", cv::CAP_GSTREAMER);

class ImagePublisher : public rclcpp::Node {
   public:
    //We might need to change the width and height of the image. When we do, we update the timer_ to match the FPS of the new resolution. 
    //It is possible to control the behavior of the queue more precisely by setting the queue size.
    // maximum time the queue can hold a frame. EX:queue max-size-buffers=100 max-size-time=200000000
    ImagePublisher()
        : Node("img_pub_node"),
          cap("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, "
              "framerate=22/1 ! nvvidconv ! video/x-raw, format=(string)GRAY8 ! tee name=t ! queue ! appsink t. ! "
              "queue ! nvoverlaysink",
              cv::CAP_GSTREAMER) {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera with GStreamer pipeline");
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(22), std::bind(&ImagePublisher::publishImage, this));
    }

    ~ImagePublisher() { cap.release(); }

   private:
    void publishImage() {
        cv::Mat frame;

        if (!cap.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
            return;
        }
        std::cout << "Frame size: " << frame.size() << std::endl;

        cv_bridge::CvImage cv_image;
        cv_image.image = frame;
        cv_image.encoding = "mono8";
        msg_ = cv_image.toImageMsg();
        msg_->header.stamp = this->now();
        publisher_->publish(*msg_);
        RCLCPP_INFO(this->get_logger(), "Image %d published", count_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
    int count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}