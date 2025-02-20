#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

class ImageSubscriber : public rclcpp::Node {
   public:
    ImageSubscriber() : Node("img_sub_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
    }

   private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image with size: %zu bytes", msg->data.size());
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        ///////////////////////////////////////////////////////////////////
        //
        // completly ripped from chat GPT, but it is only a template soo idc
        //
        ///////////////////////////////////////////////////////////////////
        cv::Mat frame = cv_ptr->image;
        // cv::imshow("Image", frame);
        cv::cuda::GpuMat gpu_frame;
        gpu_frame.upload(frame);

        // Create Gaussian filter using CUDA
        cv::Ptr<cv::cuda::Filter> gaussian_filter =
            cv::cuda::createGaussianFilter(gpu_frame.type(), gpu_frame.type(), cv::Size(5, 5), 1.0);

        // Apply the filter
        cv::cuda::GpuMat gpu_blurred;
        gaussian_filter->apply(gpu_frame, gpu_blurred);

        // Download back to CPU (if needed)
        cv::Mat blurred_frame;
        gpu_blurred.download(blurred_frame);

        // Display the processed image
        // cv::namedWindow("Filtered Image", cv::WINDOW_NORMAL);
        // cv::imshow("Filtered Image", blurred_frame);
        ///////////////////////////////////////////////////////////////////
        //
        // completly ripped from chat GPT, but it is only a template soo idc
        //
        ///////////////////////////////////////////////////////////////////

        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}