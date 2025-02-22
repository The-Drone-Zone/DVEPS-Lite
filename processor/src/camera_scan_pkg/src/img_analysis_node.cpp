#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaarithm.hpp>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_scan_pkg/msg/obstacle.hpp"
#include "camera_scan_pkg/msg/obstacle_array.hpp"

class ImageAnalysis : public rclcpp::Node {
   public:
    cv::cuda::GpuMat gpu_frame;
    cv::Mat frame;
    cv::Ptr<cv::cuda::Filter> gaussFilter;
    cv::Ptr<cv::cuda::Filter> morphFilter;
    cv::Ptr<cv::cuda::CannyEdgeDetector> canny;

    ImageAnalysis() : Node("img_analysis_node") {
        // Create Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageAnalysis::analysisCallback, this, std::placeholders::_1));

        // Create Publisher
        publisher_ = this-> create_publisher<camera_scan_pkg::msg::ObstacleArray>(
            "output_obstacles", 10
        );
        
        // Create Canny Edge Detector (That way its not recreated each iteration) 
        canny = cv::cuda::createCannyEdgeDetector(100, 200);
    }

   private:
    void analysisCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Start time
        auto start = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "Received image with size: %zu bytes", msg->data.size());
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        frame = cv_ptr->image;
        if (!frame.empty()) {
            gpu_frame.upload(frame);
        }

        // Initialize filters once (ones that are dependent on gpu_frame existing)
        if (!gpu_frame.empty() && !gaussFilter) {
            // Create Gaussian filter for threshold
            gaussFilter = cv::cuda::createGaussianFilter(gpu_frame.type(), gpu_frame.type(), cv::Size(31,31), 15);
             // No instantiation for structuring element gpu kernel exists, must be uploaded
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            morphFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, gpu_frame.type(), kernel);
        }

        camera_scan_pkg::msg::ObstacleArray out_msg = processFrame();

        RCLCPP_INFO(this->get_logger(), "Publishing %zu analyzed image obstacles", out_msg.obstacles.size());
        publisher_->publish(out_msg);

        // End time
        auto end = std::chrono::high_resolution_clock::now();
        // Compute duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // Print analysis time
        RCLCPP_INFO(this->get_logger(), "Analysis Time: %ld ms", duration);
    }

    // Convert BGR Image to GrayscaleS
    void grayscale() {
        cv::cuda::cvtColor(gpu_frame, gpu_frame, cv::COLOR_BGR2GRAY);
    }

    // Apply Adaptive Threshold
    void threshold() {
        // No direct adaptive threshold for cuda so we combine gaussian filtering with thresholding
        cv::cuda::GpuMat blurred;
        gaussFilter->apply(gpu_frame, blurred);

        // Threshold type could be THRESH_BINARY or THRESH_BINARY_INV
        cv::cuda::threshold(blurred, gpu_frame, 127, 255, cv::THRESH_BINARY_INV);
    }

    // Apply Dilation to increase line size
    void dilation() {
        // Apply dilation filter
        morphFilter->apply(gpu_frame, gpu_frame);
    }

    // Canny edge detection with thresholds at 100 and 200
    void edgeDetection() {
        canny->detect(gpu_frame, gpu_frame);
    }

    // Contour Extraction (no OpenCV CUDA function for doing this on GPU exists)
    // Must be done on CPU
    std::vector<std::vector<cv::Point>> contours() {
        // Initialize variables
        std::vector<std::vector<cv::Point>> contourLines;
        std::vector<cv::Vec4i> hierarchy;

        // Move frame from GPU to CPU
        gpu_frame.download(frame);

        // Find Contours
        cv::findContours(frame, contourLines, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        return contourLines;
    }

    // Bounding Boxes (no OpenCV CUDA function for doing this on GPU exists)
    // Must be done on CPU after contours
    camera_scan_pkg::msg::ObstacleArray boundingBoxes(std::vector<std::vector<cv::Point>> contourLines) {
        camera_scan_pkg::msg::ObstacleArray msg = camera_scan_pkg::msg::ObstacleArray();

        for (const auto& cnt : contourLines) {
            // get min area rectangle
            cv::RotatedRect rect = cv::minAreaRect(cnt);

            // Check if bounding box is large enough
            if (rect.size.width * rect.size.height > 250) {
                // Declare Obstacle variables
                camera_scan_pkg::msg::Obstacle obstacle;

                // Initialize obstacle variables
                obstacle.x = rect.center.x;
                obstacle.y = rect.center.y;
                obstacle.width = rect.size.width;
                obstacle.height = rect.size.height;
                obstacle.angle = rect.angle;
                obstacle.distance = -1.0;

                // Get corner points of the bounding box
                cv::Point2f points[4];
                rect.points(points);

                for (int i = 0; i < 4; ++i) {
                    obstacle.corners[i].x = points[i].x;
                    obstacle.corners[i].y = points[i].y;
                    obstacle.corners[i].z = 0.0;
                }
                msg.obstacles.push_back(obstacle);
            }

        }
        return msg;
    }

    camera_scan_pkg::msg::ObstacleArray processFrame() {
        //grayscale(); //TBD
        threshold();
        dilation();
        edgeDetection();
        return boundingBoxes(contours());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<camera_scan_pkg::msg::ObstacleArray>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageAnalysis>());
    rclcpp::shutdown();
    return 0;
}