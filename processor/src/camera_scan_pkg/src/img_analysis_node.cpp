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
#include <opencv2/cudaoptflow.hpp>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_scan_pkg/msg/obstacle.hpp"
#include "camera_scan_pkg/msg/obstacle_array.hpp"

class ImageAnalysis : public rclcpp::Node {
   public:
    class TrackedObstacle {
        public:
            std::vector<cv::Point2f> keypoints;
            cv::cuda::GpuMat gpu_pointsMat;
            int trackCount;

            TrackedObstacle(std::vector<cv::Point2f> kp) {
                copy(kp.begin(), kp.end(), back_inserter(keypoints));
                // Convert Point2f to GpuMat for optical flow
                cv::Mat pointsMat = cv::Mat(1, kp.size(), CV_32FC2, kp.data());
                gpu_pointsMat.upload(pointsMat);
                trackCount = 0;
            }
    };

    std::vector<TrackedObstacle> tracked;
    // Number of frames tracked before we stop drone
    const int MAX_TRACKED = 5;

    cv::cuda::GpuMat gpu_prevFrame;
    cv::cuda::GpuMat gpu_frame;
    // For drawing (For Testing Only)
    cv::Mat origFrame;
    cv::Mat frame;
    cv::Ptr<cv::cuda::Filter> gaussFilter;
    cv::Ptr<cv::cuda::Filter> morphFilter;
    cv::Ptr<cv::cuda::CannyEdgeDetector> canny;
    cv::Ptr<cv::cuda::FastFeatureDetector> fast;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> opticalFlow;

    long int time_sum = 0;
    long int counter = 0;

    ImageAnalysis() : Node("img_analysis_node") {
        // Create Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageAnalysis::analysisCallback, this, std::placeholders::_1));

        // Create Obstacle Publisher
        publisher_ = this-> create_publisher<camera_scan_pkg::msg::ObstacleArray>(
            "output_obstacles", 10
        );

        // Create Image Publisher (For Testing Only)
        image_publisher_ = this-> create_publisher<sensor_msgs::msg::Image>(
            "analyzed_image", 10
        );
        
        // Create Canny Edge Detector (That way its not recreated each iteration) 
        canny = cv::cuda::createCannyEdgeDetector(100, 200);
        // Create Fast Feature Detector (That way its not recreated each iteration) 
        fast = cv::cuda::FastFeatureDetector::create();
        // Create Optical Flow Calculator (That way its not recreated each iteration) 
        opticalFlow = cv::cuda::SparsePyrLKOpticalFlow::create();
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<camera_scan_pkg::msg::ObstacleArray>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; // For Testing Only

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

        if(!gpu_frame.empty()) {
            camera_scan_pkg::msg::ObstacleArray out_msg = processFrame();

            RCLCPP_INFO(this->get_logger(), "Publishing %zu analyzed image obstacles", out_msg.obstacles.size());
            publisher_->publish(out_msg);

            // Convert and Publish Image (For Testing Only)
            sensor_msgs::msg::Image::SharedPtr image_msg;
            cv_bridge::CvImage cv_image;
            cv_image.image = origFrame; // frame or origFrame
            cv_image.encoding = "mono8";
            image_msg = cv_image.toImageMsg();
            image_msg->header.stamp = this->now();
            image_publisher_->publish(*image_msg);
        }

        // End time
        auto end = std::chrono::high_resolution_clock::now();
        // Compute duration
        std::chrono::milliseconds duration_ms = std::chrono::duration_cast<std::chrono::milliseconds >(end - start);
        // Print analysis time
        RCLCPP_INFO(this->get_logger(), "Analysis Time: %ld ms", duration_ms.count());
        time_sum += duration_ms.count();
        counter += 1;
        RCLCPP_INFO(this->get_logger(), "Current Average Analysis FPS: %ld fps", 1000 / (time_sum / counter));
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

    void featureDetection(const camera_scan_pkg::msg::ObstacleArray& msg) {
        std::vector<cv::KeyPoint> keypoints;
        // Conduct fast feature detection on gpu frame
        fast->detect(gpu_frame, keypoints);
        // Loop through list of obstacles
        for (const camera_scan_pkg::msg::Obstacle& obstacle : msg.obstacles) {
            // KeyPoint must be converted to Point2f for Optical Flow
            std::vector<cv::Point2f> new_kp;
            new_kp.reserve(keypoints.size());  // Reserve memory to avoid repeated allocations

            float x_min = obstacle.x;
            float x_max = obstacle.x + obstacle.width;
            float y_min = obstacle.y;
            float y_max = obstacle.y + obstacle.height;

            // Filter keypoints that are within obstacle
            std::for_each(keypoints.begin(), keypoints.end(), [&](const cv::KeyPoint& kp) {
                if ((x_min <= kp.pt.x && kp.pt.x <= x_max) && 
                    (y_min <= kp.pt.y && kp.pt.y <= y_max)) {
                    new_kp.push_back(kp.pt);
                }
            });
            
            // Add keypoints to track list for future optical flow use
            tracked.push_back(TrackedObstacle(new_kp));
        }
    }

    camera_scan_pkg::msg::ObstacleArray TrackOpticalFlow(camera_scan_pkg::msg::ObstacleArray& msg) {
        // Check if a previous frame exists
        if (!gpu_prevFrame.empty()) {
            tracked.erase(
                std::remove_if(tracked.begin(), tracked.end(),
                    [&](TrackedObstacle& obstacle) {
                        // If this is the first detection, just increment count and continue
                        if (obstacle.trackCount == 0 || obstacle.gpu_pointsMat.empty()) {
                            obstacle.trackCount++;
                            return false; // Keep this obstacle
                        }

                        // Conduct Optical Flow
                        cv::cuda::GpuMat gpu_nextPts, gpu_status;
                        opticalFlow->calc(gpu_prevFrame, gpu_frame, obstacle.gpu_pointsMat, gpu_nextPts, gpu_status);

                        // Preallocate memory for downloads
                        size_t keypoints_size = obstacle.keypoints.size();
                        std::vector<cv::Point2f> nextPts;
                        nextPts.reserve(keypoints_size);
                        std::vector<unsigned char> status;
                        status.reserve(keypoints_size);

                        gpu_nextPts.download(cv::Mat(1, keypoints_size, CV_32FC2, nextPts.data()));
                        gpu_status.download(cv::Mat(1, keypoints_size, CV_8U, status.data()));

                        // Filter matched keypoints in place
                        size_t i = 0;
                        obstacle.keypoints.erase(
                            std::remove_if(obstacle.keypoints.begin(), obstacle.keypoints.end(),
                                [&](const cv::Point2f& pt) { return status[i++] == 0; }),
                            obstacle.keypoints.end());

                        // If no keypoints remain, remove this obstacle
                        if (obstacle.keypoints.empty()) return true;

                        // Update tracked obstacle variables
                        obstacle.trackCount++;
                        if (obstacle.trackCount >= MAX_TRACKED) {
                            msg.tracked_obstacle = true;
                            return true; // Remove obstacle from tracking
                        }

                        // Update GPU points only if changes occurred
                        cv::Mat pointsMat(1, obstacle.keypoints.size(), CV_32FC2, obstacle.keypoints.data());
                        obstacle.gpu_pointsMat.upload(pointsMat);

                        return false; // Keep obstacle
                    }),
                tracked.end());
        }

        // Store current frame for the next optical flow calculation
        gpu_frame.copyTo(gpu_prevFrame);
        return msg;
    }


    void draw(const camera_scan_pkg::msg::ObstacleArray& msg) {
        // Draw boxes
        for (const camera_scan_pkg::msg::Obstacle& obstacle : msg.obstacles) {
            for (size_t i = 0; i < 4; ++i) {
                cv::line(origFrame, 
                    cv::Point(obstacle.corners[i].x, obstacle.corners[i].y), 
                    cv::Point(obstacle.corners[(i + 1) % 4].x, obstacle.corners[(i + 1) % 4].y), 
                    cv::Scalar(0, 255, 0), // color
                    2); // thickness
            }
        }
        
        // Draw points
        for (const TrackedObstacle& obstacle : tracked) {
            for (const cv::Point2f& point : obstacle.keypoints) {
                cv::circle(origFrame, point, 3, cv::Scalar(0, 0, 255), -1); // Filled circle
            }
        }
    }

    camera_scan_pkg::msg::ObstacleArray processFrame() {
        camera_scan_pkg::msg::ObstacleArray msg;
        msg.tracked_obstacle = false;
        frame.copyTo(origFrame);
        //grayscale(); //TBD
        threshold();
        dilation();
        edgeDetection();
        msg = boundingBoxes(contours());
        featureDetection(msg);
        msg = TrackOpticalFlow(msg);
        draw(msg); // For Testing Only
        return msg;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageAnalysis>());
    rclcpp::shutdown();
    return 0;
}