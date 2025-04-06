#include <cv_bridge/cv_bridge.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

        
class ImagePublisher : public rclcpp::Node {
   public:
    long int time_sum = 0;
    long int counter = 0;
    //We might need to change the width and height of the image. When we do, we update the timer_ to match the FPS of the new resolution. 
    //It is possible to control the behavior of the queue more precisely by setting the queue size.
    // maximum time the queue can hold a frame. EX:queue max-size-buffers=100 max-size-time=200000000
    ImagePublisher() 
        : Node("img_pub_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        init_gstreamer_pipeline();
    }

    ~ImagePublisher() { 
        //cap.release();
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

   private:
    void init_gstreamer_pipeline() {
        gst_init(nullptr, nullptr);

        std::string pipeline_str =
            "nvarguscamerasrc ! nvvidconv ! video/x-raw, format=BGRx ! "
            "tee name=t "
            "t. ! queue ! videoconvert ! video/x-raw, format=BGR ! appsink name=mysink sync=false "
            "t. ! queue ! nv3dsink sync=false";

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_ || error) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
            return;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
        gst_app_sink_set_emit_signals((GstAppSink *)appsink_, true);
        gst_app_sink_set_drop((GstAppSink *)appsink_, true);
        gst_app_sink_set_max_buffers((GstAppSink *)appsink_, 1);

        GstAppSinkCallbacks callbacks = { nullptr, nullptr, on_new_sample_static };
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data) {
        return static_cast<ImagePublisher *>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink *sink) {
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);
        GstStructure *s = gst_caps_get_structure(caps, 0);

        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        cv::Mat frame(height, width, CV_8UC3, (char *)map.data);
        cv::Mat clone = frame.clone();  // clone to avoid using mapped memory

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", clone).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
//bgr8
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    GstElement *pipeline_ = nullptr;
    GstElement *appsink_ = nullptr;
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