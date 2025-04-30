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
    ImagePublisher() 
        : Node("img_pub_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        init_gstreamer_pipeline();
    }

    ~ImagePublisher() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

   private:
    void init_gstreamer_pipeline() {
        gst_init(nullptr, nullptr); //init the GStreamer lib do ont remove

        /*
        nvarguscamerasrc: captures video from CSI camera using nvidia's argus api
        nvvidconv: vonverts video formats with gpu i beleive
        video/x-raw, format=BGRx: specifies the output format after conversion BGRx required by tee i think
        tee name=t, how we split into two paths, appsink for OpenVC and nv3dsink for displayport.
        the queues are the different video streams one to open cv one to display port.
        */
        // std::string pipeline_str =
        //     "nvarguscamerasrc ! nvvidconv ! video/x-raw, format=BGRx ! "
        //     "tee name=t "
        //     "t. ! queue ! videoconvert ! video/x-raw, format=BGR ! appsink name=opencv_sink sync=false "
        //     "t. ! queue ! nv3dsink sync=false";

        #ifdef USE_APPSINK_ONLY
        std::string pipeline_str =
            "nvarguscamerasrc ! nvvidconv ! videoflip method=vertical-flip ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink name=opencv_sink sync=false";
        #else
        std::string pipeline_str =
            "nvarguscamerasrc ! nvvidconv ! videoflip method=vertical-flip ! video/x-raw, format=BGRx ! "
            "tee name=t "
            "t. ! queue ! videoconvert ! video/x-raw, format=BGR ! appsink name=opencv_sink sync=false "
            "t. ! queue ! nv3dsink sync=false";
        #endif

        GError *error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error); //turns the string into an actual GStreamer object
        if (!pipeline_ || error) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
            return;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "opencv_sink"); //gets the opencv pipline
        gst_app_sink_set_emit_signals((GstAppSink *)appsink_, true); //for emiting signals when a new element is available
        gst_app_sink_set_drop((GstAppSink *)appsink_, true); // drops frames is procesing is to long
        gst_app_sink_set_max_buffers((GstAppSink *)appsink_, 1); // only buffers 1 frame or the most recent capture.

        //create callback for when we get a new video frame.
        GstAppSinkCallbacks callbacks = { nullptr, nullptr, on_new_sample_static }; 
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

        //starts the image and video capture
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }


    //this function is static because GStrwamer is a C library I beleive and in on_new_sample() we are accessing class variables
    // so we make this function static and when it is used as a callback we pass the running class into it so tht we can use class 
    // variable inside of on_new_sample() 
    static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data) {
        return static_cast<ImagePublisher *>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink *sink) {
        GstSample *sample = gst_app_sink_pull_sample(sink); //get frame from the pipline
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer *buffer = gst_sample_get_buffer(sample); //actual pixels
        GstCaps *caps = gst_sample_get_caps(sample); //format description
        GstStructure *s = gst_caps_get_structure(caps, 0); //framde data like width height

        //get frame dimensions
        int width, height;
        gst_structure_get_int(s, "width", &width);
        gst_structure_get_int(s, "height", &height);

        //this GStreamer works on the fram ein weird placesthat might not be able to be read by the CPU
        //so this mapping is used to give the CPU for safe access to the frame and its memory address for access.
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        cv::Mat frame(height, width, CV_8UC3, (char *)map.data);
        cv::Mat clone = frame.clone();  // clone so that when GStreamer unmaps the image we still have acopy to work with.

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", clone).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);

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