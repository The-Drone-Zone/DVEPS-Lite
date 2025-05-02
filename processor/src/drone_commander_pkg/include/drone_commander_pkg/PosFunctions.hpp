#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>

class DroneCommander;

class PositionControl {
    public:
        PositionControl(std::shared_ptr<DroneCommander> commander);

        px4_msgs::msg::TrajectorySetpoint turnByAngle(float angle_degrees, bool commanded=false);
        px4_msgs::msg::TrajectorySetpoint moveForwardByMeters(float dist);
        bool checkDist(std::array<float, 2> start_pos);
        bool checkAnalysisHeight();
        float getCurrentHeading();
        std::array<float, 3> getLocalPosition();

    private:
        void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
        void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        void globalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
        void initFrame();
        void initSubscribers();

        std::shared_ptr<DroneCommander> commander_;

        px4_msgs::msg::VehicleOdometry current_odometry_;
        px4_msgs::msg::VehicleLocalPosition current_local_position_;
        px4_msgs::msg::VehicleGlobalPosition current_global_position_;
        geometry_msgs::msg::Point local_offset_pose_;
        float current_heading_;
        float local_offset_;
        float place_holder_yaw;

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_subscriber_;
};

#endif // POSITION_CONTROL_H
