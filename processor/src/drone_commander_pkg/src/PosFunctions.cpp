#include "rclcpp/rclcpp.hpp"
#include "drone_commander_pkg/PosFunctions.hpp"
#include "drone_commander_pkg/DroneCommander.hpp"


PositionControl::PositionControl(std::shared_ptr<DroneCommander> commander)
    : commander_(commander) 
{
    initSubscribers();
    initFrame();
}

void PositionControl::initSubscribers()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    position_subscriber_ = commander_->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&PositionControl::positionCallback, this, std::placeholders::_1));
    odometry_subscriber_ = commander_->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&PositionControl::odometryCallback, this, std::placeholders::_1));
    global_position_subscriber_ = commander_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos,std::bind(&PositionControl::globalPositionCallback, this, std::placeholders::_1));
    
}

void PositionControl::positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    current_local_position_ = *msg;
    RCLCPP_INFO(commander_->get_logger(), "Local Position - X: %f, Y: %f, Z: %f", msg->x, msg->y, msg->z);
}

void PositionControl::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    current_odometry_ = *msg;

    float q0 = msg->q[0];  
    float q1 = msg->q[1];  
    float q2 = msg->q[2];  
    float q3 = msg->q[3];  

    float psi = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    //current_heading_ = psi * (180.0 / M_PI) - local_offset_;

   // RCLCPP_INFO(commander_->get_logger(), "Odometry - X: %f, Y: %f, Z: %f", current_odometry_.position[0], current_odometry_.position[1], current_odometry_.position[2]);
}

void PositionControl::globalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    current_global_position_ = *msg;
    //RCLCPP_INFO(commander_->get_logger(), "Global Position - Lat: %f, Lon: %f, Alt: %f", msg->lat, msg->lon, msg->alt);
}

void PositionControl::initFrame()
{
    local_offset_pose_.x = 0.0;
    local_offset_pose_.y = 0.0;
    local_offset_pose_.z = 0.0;
    local_offset_ = 0.0;

    int iterations = 30;

    for (int i = 1; i <= iterations; i++) {
        if (current_odometry_.q[0] != 0.0 && current_odometry_.q[1] != 0.0 && 
            current_odometry_.q[2] != 0.0 && current_odometry_.q[3] != 0.0) {
            
            float q0 = current_odometry_.q[0];  
            float q1 = current_odometry_.q[1];  
            float q2 = current_odometry_.q[2];  
            float q3 = current_odometry_.q[3];  

            float psi = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
            local_offset_ += (psi * (180.0 / M_PI));
        }

        local_offset_pose_.x += current_odometry_.position[0];
        local_offset_pose_.y += current_odometry_.position[1];
        local_offset_pose_.z += current_odometry_.position[2];
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    local_offset_pose_.x /= iterations;
    local_offset_pose_.y /= iterations;
    local_offset_pose_.z /= iterations;
    local_offset_ /= iterations;
}

px4_msgs::msg::TrajectorySetpoint PositionControl::turnByAngle(float angle_degrees)
{
    float angle_radians = angle_degrees * (M_PI / 180.0);
    float current_yaw = getCurrentHeading();
    float target_yaw = current_yaw + angle_radians;

    if (target_yaw > M_PI) {
        target_yaw -= 2 * M_PI;
    } else if (target_yaw < -M_PI) {
        target_yaw += 2 * M_PI;
    }

    px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_msg;
    trajectory_setpoint_msg.position[0] = current_local_position_.x;
    trajectory_setpoint_msg.position[1] = current_local_position_.y;
    trajectory_setpoint_msg.position[2] = current_local_position_.z;
    trajectory_setpoint_msg.yaw = target_yaw;
    RCLCPP_INFO(commander_->get_logger(), "TURN TURN TURN MESSAGE: %f, Y: %f, Z: %f", trajectory_setpoint_msg.position[0], trajectory_setpoint_msg.position[1], trajectory_setpoint_msg.position[2]);

    return trajectory_setpoint_msg;
}

float PositionControl::getCurrentHeading()
{
    return current_heading_;
}

px4_msgs::msg::VehicleLocalPosition PositionControl::getLocalPosition()
{
    return current_local_position_;
}