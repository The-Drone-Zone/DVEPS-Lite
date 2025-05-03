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
    //RCLCPP_INFO(commander_->get_logger(), "Local Position - X: %f, Y: %f, Z: %f", msg->x, msg->y, msg->z);
}

void PositionControl::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    current_odometry_ = *msg;

    float q0 = msg->q[0];  
    float q1 = msg->q[1];  
    float q2 = msg->q[2];  
    float q3 = msg->q[3];  

    float psi = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
    current_heading_ = psi;

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
    RCLCPP_INFO(commander_->get_logger(), "LOCAL OFFSET: %f", local_offset_);
}

px4_msgs::msg::TrajectorySetpoint PositionControl::turnByAngle(float angle_degrees, bool commanded)
{
    std::array<float, 3> current_pos = getLocalPosition();
    float target_yaw;

    if(commanded){
        float angle_radians = angle_degrees * (M_PI / 180.0);
        float current_yaw = getCurrentHeading();
        RCLCPP_INFO(commander_->get_logger(), "CURRENT YAW: %f, ANGLE_DEGREES %f", current_yaw * (180.0 / M_PI), angle_degrees);

        target_yaw = current_yaw + angle_radians;
        place_holder_yaw = target_yaw;
        current_pos[2] -= 0.35; //avoid decrease in height from turning
    }
    else{
        target_yaw = place_holder_yaw;
    }

    if (target_yaw > M_PI) {
        target_yaw -= 2 * M_PI;
    } else if (target_yaw < -M_PI) {
        target_yaw += 2 * M_PI;
    }

    px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_msg;
    trajectory_setpoint_msg.position[0] = current_pos[0];
    trajectory_setpoint_msg.position[1] = current_pos[1];
    trajectory_setpoint_msg.position[2] = current_pos[2];
    trajectory_setpoint_msg.yaw = target_yaw;
    trajectory_setpoint_msg.yawspeed = 0.0;
    RCLCPP_INFO(commander_->get_logger(), "TURN TURN TURN YAW: %f, ELIVATION %f", target_yaw * (180.0 / M_PI), trajectory_setpoint_msg.position[2]);


    return trajectory_setpoint_msg;
}

px4_msgs::msg::TrajectorySetpoint PositionControl::moveForwardByMeters(float distance_meters){

    std::array<float, 3> current_pos = getLocalPosition();
    float heading = getCurrentHeading();  // This is in radians

    float dx = distance_meters * std::cos(heading);
    float dy = distance_meters * std::sin(heading);

    auto trajectory_setpoint_msg = TrajectorySetpoint();

    trajectory_setpoint_msg.position[0] = current_pos[0] + dx;
    trajectory_setpoint_msg.position[1] = current_pos[1] + dy;
    trajectory_setpoint_msg.position[2] = current_pos[2];
    trajectory_setpoint_msg.yaw = heading;
    trajectory_setpoint_msg.yawspeed = 0.0; //maybe dont need this try deleting it if it does not work at first

    RCLCPP_INFO(commander_->get_logger(), "Move Forward: target x: %f, y: %f, current heading: %f deg",
                trajectory_setpoint_msg.position[0], trajectory_setpoint_msg.position[1], heading * (180.0 / M_PI));

    return trajectory_setpoint_msg;
}

bool PositionControl::checkDist(std::array<float, 2> start_pos){

    std::array<float, 3> current_pos = getLocalPosition();
    float dx = current_pos[0] - start_pos[0];
    float dy = current_pos[1] - start_pos[1];

    float dist = std::sqrt(dx * dx + dy * dy);
    return dist > 4.5;
}

float PositionControl::getCurrentHeading()
{
    return current_heading_;
}

bool PositionControl::checkAnalysisHeight()  {
    std::array<float, 3> current_pos = getLocalPosition();

    return fabs(current_pos[2]) > 1;
}

std::array<float, 3> PositionControl::getLocalPosition()
{
    std::array<float, 3> pos = {};
    pos[0] = current_local_position_.x;
    pos[1] = current_local_position_.y;
    pos[2] = current_local_position_.z;
    return pos;
}