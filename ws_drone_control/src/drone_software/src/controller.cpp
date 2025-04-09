#include "controller.hpp"

Controller::Controller(rclcpp::Node::SharedPtr node) {
    node_ = node;  // Assign the node to the member variable
}

void Controller::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;  // Assign the node to the member variable
    rclcpp::QoS qos(10);  // Depth of 10
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    ros_attitude_setpoint_pub_ = node->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
    auto ros_vehicle_attitude_sub_ = node->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos,
        std::bind(&Controller::vehicleAttitudeCallback, this, std::placeholders::_1));
}

void Controller::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleAttitude message");
}

void Controller::publishVehicleAttitudeSetpoint(float roll, float pitch, float yaw, float thrust) {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.q_d = rpyToQuaternion(roll, pitch, yaw);  // Quaternion [w, x, y, z]
    msg.thrust_body = {0.0f, 0.0f, -thrust};  // Thrust in body frame [x, y, z]

    ros_attitude_setpoint_pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleAttitudeSetpoint: thrust=%.2f", thrust);
}

std::array<float, 4> Controller::rpyToQuaternion(float roll, float pitch, float yaw) {
    // Convert roll, pitch, yaw to quaternion
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    float w = cy * cp * cr + sy * sp * sr;
    float x = cy * cp * sr - sy * sp * cr;
    float y = sy * cp * sr + cy * sp * cr;
    float z = sy * cp * cr - cy * sp * sr;

    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f", w, x, y, z);
    
    return {w, x, y, z};  // Return the quaternion as an array
}