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

void Controller::publishVehicleAttitudeSetpoint(float x_error, float y_error, float yaw, float thrust) {
    auto roll_pitch = xyToRollPitch(x_error, y_error);
    float roll = roll_pitch[0];
    float pitch = roll_pitch[1];
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.q_d = rpyToQuaternion(roll, pitch, yaw);  // Quaternion [w, x, y, z]
    msg.thrust_body = {0.0f, 0.0f, -thrust};  // Thrust in body frame [x, y, z]

    ros_attitude_setpoint_pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleAttitudeSetpoint: thrust=%.2f", thrust);
}

std::array<float, 2> Controller::xyToRollPitch(float x_error, float y_error) {
    // PD Controller Parameters for x/y control
    float Kp_xy = 7.344;  // Proportional gain for x/y
    float Kd_xy = 6.778;  // Derivative gain for x/y

    // Static variables to store previous errors and timestamp
    static float prev_x_error = 0.0f;
    static float prev_y_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now();

    // Current timestamp
    rclcpp::Time current_time = rclcpp::Clock().now();

    // Calculate dynamic dt (time difference in seconds)
    float dt = (current_time - prev_time).seconds();
    if (dt <= 0.0f) {
        dt = 0.01f;  // Fallback to default value if dt is invalid
    }

    // Update the previous timestamp
    prev_time = current_time;

    // Derivative terms (discrete-time implementation)
    float x_derivative = (x_error - prev_x_error) / dt;
    float y_derivative = (y_error - prev_y_error) / dt;

    // PD Control Outputs
    float roll_desired = Kp_xy * y_error + Kd_xy * y_derivative;  // Roll to correct y position
    float pitch_desired = Kp_xy * x_error + Kd_xy * x_derivative; // Pitch to correct x position

    // Update previous errors
    prev_x_error = x_error;
    prev_y_error = y_error;

    return {roll_desired, pitch_desired};  // Return roll and pitch as an array
}

std::array<float, 4> Controller::rpyToQuaternion(float roll, float pitch, float yaw) {
    // Convert controlled roll, pitch, yaw to quaternion
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