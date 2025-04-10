#include "controller.hpp"

Controller::Controller(rclcpp::Node::SharedPtr node) : vicon_position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {
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

void Controller::publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw) {
    auto roll_pitch = xyToRollPitch(xyz_error[0], xyz_error[1]);
    float thrust = zToThrust(xyz_error[2]);  // Assuming z_error is the desired thrust
    yaw = 0.0f;  // Set yaw to 0 for simplicity

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.q_d = rpyToQuaternion(roll_pitch[0], roll_pitch[1], yaw);  // Quaternion [w, x, y, z]
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust};  // Thrust in body frame [x, y, z]

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

float Controller::zToThrust(float z_error) {
    // PD controller parameters
    float Kp_z = 5.534;  // Proportional gain for z control
    float Kd_z = 5.108;  // Derivative gain for z control

    // Static variables to store previous errors and timestamp  
    static float prev_z_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now(); // Initialize with current time

    // Current timestamp
    rclcpp::Time current_time = rclcpp::Clock().now();

    // Calculate dynamic dt (time difference in seconds)
    float dt = (current_time - prev_time).seconds();
    if (dt <= 0.0f) {
        RCLCPP_WARN(rclcpp::get_logger("offboard_control_node"), "Invalid dt detected, using fallback value.");
        dt = 0.01f;  // Fallback to default value if dt is invalid
    }

    // Update the previous timestamp
    prev_time = current_time;

    // Derivative term (discrete-time implementation)
    float z_derivative = (z_error - prev_z_error) / dt;

    // PD Control Output 
    float thrust = (Kp_z * z_error + Kd_z * z_derivative) / 100.0f;  // Thrust to correct z position
    thrust = std::clamp(thrust, 0.0f, 1.0f);  // Clamp thrust to [0, 1] range

    // Update previous error
    prev_z_error = z_error;

    return thrust;  // Return thrust as a single value
}

std::array<float, 3>  Controller::goalPosition(const std::array<float, 3>& goal_position) {
    rclcpp::QoS qos(10);  // Depth of 10
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  // Use Reliable reliability

    auto ros_vicon_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Vicon", qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 6) {
                vicon_position_[0] = msg->data[2];
                vicon_position_[1] = msg->data[3];
                vicon_position_[2] = msg->data[4];
                vicon_position_[3] = msg->data[5];
                vicon_position_[4] = msg->data[6];
                vicon_position_[5] = msg->data[7];

                RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"),
                            "Vicon position updated: x=%.2f, y=%.2f, z=%.2f, r=%.2f, p=%.2f, y=%.2f",
                            vicon_position_[0], vicon_position_[1], vicon_position_[2],
                            vicon_position_[3], vicon_position_[4], vicon_position_[5]);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("offboard_control_node"),
                            "Received Vicon data with insufficient elements.");
            }
        });

    //X and Y error
    float x_error = goal_position[0] - vicon_position_[0];
    float y_error = goal_position[1] - vicon_position_[1];
    //Z error
    float z_error = goal_position[2] - vicon_position_[2];

    // Allow the node to spin and process callbacks
    //rclcpp::spin(node_);
    return {x_error, y_error, z_error};  // Return the errors as an array
}


