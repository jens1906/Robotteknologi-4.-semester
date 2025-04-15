#include "controller.hpp"

Controller::Controller(rclcpp::Node::SharedPtr node) : vicon_position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {
    node_ = node;  // Assign the node to the member variable
}

// Initialize ROS topics
void Controller::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node; 
    rclcpp::QoS qos(10); 
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
    float thrust = zToThrust(xyz_error[2]);  // Compute thrust
    yaw = 0.0f;  // Set yaw to 0 for simplicity

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.q_d = rpyToQuaternion(roll_pitch[0], roll_pitch[1], yaw);  // Quaternion [w, x, y, z]
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust};  // Thrust in body frame [x, y, z]

    ros_attitude_setpoint_pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleAttitudeSetpoint: thrust=%.2f", thrust);
}

std::array<float, 4> Controller::rpyToQuaternion(float roll, float pitch, float yaw) {
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
    return {w, x, y, z};
}

std::array<float, 2> Controller::xyToRollPitch(float x_error, float y_error) {
    float Kp_xy = 7.344;
    float Kd_xy = 6.778;

    static float prev_x_error = 0.0f;
    static float prev_y_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now();

    rclcpp::Time current_time = rclcpp::Clock().now();
    float dt = (current_time - prev_time).seconds();
    if (dt <= 0.0f) {
        dt = 0.01f;
    }
    prev_time = current_time;

    float x_derivative = (x_error - prev_x_error) / dt;
    float y_derivative = (y_error - prev_y_error) / dt;

    // PD Control Outputs
    float roll_desired = Kp_xy * y_error + Kd_xy * y_derivative * (3.14159/180);  // Roll to correct y position
    float pitch_desired = -(Kp_xy * x_error + Kd_xy * x_derivative) * (3.14159/180); // Pitch to correct x position

    roll_desired = std::clamp(roll_desired, -0.5f, 0.5f);
    pitch_desired = std::clamp(pitch_desired, -0.5f, 0.5f);

    prev_x_error = x_error;
    prev_y_error = y_error;

    return {roll_desired, pitch_desired};
}

float Controller::zToThrust(float z_error) {
    float Kp_z = 5.534;
    float Kd_z = 5.108;

    static float prev_z_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now();

    rclcpp::Time current_time = rclcpp::Clock().now();
    float dt = (current_time - prev_time).seconds();
    if (dt <= 0.0f) {
        RCLCPP_WARN(rclcpp::get_logger("offboard_control_node"), "Invalid dt detected, using fallback value.");
        dt = 0.01f;
    }
    prev_time = current_time;

    float z_derivative = (z_error - prev_z_error) / dt;
    float thrust = (Kp_z * z_error + Kd_z * z_derivative) / 100.0f;
    thrust = std::clamp(thrust, 0.0f, 0.8f);

    prev_z_error = z_error;

    return thrust;
}

void Controller::goalPosition(const std::array<float, 3>& goal_position) {
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "goalPosition called with goal: x=%.2f, y=%.2f, z=%.2f",
                goal_position[0], goal_position[1], goal_position[2]);

    std::atomic<bool> vicon_data_received(false);

    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    auto ros_vicon_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Vicon", qos,
        [this, &vicon_data_received](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
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

                vicon_data_received.store(true);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("offboard_control_node"),
                            "Received Vicon data with insufficient elements.");
            }
        });

    rclcpp::Time start_time = rclcpp::Clock().now();
    while (!vicon_data_received.load() && (rclcpp::Clock().now() - start_time).seconds() < 1.0) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!vicon_data_received.load()) {
        RCLCPP_WARN(rclcpp::get_logger("offboard_control_node"), "Timeout waiting for Vicon data.");
        return;
    }

    float x_error = goal_position[0] - vicon_position_[0];
    float y_error = goal_position[1] - vicon_position_[1];
    float z_error = goal_position[2] - vicon_position_[2];

    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Errors computed: x=%.2f, y=%.2f, z=%.2f",
                x_error, y_error, z_error);

    publishVehicleAttitudeSetpoint({x_error, y_error, z_error}, 0.0f);
}