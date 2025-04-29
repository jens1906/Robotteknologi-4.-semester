#include "initialize.hpp"

Initialize::Initialize(rclcpp::Node::SharedPtr node) : node_(node) {
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    actuator_motors_pub_ = node_->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
}

void Initialize::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}

void Initialize::arm() {
    publish_vehicle_command(400, 1.0);  // Command 400 is for arming
}

void Initialize::disarm() {
    publish_vehicle_command(400, 0.0);  // Command 400 is for disarming
}

void Initialize::enable_offboard_mode() {
    auto timer = node_->create_wall_timer(
        std::chrono::milliseconds(100),  // Publish every 100ms
        [this]() {
            publish_vehicle_command(176, 1.0, 6.0);  // Command 176 is for offboard mode
        });

    RCLCPP_INFO(node_->get_logger(), "Offboard mode command is being published continuously.");
    rclcpp::spin_some(node_);  // Process callbacks to start the timer
}

void Initialize::spin_motor(uint8_t motor_index, float throttle) {
    px4_msgs::msg::ActuatorMotors msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds

    if (motor_index > 0 && motor_index <= 8) {
        msg.control[motor_index - 1] = throttle;  // Motor index is 1-based, array is 0-based
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid motor index. Must be between 1 and 8.");
        return;
    }

    actuator_motors_pub_->publish(msg);
}
