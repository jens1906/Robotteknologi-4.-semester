#include "initialize.hpp"

Initialize::Initialize(rclcpp::Node::SharedPtr node) : node_(node) {  // Properly initialize node_
    ros_vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    ros_offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
}

void Initialize::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;  // Typically 1 for PX4
    msg.target_component = 1;  // Typically 1 for PX4
    msg.source_system = 1;  // Set your system ID
    msg.source_component = 1;  // Set your component ID
    msg.from_external = true;  // Indicate this is from an external source

    ros_vehicle_command_pub_->publish(msg);
}

void Initialize::disarm(bool kill) {
    float param2 = kill ? 21196.0 : 0.0;  // 21196.0 is the kill switch value
    publishVehicleCommand(400, 0.0, param2);  // Command 400 is for disarming
}

void Initialize::arm() {
    publishVehicleCommand(400, 1.0);  // Command 400 is for arming
}

void Initialize::enable_offboard_mode() {
    if (!node_) {
        RCLCPP_ERROR(rclcpp::get_logger("offboard_control_node"), "Node is not initialized!");
        return;
    }

    std::atomic<bool> is_offboard_enabled(false);

    // Create a subscription to monitor VehicleControlMode messages with BEST_EFFORT QoS
    rclcpp::QoS qos(10);  // Depth of 10
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);  // Set reliability to BEST_EFFORT

    auto vehicle_control_mode_sub = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos,
        [&is_offboard_enabled, this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
            if (!msg) {
                RCLCPP_ERROR(node_->get_logger(), "Received null message in VehicleControlMode subscription.");
                return;
            }

            RCLCPP_INFO(node_->get_logger(),
                        "Vehicle Control Mode: Armed=%d, Offboard=%d, Manual=%d",
                        msg->flag_armed,
                        msg->flag_control_offboard_enabled,
                        msg->flag_control_manual_enabled);

            // Update the shared variable
            is_offboard_enabled.store(msg->flag_control_offboard_enabled);

            // If offboard mode is not enabled, send the command
            if (!msg->flag_control_offboard_enabled) {
                publishVehicleCommand(176, 1.0, 6.0);  // Command 176 is for offboard mode
            }
        });

    // Wait until offboard mode is enabled
    while (!is_offboard_enabled.load() && rclcpp::ok()) {
        rclcpp::spin_some(node_);  // Process callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Avoid busy-waiting
    }

    RCLCPP_INFO(node_->get_logger(), "Offboard mode enabled!");
}

void Initialize::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};  // Declare and initialize the message

    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.position = false;           // Disable position control
    msg.velocity = false;           // Disable velocity control
    msg.acceleration = false;       // Disable acceleration control
    msg.attitude = true;            // Enable attitude control
    msg.body_rate = false;          // Disable body rate control
    msg.thrust_and_torque = false;  // Disable thrust control
    msg.direct_actuator = false;    // Disable direct actuator control

    ros_offboard_control_mode_pub_->publish(msg);
}

void Initialize::turn_on_drone() {
    publishOffboardControlMode();
    arm();
}


