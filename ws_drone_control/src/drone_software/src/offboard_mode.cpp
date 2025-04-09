#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <chrono>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Global variables for publishers
rclcpp::Publisher<OffboardControlMode>::SharedPtr ros_offboard_control_mode_pub_;
rclcpp::Publisher<VehicleCommand>::SharedPtr ros_vehicle_command_pub_;
rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_;
rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr ros_vehicle_rates_setpoint_pub_;
rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr ros_vehicle_thrust_setpoint_pub_;

std::atomic<bool> is_offboard_enabled(false);

// Callback for VehicleAttitude messages
void VehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleAttitude message");
}

// Callback for VehicleLocalPosition messages
void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleLocalPosition message");
}

// Callback for VehicleStatus messages
void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleStatus message");
}

// Function to publish OffboardControlMode
void publishOffboardControlMode() {
    OffboardControlMode msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.position = false;       // Disable position control
    msg.velocity = false;       // Disable velocity control
    msg.acceleration = false;   // Disable acceleration control
    msg.attitude = true;       // Disable attitude control
    msg.body_rate = false;       // Enable body rate control
    msg.thrust_and_torque = false;  // Enable thrust control
    msg.direct_actuator = false; // Disable direct actuator control

    ros_offboard_control_mode_pub_->publish(msg);
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published OffboardControlMode for body rate control");
}

// Function to publish VehicleCommand
void publishVehicleCommand(uint16_t command, float param1, float param2 = 0.0) {
    VehicleCommand msg{};
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
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleCommand: command=%d, param1=%.1f, param2=%.1f", command, param1, param2);
}

void publishVehicleRatesSetpoint(float roll_rate, float pitch_rate, float yaw_rate, float thrust) {
    VehicleRatesSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.roll = roll_rate;
    msg.pitch = pitch_rate;
    msg.yaw = yaw_rate;
    msg.thrust_body = {0.0, 0.0, -thrust};
    msg.reset_integral = 0;  // Set to true if you want to reset the integral term

    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"),
    //            "Publishing VehicleRatesSetpoint: roll=%.2f, pitch=%.2f, yaw=%.2f, thrust=%.2f",
    //            roll_rate, pitch_rate, yaw_rate, thrust);

    ros_vehicle_rates_setpoint_pub_->publish(msg);
}

void publishVehicleAttitudeSetpoint(float w, float x, float y, float z, float thrust) {
    VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.q_d = {w, x, y, z};  // Quaternion [w, x, y, z]
    msg.thrust_body = {0.0f, 0.0f, -thrust};  // Thrust in body frame [x, y, z]

    ros_attitude_setpoint_pub_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleAttitudeSetpoint: thrust=%.2f", thrust);
}

void publishVehicleThrustSetpoint(float thrust) {
    VehicleThrustSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects timestamp in microseconds
    msg.xyz = {thrust,thrust,thrust};

    ros_vehicle_thrust_setpoint_pub_->publish(msg);
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published VehicleThrustSetpoint message");
}

// Function to arm the vehicle
void arm() {
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Sending arm command...");
    publishVehicleCommand(400, 1.0);  // Command 400 is for arming
}

// Function to disarm the vehicle
void disarm(bool kill = false) {
    float param2 = kill ? 21196.0 : 0.0;  // 21196.0 is the kill switch value
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Sending disarm command (kill=%d)...", kill);
    publishVehicleCommand(400, 0.0, param2);  // Command 400 is for disarming
}

// Function to switch to offboard mode
void offboard_mode() {
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Sending offboard mode command...");
    publishVehicleCommand(176, 1.0, 6.0);  // Command 176 is for offboard mode
}

// Timer callback to publish OffboardControlMode and arm the vehicle
void publishOffboardControlAndArm() {
    publishOffboardControlMode();
    arm();
    //publishVehicleAttitudeSetpoint(0.7071, 0.0, 0.7071, 0.0, 0.5);
    //RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Published OffboardControlMode and arm command");
}

// Callback for VehicleControlMode messages
void vehicleControlModeCallback(const VehicleControlMode::SharedPtr msg) {
    if (!is_offboard_enabled.load()) {
        RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"),
                    "Vehicle Control Mode: Armed=%d, Offboard=%d, Manual=%d",
                    msg->flag_armed,
                    msg->flag_control_offboard_enabled,
                    msg->flag_control_manual_enabled);
                        // Update the shared variable
        is_offboard_enabled.store(msg->flag_control_offboard_enabled);
        offboard_mode();
    } else {
        return;
    }
}

void monitorConsoleInput(rclcpp::Node::SharedPtr node, std::atomic<bool>& running) {
    std::string input;
    rclcpp::TimerBase::SharedPtr timer = nullptr;  // Initialize the timer as nullptr
    while (running.load() && rclcpp::ok()) {
        std::cout << "Enter command (kill to disarm with kill switch, or press Enter to enable): ";
        std::getline(std::cin, input);

        if (input == "kill") {
            // Disarm with kill switch
            RCLCPP_INFO(node->get_logger(), "Kill command received. Disarming...");
            disarm(true);
            if (timer) {
                timer->cancel();  // Stop the timer if it is running
                RCLCPP_INFO(node->get_logger(), "Stopped publishing OffboardControlMode and arming commands.");
            }
        } else if (input.rfind("thrust:", 0) == 0) {  // Check if input starts with "thrust:"
            try {
                float thrust = std::stof(input.substr(7));  // Extract the thrust value
                RCLCPP_INFO(node->get_logger(), "Setting thrust to %.2f", thrust);
                //publishVehicleThrustSetpoint(thrust);  // Example: Set thrust with no roll/pitch/yaw
                //publishVehicleRatesSetpoint(0.0f, 0.0f, 0.0f, thrust);  // Example: Set thrust with no roll/pitch/yaw
                publishVehicleAttitudeSetpoint(0.7071f, 0.0f, 0.7071f, 0.0f, thrust);  // Example: 90-degree rotation around Z-axis
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Invalid thrust value. Please enter a valid number.");
            }
        }else {
            // Enable the timer if it is not already running
            if (!timer || timer->is_canceled()) {  // Check if the timer is null or canceled
                timer = node->create_wall_timer(
                    20ms,  // 20ms = 50 Hz
                    publishOffboardControlAndArm);
                RCLCPP_INFO(node->get_logger(), "Started publishing OffboardControlMode and arming commands.");
            } else {
                RCLCPP_INFO(node->get_logger(), "Timer is already running.");
            }
        }
    }
}

void thrustConsoleController(){
     
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::QoS qos(10);  // Depth of 10
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    auto node = rclcpp::Node::make_shared("offboard_control_node");

    // Initialize publishers
    ros_offboard_control_mode_pub_ = node->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    ros_vehicle_command_pub_ = node->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    ros_attitude_setpoint_pub_  = node->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
    ros_vehicle_rates_setpoint_pub_ = node->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
    ros_vehicle_thrust_setpoint_pub_ = node->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);

    // Initialize subscribers
    auto ros_vehicle_mode_sub_ = node->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, vehicleControlModeCallback);
    auto ros_vehicle_attitude_sub_ = node->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos, VehicleAttitudeCallback);
    auto ros_vehicle_local_position_sub_ = node->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, VehicleLocalPositionCallback);
    auto ros_vehicle_status_sub_ = node->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos, VehicleStatusCallback);

    RCLCPP_INFO(node->get_logger(), "Waiting for offboard mode to be enabled...");
    while (!is_offboard_enabled.load() && rclcpp::ok()) {
        rclcpp::spin_some(node);  // Process incoming messages
        std::this_thread::sleep_for(100ms);  // Avoid busy-waiting
    }
    RCLCPP_INFO(node->get_logger(), "Offboard mode enabled! Proceeding...");

    // Create a timer to periodically publish messages
    std::atomic<bool> running(true);
    std::thread console_thread(monitorConsoleInput, node, std::ref(running));  // Pass running as a reference

    rclcpp::spin(node);
    running.store(false);  // Signal the thread to stop
    console_thread.join();  // Wait for the console thread to finish
    rclcpp::shutdown();
    return 0;
}

