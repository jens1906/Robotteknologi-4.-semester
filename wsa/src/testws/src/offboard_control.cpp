#include "offboard_control.hpp"

OffboardControl::OffboardControl()
    : Node("offboard_control"), offboard_setpoint_counter_(0) {
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() -> void {
            // Publish OffboardControlMode and TrajectorySetpoint continuously
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // Increment the counter
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        });
}

void OffboardControl::arm() {
    if (!arming_timer_) {
        arming_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Publish every 100ms
            [this]() {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                RCLCPP_INFO(this->get_logger(), "Arming command sent.");
            });
    }
}

void OffboardControl::disarm() {
    if (arming_timer_) {
        arming_timer_->cancel();  // Stop the arming timer
        arming_timer_.reset();
    }
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
}

void OffboardControl::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;  // Enable position control
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};  // Hover at 5 meters
    msg.yaw = 0.0;                    // Yaw angle of 0 degrees
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::turn_on_drone() {
    RCLCPP_INFO(this->get_logger(), "Publishing OffboardControlMode...");
    publish_offboard_control_mode();

    RCLCPP_INFO(this->get_logger(), "Arming the drone...");
    arm();

    RCLCPP_INFO(this->get_logger(), "Drone is now armed and ready.");
}

void OffboardControl::start_offboard_control() {
    RCLCPP_INFO(this->get_logger(), "Starting offboard control...");
    // This method is currently a placeholder. If additional setup is required
    // for offboard control, it can be added here.
}
