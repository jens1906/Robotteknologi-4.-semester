#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

class SimpleOffboardControl : public rclcpp::Node {
public:
    SimpleOffboardControl() : Node("simple_offboard_control") {
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { this->control_loop(); });
    }

private:
    void control_loop() {
        if (counter_ < 10) {
            // Publish OffboardControlMode
            px4_msgs::msg::OffboardControlMode offboard_mode_msg{};
            offboard_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_mode_msg.position = false;
            offboard_mode_msg.velocity = false;
            offboard_mode_msg.acceleration = false;
            offboard_mode_msg.attitude = true;
            offboard_mode_msg.body_rate = false;
            offboard_control_mode_publisher_->publish(offboard_mode_msg);

            // Publish VehicleCommand to set offboard mode
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        } else if (counter_ == 10) {
            // Arm the drone
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        }

        // Continuously publish OffboardControlMode and VehicleAttitudeSetpoint
        if (counter_ >= 10) {
            // Publish OffboardControlMode
            px4_msgs::msg::OffboardControlMode offboard_mode_msg{};
            offboard_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_mode_msg.position = false;
            offboard_mode_msg.velocity = false;
            offboard_mode_msg.acceleration = false;
            offboard_mode_msg.attitude = true;
            offboard_mode_msg.body_rate = false;
            offboard_control_mode_publisher_->publish(offboard_mode_msg);

            // Publish VehicleAttitudeSetpoint
            px4_msgs::msg::VehicleAttitudeSetpoint attitude_msg{};
            attitude_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            attitude_msg.q_d = {1.0, 0.0, 0.0, 0.0};  // Quaternion: no rotation
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
            attitude_msg.thrust_body = {0.0, 0.0, -0.5};  // Thrust of 0.5
=======
            attitude_msg.thrust_body = {0.0, 0.0, -0.1};  // Thrust of 0.5
>>>>>>> Stashed changes
=======
            attitude_msg.thrust_body = {0.0, 0.0, -0.1};  // Thrust of 0.5
>>>>>>> Stashed changes
=======
            attitude_msg.thrust_body = {0.0, 0.0, -0.1};  // Thrust of 0.5
>>>>>>> Stashed changes
=======
            attitude_msg.thrust_body = {0.0, 0.0, -0.1};  // Thrust of 0.5
>>>>>>> Stashed changes
            attitude_setpoint_publisher_->publish(attitude_msg);
        }

        counter_++;
    }

    void publish_vehicle_command(uint16_t command, float param1, float param2) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleOffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
