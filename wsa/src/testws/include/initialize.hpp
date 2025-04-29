#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>

class Initialize {
public:
    explicit Initialize(rclcpp::Node::SharedPtr node);

    void arm();
    void disarm();
    void enable_offboard_mode();
    void spin_motor(uint8_t motor_index, float throttle);

private:
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_pub_;
};

#endif // INITIALIZE_HPP
