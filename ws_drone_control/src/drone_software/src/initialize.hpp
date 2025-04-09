#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>


class Initialize {
public:
    Initialize(rclcpp::Node::SharedPtr node);
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr ros_vehicle_command_pub_;
    void publishVehicleCommand(uint16_t command, float param1, float param2 = 0.0);
    void disarm(bool kill = false);
    void enable_offboard_mode();
    void turn_on_drone();

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ros_offboard_control_mode_pub_;
    void arm();
    void publishOffboardControlMode();
};

#endif  