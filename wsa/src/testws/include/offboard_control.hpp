#ifndef OFFBOARD_CONTROL_HPP
#define OFFBOARD_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl();

    void arm();
    void disarm();
    void turn_on_drone(); // New method to turn on the drone
    void start_offboard_control();

private:
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr arming_timer_;  // Timer for continuous arming commands
    int offboard_setpoint_counter_;
};

#endif // OFFBOARD_CONTROL_HPP
