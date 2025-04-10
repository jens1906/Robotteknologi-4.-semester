#ifndef DRONE_CONTROLLER_HPP
#define DRONE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace drone_control {  // Add this namespace

class DroneController : public rclcpp::Node
{
public:
    DroneController();

private:
    void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void orientation_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr orientation_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_subscription_;

    // Declare the publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_publisher_;
    
    // Declare member variables
    geometry_msgs::msg::PoseStamped current_position_;
    geometry_msgs::msg::PoseStamped current_orientation_;
    double current_speed_;
    
    // Declare the control function
    double compute_control();
};

}  // namespace drone_control

#endif // DRONE_CONTROLLER_HPP