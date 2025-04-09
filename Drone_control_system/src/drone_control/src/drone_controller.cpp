#include "drone_control/drone_controller.hpp"

namespace drone_control {  // Add this namespace

DroneController::DroneController()
    : Node("drone_controller")
{
    position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/position", 10,
        std::bind(&DroneController::position_callback, this, std::placeholders::_1));

    orientation_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/orientation", 10,
        std::bind(&DroneController::orientation_callback, this, std::placeholders::_1));

    speed_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "drone/speed", 10,
        std::bind(&DroneController::speed_callback, this, std::placeholders::_1));
}

void DroneController::position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received position: [%f, %f, %f]",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void DroneController::orientation_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received orientation: [%f, %f, %f]",
                msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void DroneController::speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received speed: [%f]", msg->data);
}

}  // namespace drone_control