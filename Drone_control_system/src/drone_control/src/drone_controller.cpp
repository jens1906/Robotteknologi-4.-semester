#include "drone_control/drone_controller.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

namespace drone_control {

DroneController::DroneController()
    : Node("drone_controller")
{
    // Subscriptions
    position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/position", 10,
        std::bind(&DroneController::position_callback, this, std::placeholders::_1));

    orientation_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone/orientation", 10,
        std::bind(&DroneController::orientation_callback, this, std::placeholders::_1));

    speed_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "drone/speed", 10,
        std::bind(&DroneController::speed_callback, this, std::placeholders::_1));

    // Publisher for PX4 desired attitude
    attitude_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "fmu/vehicle_attitude_setpoint/in", 10);
}

void DroneController::position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Store the latest position
    current_position_ = *msg;

    // Compute the control output
    double desired_angle = compute_control();

    // Publish the desired attitude setpoint to PX4
    auto attitude_msg = px4_msgs::msg::VehicleAttitudeSetpoint();

    // Convert the desired angle to a quaternion (assuming roll control)
    attitude_msg.q_d[0] = std::cos(desired_angle / 2.0);  // w
    attitude_msg.q_d[1] = std::sin(desired_angle / 2.0);  // x (roll)
    attitude_msg.q_d[2] = 0.0;                           // y (pitch)

    // Publish the message
    attitude_publisher_->publish(attitude_msg);

    RCLCPP_INFO(this->get_logger(), "Published desired attitude setpoint.");
}

void DroneController::orientation_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Store the latest orientation
    current_orientation_ = *msg;

    RCLCPP_INFO(this->get_logger(), "Received orientation.");
}

void DroneController::speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Store the latest speed
    current_speed_ = msg->data;

    RCLCPP_INFO(this->get_logger(), "Received speed: [%f]", msg->data);
}

double DroneController::compute_control()
{
    // Control logic
    double x = current_position_.pose.position.x;
    double error = 0.0 - x; // Assume we want to stabilize at x = 0

    // PD controller parameters
    double z_c = 0.9230;  // Controller zero
    double K = 7.3441;    // Controller gain

    // Discrete-time PD control logic
    static double previous_error = 0.0; // Store the previous error
    double T_s = 0.01;                   // Sampling time (10 ms)

    // Compute the derivative of the error
    double derivative = (error - previous_error) / T_s;

    // Compute the control output
    double control_output = K * (error + z_c * derivative);

    // Update the previous error
    previous_error = error;

    return control_output;
}

}  // namespace drone_control