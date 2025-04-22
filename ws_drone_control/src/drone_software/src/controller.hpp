#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include <array>
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

class Controller {
public:
    Controller(rclcpp::Node::SharedPtr node); // Constructor to initialize the node
    void initialize(rclcpp::Node::SharedPtr node); // Activation of ROS topics
    void publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw); // Publish attitude setpoint
    void goalPosition(const std::array<float, 3>& goal_position); // Process goal position and control logic

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    std::array<float, 6> vicon_position_;  // Store the latest Vicon position
    std::array<float, 2> xyToRollPitch(float x_error, float y_error); // PD control for roll/pitch
    float zToThrust(float z_error); // PD control for thrust

    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_; // Publisher
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr ros_vehicle_attitude_sub_; // Sub to vehicle attitude
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ros_vicon_sub_; // Sub to Vicon

    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg); // Callback
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw); // RPY to quaternion
};

#endif
