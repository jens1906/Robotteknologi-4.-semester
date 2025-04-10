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
    Controller(rclcpp::Node::SharedPtr node);
    void initialize(rclcpp::Node::SharedPtr node);
    void publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw);
    std::array<float, 3> goalPosition(const std::array<float, 3>& goal_position);

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    std::array<float, 6> vicon_position_;  // Store the latest Vicon position
    std::array<float, 2> xyToRollPitch(float x_error, float y_error);
    std::array<float, 1> zToThrust(float z_error);
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_;
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw);
};

#endif