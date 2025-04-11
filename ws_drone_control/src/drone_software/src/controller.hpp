#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include <array>  
#include <iostream> 
#include <std_msgs/msg/float64_multi_array.hpp>  
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

//Next we set up which functinos are public and private
//and which variables are public and private
//AKA which functions can only be used in the class and which can be used outside the class
class Controller {
public:
    Controller(rclcpp::Node::SharedPtr node); // Constructor to initialize the node
    void initialize(rclcpp::Node::SharedPtr node); // Actiovation of ros topics
    void publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw); // Control the format of the attitude setpoint commands
    std::array<float, 3> goalPosition(const std::array<float, 3>& goal_position); // Get the goal position

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    std::array<float, 6> vicon_position_;  // Store the latest Vicon position
    std::array<float, 2> xyToRollPitch(float x_error, float y_error); // PID for XY roll pitch calculation
    float zToThrust(float z_error); // PID for thrust control
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_; // Publisher for vehicle attitude setpoint
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg); // Callback for vehicle attitude
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw); // Convert roll, pitch, yaw to quaternion because of the px4 message format
};

#endif