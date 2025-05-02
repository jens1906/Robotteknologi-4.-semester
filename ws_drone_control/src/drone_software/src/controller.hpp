#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include <array>
#include <thread>
#include <atomic>
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <drone_software/msg/debug_variables.hpp>
#include <condition_variable>
#include <mutex>

class Controller {
public:
    Controller(rclcpp::Node::SharedPtr node); // Constructor to initialize the node
    void initialize(rclcpp::Node::SharedPtr node); // Activation of ROS topics
    void publishVehicleAttitudeSetpoint(float roll, float pitch, float thrust, float yaw); // Publish attitude setpoint
    void startGoalPositionThread(); // Start thread for goal position control
    void stopGoalPositionThread(); // Stop the goal position thread
    void simulateDroneCommands(const std::array<float, 3>& xyz_error, float yaw); // Simulate drone commands
    void manualMotorSet(float T); // Set the motor thrust directly
    void viconCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg); // Callback for Vicon data

    // New methods for zControlMode
    void zControlMode(float z_offset, float max_z_thrust); // Control z-axis position
    void stopZControlMode(); // Stop zControlMode thread

    std::array<float, 3> goal_position; // Store the goal position
    float goal_yaw; // Store the goal yaw
    std::array<float, 6> vicon_position_;  // Store the latest Vicon position

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    std::array<float, 3> vicon_velocity_;  // Store the latest Vicon velocity
    rclcpp::Time prev_vicon_time_;         // Store the previous Vicon timestamp
    float vicon_dt_;  // Time difference between Vicon readings
    std::array<float, 3> prev_pos_ = {0.0f, 0.0f, 0.0f}; // Store the previous position for velocity calculation

    std::array<float, 2> xyToRollPitch(float x_error, float y_error, float vx_err, float vy_err, float dt); // PID for XY roll pitch calculation
    float zToThrust(float z_error, float dt); // PID for thrust control
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_; // Publisher for vehicle attitude setpoint
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr ros_vehicle_attitude_sub_; // Subscription for vehicle attitude
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ros_vicon_sub_; // Subscription for Vicon data
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg); // Callback for vehicle attitude
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw); // Convert roll, pitch, yaw to quaternion because of the PX4 message format

    std::thread goal_position_thread_; // Thread for goal position control
    std::atomic<bool> stop_thread_; // Flag to stop the goal position thread
    std::thread z_control_thread_; // Thread for zControlMode
    std::atomic<bool> stop_z_control_{false}; // Flag to stop zControlMode
    std::atomic<float> target_z_{0.0f}; // Target z position
    std::atomic<float> max_z_thrust_{0.0f}; // Maximum z thrust

    std::array<float, 4> vehicle_attitude_quaternion_; // Store the latest vehicle attitude quaternion
    float initial_yaw_offset_; // Initial yaw offset for Vicon data

    mutable std::mutex vicon_mutex_; // Mutex for Vicon data synchronization
    std::condition_variable vicon_update_cv_; // Condition variable for Vicon updates
    bool vicon_updated_ = false; // Flag to indicate a new Vicon update

    rclcpp::Publisher<drone_software::msg::DebugVariables>::SharedPtr ros_debug_pub_; // Publisher for debug variables
    void publishDebugVariables(const std::array<float, 3>& position, const std::array<float, 3>& velocity, const std::array<float, 3>& errors, float thrust, const std::array<float, 3>& goal_position); // Publish debug variables
};

#endif