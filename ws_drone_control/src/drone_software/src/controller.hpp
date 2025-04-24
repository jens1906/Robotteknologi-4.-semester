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
#include <condition_variable>
#include <mutex>

class Controller {
public:
    Controller(rclcpp::Node::SharedPtr node); // Constructor to initialize the node
    void initialize(rclcpp::Node::SharedPtr node); // Activation of ROS topics
    void publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw); // Control the format of the attitude setpoint commands
    void goalPosition(const std::array<float, 3>& goal_position); // Get the goal position
    void startGoalPositionThread(const std::array<float, 3>& goal_position);
    void stopGoalPositionThread();
    void simulateDroneCommands(const std::array<float, 3>& xyz_error, float yaw);
    void manualMotorSet(float T); // Set the motor thrust directly

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    std::array<float, 6> vicon_position_;  // Store the latest Vicon position
    std::array<float, 3> vicon_velocity_;  // Add this line
    rclcpp::Time prev_vicon_time_;         // Add this line
    std::array<float, 2> xyToRollPitch(float x_error, float y_error, float vx_err, float vy_err); // PID for XY roll pitch calculation
    float zToThrust(float z_error); // PID for thrust control
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_; // Publisher for vehicle attitude setpoint
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr ros_vehicle_attitude_sub_; // Subscription for vehicle attitude
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ros_vicon_sub_; // Subscription for Vicon data
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg); // Callback for vehicle attitude
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw); // Convert roll, pitch, yaw to quaternion because of the PX4 message format

    std::thread goal_position_thread_;
    std::atomic<bool> stop_thread_;

    std::condition_variable vicon_update_cv_; // Condition variable for Vicon updates
    std::mutex vicon_mutex_;                 // Mutex for Vicon data synchronization
    bool vicon_updated_ = false;             // Flag to indicate a new Vicon update
};

#endif
