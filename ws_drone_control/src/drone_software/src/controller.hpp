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

class Controller
{
public:
    Controller(rclcpp::Node::SharedPtr node);                                              // Constructor to initialize the node
    void initialize(rclcpp::Node::SharedPtr node);                                         // Activation of ROS topics
    void publishVehicleAttitudeSetpoint(float roll, float pitch, float thrust, float yaw); // Publish attitude setpoint
    void startGoalPositionThread();                                                        // Start thread for goal position control
    void stopGoalPositionThread();                                                         // Stop the goal position thread
    void viconCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);             // Callback for Vicon data

    std::array<float, 3> goal_position;   // Store the goal position
    float goal_yaw;                       // Store the goal yaw
    std::array<float, 6> vicon_position_; // Store the latest Vicon position

    void setXYOuterGains(float kp, float kd);
    void setXYInnerGains(float kp, float kd);
    void setZGains(float kp, float kd);

    // Add getter methods for the current values
    std::pair<float, float> getXYOuterGains() const { return {Kp_xy_outer, Kd_xy_outer}; }
    std::pair<float, float> getXYInnerGains() const { return {Kp_xy_inner, Kd_xy_inner}; }
    std::pair<float, float> getZGains() const { return {Kp_z, Kd_z}; }

private:
    rclcpp::Node::SharedPtr node_;                       // Store the shared node
    std::array<float, 3> vicon_velocity_;                // Store the latest Vicon velocity
    rclcpp::Time prev_vicon_time_;                       // Store the previous Vicon timestamp
    float vicon_dt_;                                     // Time difference between Vicon readings
    std::array<float, 3> prev_pos_ = {0.0f, 0.0f, 0.0f}; // Store the previous position for velocity calculation

    std::array<float, 2> xyToRollPitch(float x_error, float y_error, float vx_err, float vy_err, float dt); // PID for XY roll pitch calculation
    float zToThrust(float z_error, float dt);                                                               // PID for thrust control
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_;        // Publisher for vehicle attitude setpoint
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr ros_vehicle_attitude_sub_;              // Subscription for vehicle attitude
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ros_vicon_sub_;                       // Subscription for Vicon data
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);                      // Callback for vehicle attitude
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw);                               // Convert roll, pitch, yaw to quaternion because of the PX4 message format

    std::thread goal_position_thread_;        // Thread for goal position control
    std::atomic<bool> stop_thread_;           // Flag to stop the goal position thread
    std::thread z_control_thread_;            // Thread for zControlMode
    std::atomic<bool> stop_z_control_{false}; // Flag to stop zControlMode
    std::atomic<float> target_z_{0.0f};       // Target z position
    std::atomic<float> max_z_thrust_{0.0f};   // Maximum z thrust

    std::array<float, 4> vehicle_attitude_quaternion_; // Store the latest vehicle attitude quaternion
    float initial_yaw_offset_;                         // Initial yaw offset for Vicon data

    mutable std::mutex vicon_mutex_;          // Mutex for Vicon data synchronization
    std::condition_variable vicon_update_cv_; // Condition variable for Vicon updates
    bool vicon_updated_ = false;              // Flag to indicate a new Vicon update

    rclcpp::Publisher<drone_software::msg::DebugVariables>::SharedPtr ros_debug_pub_;                                                                                                                                                                                                                                                                                     // Publisher for debug variables
    void publishDebugVariables(const std::array<float, 3> &position, const std::array<float, 3> &velocity, const std::array<float, 3> &errors, float thrust, const std::array<float, 3> &goal_position, float roll_desired_pre_clamp, float pitch_desired_pre_clamp, float Kp_xy_outer, float Kd_xy_outer, float Kp_xy_inner, float Kd_xy_inner, float Kp_z, float Kd_z, float x_integral_inner, float y_integral_inner, float Ki_xy_inner, float z_integral_inner, float Ki_z); // Updated signature

    float pi_ = 3.14159265358979323846; // Pi constant

    float getYawOffset(float vicon_yaw); //Calcuate the yaw offset

    // Integral Terms for PID controllers (commented out for now)

    float Kp_xy_outer = 2.0f; //0.111f; //Måske prøve 0.9804f   ?
    float Kd_xy_outer = 0.2f;//0.6128f; //0.1804f; //Måske prøve 0.6128f   ?


    float x_integral_inner_ = 0.0f; // Integral term for x inner PID controller
    float y_integral_inner_ = 0.0f; // Integral term for y inner PID controller

    float Kp_xy_inner = 0.3f;//0.2976; //0.1f; //Måske prøve 0.2788f   ?
    float Ki_xy_inner = 0.03f; // Integral gain for xy inner PID controller
    float Kd_xy_inner = 0.0465f; //0.05f; //Måske prøve 0.0523f   ?


    float z_integral_inner_ = 0.0f; // Integral term for z inner PID controller

    float Kp_z = 1.3f; //Stable p 1.3f | 0.8173f Settling time 10s | Perhaps 0.20430f (Settling time 20s)
    float Ki_z = 0.25f; // Integral term for z PID controller
    float Kd_z = 0.75f; //Stable d 1.0f | 2.2140f Settling time 10s | Perhaps 1.10700f (Settling time 20s)
};

#endif