#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

// This is the layout file for the code, first the needed libaries are included
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <atomic>

// Next we set up which functinos are public and private
// and which variables are public and private
// AKA which functions can only be used in the class and which can be used outside the class
class Initialize
{
public:
    Initialize(rclcpp::Node::SharedPtr node);                                             // Activates the ROS publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr ros_vehicle_command_pub_; // Publisher for vehicle commands
    void publishVehicleCommand(uint16_t command, float param1, float param2 = 0.0);       // Controlls the setup of a vehicle command
    void disarm(bool kill = false);                                                       // Setup for a diarm command aka kill
    void enable_offboard_mode();                                                          // Setup for the offboard mode
    void turn_on_drone();                                                                 // Setup for the drone to turn on just to functions
    void disablePublishing();                                                             // Disable publishing to ROS topics

private:
    rclcpp::Node::SharedPtr node_;                                                                   // Connected to the ROS node
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ros_offboard_control_mode_pub_; // Publisher for offboard control mode
    std::atomic<bool> can_publish_{true};                                                            // Flag to control whether publishing is allowed
    void arm();                                                                                      // Fucntion to arm the drone
    void publishOffboardControlMode();                                                               // Function to publish the offboard control mode
};

#endif