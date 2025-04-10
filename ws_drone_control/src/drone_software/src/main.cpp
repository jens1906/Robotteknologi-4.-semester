#include <rclcpp/rclcpp.hpp>
#include "initialize.hpp"
#include "controller.hpp"
#include <thread>
#include <atomic>
#include <iostream>

int main(int argc, char *argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a shared node
    auto node = rclcpp::Node::make_shared("offboard_control_node");

    // Create an instance of the Initialize class
    Initialize initialize(node);
    Controller controller(node);
    RCLCPP_INFO(node->get_logger(), "Enabling offboard mode...");
    initialize.enable_offboard_mode();
    controller.initialize(node);

    // Atomic flag to control the turn_on_drone thread
    std::atomic<bool> running(false);
    std::atomic<bool> terminate(false);

    // Thread for running turn_on_drone in a loop
    std::thread drone_thread([&]() {
        while (!terminate.load()) {
            if (running.load()) {
                initialize.turn_on_drone();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Adjust loop frequency as needed
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Avoid busy-waiting
            }
        }
    });

    // Monitor user input
    std::string input;
    while (rclcpp::ok()) {
        std::cout << "Enter command (turnon, turnoff, kill, setpoint): ";
        std::getline(std::cin, input);

        if (input == "turnon") {
            RCLCPP_INFO(node->get_logger(), "Turning on the drone...");
            running.store(true);  // Start the drone thread
        } else if (input == "turnoff") {
            RCLCPP_INFO(node->get_logger(), "Turning off the drone...");
            controller.publishVehicleAttitudeSetpoint({0.0f, 0.0f, 0.0f}, 0.0f);  // Example setpoint
            running.store(false);  // Stop the drone thread
        } else if (input == "kill") {
            RCLCPP_INFO(node->get_logger(), "Kill command received. Disarming...");
            running.store(false);  // Stop the drone thread
            initialize.disarm(true);  // Disarm the drone
        } else if (input == "exit") {
            RCLCPP_INFO(node->get_logger(), "Exiting...");
            terminate.store(true);  // Signal the thread to terminate
            break;
        } else if (input == "setpoint") {
            RCLCPP_INFO(node->get_logger(), "Altitude setpoint command received.");
            controller.goalPosition({1.0f, 1.0f, 1.0f});  // Example setpoint
        } else {
            RCLCPP_WARN(node->get_logger(), "Unknown command: %s", input.c_str());
        }
    }

    // Wait for the thread to finish
    drone_thread.join();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

