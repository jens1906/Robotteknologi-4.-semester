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

    // Create an instance of the Initialize classes
    Initialize initialize(node);
    Controller controller(node);
    RCLCPP_INFO(node->get_logger(), "Enabling offboard mode...");
    initialize.enable_offboard_mode();
    controller.initialize(node);

    // The next to are used to control if the drone are turned on or off
    std::atomic<bool> running(false);
    std::atomic<bool> terminate(false);

    // Because we need arming and publishoffboardcontrolmode to be sent often 
    // we multithread and makes it so these commands are run in the background
    // depending on the atomic bools.
    std::thread drone_thread([&]() {
        while (!terminate.load()) {
            if (running.load()) {
                initialize.turn_on_drone();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Adjust loop frequency as needed
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Avoid busy-waiting aka let the cpu chill
            }
        }
    });

    // A little control panel
    std::string input;
    while (rclcpp::ok()) {
        std::cout << "Enter command (turnon, turnoff, kill, setpoint): ";
        std::getline(std::cin, input);

        if (input == "turnon") {
            RCLCPP_INFO(node->get_logger(), "Turning on the drone...");
            running.store(true);  // Start the drone thread
            controller.publishVehicleAttitudeSetpoint({0.0f, 0.0f, 0.0f}, 0.0f);  // Example setpoint
        } else if (input == "turnoff") {
            RCLCPP_INFO(node->get_logger(), "Turning off the drone...");
            controller.publishVehicleAttitudeSetpoint({0.0f, 0.0f, 0.0f}, 0.0f);  // Example setpoint
            running.store(false);  // Stop the drone thread
        } else if (input == "kill") {
            RCLCPP_INFO(node->get_logger(), "Kill command received. Disarming...");
            controller.publishVehicleAttitudeSetpoint({0.0f, 0.0f, 0.0f}, 0.0f);  // Example setpoint
            running.store(false);  // Stop the drone thread
            initialize.disarm(true);  // Disarm the drone
        } else if (input == "exit") {
            RCLCPP_INFO(node->get_logger(), "Exiting...");
            terminate.store(true);  // Signal the thread to terminate
            controller.stopGoalPositionThread();
            break;
        } else if (input == "start") {
            RCLCPP_INFO(node->get_logger(), "Starting goal position thread...");
            controller.startGoalPositionThread({1.0f, 0.0f, 0.80f});  // Example goal position
        } else if (input == "stop") {
            RCLCPP_INFO(node->get_logger(), "Stopping goal position thread...");
            controller.stopGoalPositionThread();
        } else if (input == "setpoint") {
            RCLCPP_INFO(node->get_logger(), "Altitude setpoint command received.");
            controller.goalPosition({1.0f, 0.0f, 0.80f});  // Example setpoint
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

