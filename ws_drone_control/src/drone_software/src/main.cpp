#include <rclcpp/rclcpp.hpp>
#include "initialize.hpp"
#include "controller.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <limits>

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
    std::atomic<bool> test_mode(false);  // New atomic flag for test mode
    std::thread test_thread;

    std::cout << "--------------------------------------------" << std::endl;

    while (rclcpp::ok()) {
        std::cout << "Enter command (turnon, turnoff, kill, start, stop, setpoint, test, exit): ";
        std::getline(std::cin, input);

        if (input == "turnon") {
            RCLCPP_INFO(node->get_logger(), "Turning on the drone...");
            running.store(true);  // Start the drone thread
            controller.publishVehicleAttitudeSetpoint({0.0f, 0.0f, 0.0f}, 0.0f);  // Example setpoint
        } else if (input == "turnoff") {
            RCLCPP_INFO(node->get_logger(), "Turning off the drone...");
            controller.stopGoalPositionThread();
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
            if (test_thread.joinable()) {
                test_mode.store(false);  // Stop the test thread
                test_thread.join();
            }
            break;
        } else if (input == "start") {
            float x, y, z;
            std::cout << "Enter x, y, and z values separated by spaces: ";
            std::cin >> x >> y >> z;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

            RCLCPP_INFO(node->get_logger(), "Starting goal position thread with x: %.2f, y: %.2f, z: %.2f", x, y, z);
            controller.startGoalPositionThread({x, y, z});  // Use user-provided goal position
        } else if (input == "stop") {
            RCLCPP_INFO(node->get_logger(), "Stopping goal position thread...");
            controller.stopGoalPositionThread();
        } else if (input == "setpoint") {
            RCLCPP_INFO(node->get_logger(), "Altitude setpoint command received.");
            controller.goalPosition({1.0f, 0.0f, 0.80f});  // Example setpoint
        } else if (input == "test") {
            if (!test_mode.load()) {
                float x, y, z, yaw;
                std::cout << "Enter x, y, z, and yaw values separated by spaces: ";
                std::cin >> x >> y >> z >> yaw;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

                test_mode.store(true);
                test_thread = std::thread([&]() {
                    controller.simulateDroneCommands({x, y, z}, yaw);  // Execute the command once
                    test_mode.store(false);  // Reset the test mode flag
                });
                if (test_thread.joinable()) {
                    test_thread.join();  // Ensure the thread completes before proceeding
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Test mode is already running.");
            }
        } else if (input == "manual") {
            float motor_power;
            std::cout << "Enter motor power (e.g., between 0.0 and 1.0): ";
            std::cin >> motor_power;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

            if (motor_power < 0.0f || motor_power > 1.0f) {
                RCLCPP_WARN(node->get_logger(), "Invalid motor power value. Please enter a value between 0.0 and 1.0.");
            } else {
                RCLCPP_INFO(node->get_logger(), "Setting motor power to %.2f.", motor_power);
                controller.manualMotorSet(motor_power);
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Unknown command: %s", input.c_str());
        }
        std::cout << "--------------------------------------------" << std::endl;
    }

    // Wait for the thread to finish
    drone_thread.join();
    if (test_thread.joinable()) {
        test_thread.join();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

