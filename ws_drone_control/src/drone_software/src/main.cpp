#include <rclcpp/rclcpp.hpp>
#include "initialize.hpp"
#include "controller.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <limits>

bool is_goal_position_thread_running = false;       // Flag to control the goal position thread
std::array<float, 2> landing_position = {100, 100}; // Store the latest Vicon position

int main(int argc, char *argv[])
{
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

    // The next to are used to control if the drone ar<<<<<<< Updated upstream
    std::atomic<bool> running(false);
    std::atomic<bool> terminate(false);

    // Because we need arming and publishoffboardcontrolmode to be sent often
    // we multithread and makes it so these commands are run in the background
    // depending on the atomic bools.
    std::thread drone_thread([&]()
                             {
        while (rclcpp::ok() && !terminate.load()) {
            static auto last_drone_update = std::chrono::steady_clock::now();
        
            // Process incoming ROS 2 messages
            try {
                rclcpp::spin_some(node);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Error during spin_some: %s", e.what());
            }
        
            // Check if it's time to call `initialize.turn_on_drone()`
            auto now = std::chrono::steady_clock::now();
            if (running.load() && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_drone_update).count() >= 250) {
                initialize.turn_on_drone();  // Send the keep-alive command
                last_drone_update = now;    // Update the last execution time
            }
        
            // Sleep for a short duration to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } });

    // A little control panel
    std::string input;
    std::atomic<bool> test_mode(false); // New atomic flag for test mode
    std::thread test_thread;
    std::atomic<bool> vicon_test_mode(false); // New atomic flag for Vicon test mode
    std::thread vicon_test_thread;

    std::cout << "--------------------------------------------" << std::endl;

    while (rclcpp::ok())
    {
        std::cout << "Enter command (turnon, turnoff, kill, start, stop, exit, tune): ";
        std::getline(std::cin, input);

        if (input == "turnon")
        {
            RCLCPP_INFO(node->get_logger(), "Turning on the drone...");
            running.store(true); // Start the drone thread
        }
        else if (input == "turnoff")
        {
            RCLCPP_INFO(node->get_logger(), "Turning off the drone...");
            auto last_execution_time = std::chrono::steady_clock::now();
            controller.goal_position = {
                controller.vicon_position_[0],
                controller.vicon_position_[1],
                controller.goal_position[2]};
            landing_position = {100, 100};
            while (std::abs(landing_position[1] - landing_position[0]) > 0.005f || landing_position[1] == 100)
            {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_execution_time);
                if (elapsed_time.count() >= 0.12f)
                {                                                        // Check if 2 seconds have passed
                    landing_position[1] = landing_position[0];           // Update landing position
                    landing_position[0] = controller.vicon_position_[2]; // Update landing position
                    // Code to execute every 2 seconds
                    controller.goal_position = {
                        controller.goal_position[0],
                        controller.goal_position[1],
                        controller.goal_position[2] - 0.15f}; // Gradually reduce the z-coordinate

                    // Reset the timer
                    last_execution_time = current_time;
                    std::cout << "Vicon Position: z=" << controller.vicon_position_[2] << std::endl;
                    std::cout << "Goal Position: z=" << controller.goal_position[2] << std::endl;
                    std::cout << "Landing Position: z=" << (landing_position[1] - landing_position[0]) << std::endl;
                }
            }

            controller.stopGoalPositionThread();
            running.store(false); // Stop the drone thread
        }
        else if (input == "kill")
        {
            RCLCPP_INFO(node->get_logger(), "Kill command received. Disarming...");
            running.store(false);                                              // Stop the drone thread
            initialize.disarm(true);                                           // Disarm the drone
        }
        else if (input == "exit")
        {
            RCLCPP_INFO(node->get_logger(), "Exiting...");
            terminate.store(true); // Signal the thread to terminate
            controller.stopGoalPositionThread();
            initialize.disablePublishing(); // Disable publishing for safety
            if (test_thread.joinable())
            {
                test_mode.store(false); // Stop the test thread
                test_thread.join();
            }
            if (vicon_test_thread.joinable())
            {
                vicon_test_mode.store(false); // Stop the Vicon test thread
                vicon_test_thread.join();
            }
            rclcpp::shutdown();
            break; // Correctly placed within the while loop
        }
        else if (input == "start")
        {
            float x, y, z, yaw;
            std::cout << "Enter x, y, z, and yaw values separated by spaces: ";
            std::cin >> x >> y >> z >> yaw;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

            RCLCPP_INFO(node->get_logger(), "Starting goal position thread with x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", x, y, z, yaw);
            controller.goal_position = {x, y, z}; // Store the goal position
            controller.goal_yaw = yaw;            // Store the goal yaw

            if (is_goal_position_thread_running == false)
            {
                controller.startGoalPositionThread();   // Use user-provided goal position
                is_goal_position_thread_running = true; // Set the flag to true
            }
        }
        else if (input == "stop")
        {
            RCLCPP_INFO(node->get_logger(), "Stopping goal position thread...");
            controller.stopGoalPositionThread();
        }
        else if (input == "tune")
        {
            std::string gain_type;
            std::cout << "Enter gain type (xy_outer, xy_inner, z): ";
            std::getline(std::cin, gain_type);

            if (gain_type == "xy_outer")
            {
                float kp, kd;
                std::cout << "Enter Kp and Kd for XY outer loop (current: Kp="
                          << controller.getXYOuterGains().first << ", Kd="
                          << controller.getXYOuterGains().second << "): ";
                std::cin >> kp >> kd;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                controller.setXYOuterGains(kp, kd);
                RCLCPP_INFO(node->get_logger(), "Updated XY outer gains: Kp=%.4f, Kd=%.4f", kp, kd);
            }
            else if (gain_type == "xy_inner")
            {
                float kp, kd;
                std::cout << "Enter Kp and Kd for XY inner loop (current: Kp="
                          << controller.getXYInnerGains().first << ", Kd="
                          << controller.getXYInnerGains().second << "): ";
                std::cin >> kp >> kd;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                controller.setXYInnerGains(kp, kd);
                RCLCPP_INFO(node->get_logger(), "Updated XY inner gains: Kp=%.4f, Kd=%.4f", kp, kd);
            }
            else if (gain_type == "z")
            {
                float kp, kd;
                std::cout << "Enter Kp and Kd for Z control (current: Kp="
                          << controller.getZGains().first << ", Kd="
                          << controller.getZGains().second << "): ";
                std::cin >> kp >> kd;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                controller.setZGains(kp, kd);
                RCLCPP_INFO(node->get_logger(), "Updated Z gains: Kp=%.4f, Kd=%.4f", kp, kd);
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Unknown gain type: %s", gain_type.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Unknown command: %s", input.c_str());
        }
        std::cout << "--------------------------------------------" << std::endl;
    }

    // Wait for the thread to finish
    drone_thread.join();
    if (test_thread.joinable())
    {
        test_thread.join();
    }
    if (vicon_test_thread.joinable())
    {
        vicon_test_thread.join();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}