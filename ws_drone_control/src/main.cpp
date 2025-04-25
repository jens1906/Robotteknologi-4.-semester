if (input == "zcon") {
            float z_offset, max_z_thrust;
            std::cout << "Enter target z offset (e.g., 0.5 for 0.5m above current position): ";
            std::cin >> z_offset;
            std::cout << "Enter maximum z thrust (e.g., between 0.0 and 1.0): ";
            std::cin >> max_z_thrust;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the input buffer

            if (max_z_thrust < 0.0f || max_z_thrust > 1.0f) {
                RCLCPP_WARN(node->get_logger(), "Invalid max z thrust value. Please enter a value between 0.0 and 1.0.");
            } else {
                RCLCPP_INFO(node->get_logger(), "Running zcon mode with target z offset: %.2f and max z thrust: %.2f.", z_offset, max_z_thrust);
                controller.zControlMode(z_offset, max_z_thrust);  // Start or update zControlMode
            }
        } else if (input == "exit") {
            RCLCPP_INFO(node->get_logger(), "Exiting...");
            terminate.store(true);  // Signal the thread to terminate
            controller.stopZControlMode();  // Stop the zControlMode thread
            controller.stopGoalPositionThread();
            initialize.disablePublishing(); // Disable publishing for safety
            if (test_thread.joinable()) {
                test_mode.store(false);  // Stop the test thread
                test_thread.join();
            }
            break;  // Correctly placed within the while loop
