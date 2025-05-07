#include "initialize.hpp"

// Firstly we actually needs to activate the ros topic publishers and make it so they are controllable by the node
Initialize::Initialize(rclcpp::Node::SharedPtr node) : node_(node)
{
    ros_vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    ros_offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
}

// By looking into the px4 message format we can find the layout to put vehicle commands.
// We only need the first 2 because of the first param1 is used for most, and param2 is used for mode.
void Initialize::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    if (!can_publish_.load())
    {
        RCLCPP_WARN(node_->get_logger(), "Publishing is disabled. Command not sent.");
        return;
    }

    px4_msgs::msg::VehicleCommand msg{};                        // Declare the message format
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects timestamp in microseconds
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    // Next is just constant for a px4 controller
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    ros_vehicle_command_pub_->publish(msg);
}

// If anything goes wrong we should be able to stop the drone, this stops it mid air
void Initialize::disarm(bool kill)
{
    float param2 = kill ? 21196.0 : 0.0;     // 21196.0 is the kill switch value
    publishVehicleCommand(400, 0.0, param2); // Command 400 is for disarming
}

// The drone needs constant reassurance of its state, so we need to send it a command to arm it often so here
void Initialize::arm()
{
    publishVehicleCommand(400, 1.0); // Command 400 is for arming
}

// The drone will not function without it being in offboard mode,
// so this function checks the current mode and send a command to chance mode until it is the right mode
void Initialize::enable_offboard_mode()
{
    // Just some node safety because it is the first function run
    if (!node_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("offboard_control_node"), "Node is not initialized!");
        return;
    }

    // Bool to check if the drone is in offboard mode
    std::atomic<bool> is_offboard_enabled(false);

    // These two lines discribe the topic we are subscribing to
    // and if there may be errors or not
    rclcpp::QoS qos(10);                                    // Depth of 10
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); // Set reliability to BEST_EFFORT

    // Now we get the subscription up and running so we get if the mode is offboard mode
    auto vehicle_control_mode_sub = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos,
        [&is_offboard_enabled, this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
        {
            if (!msg)
            {
                RCLCPP_ERROR(node_->get_logger(), "Received null message in VehicleControlMode subscription.");
                return;
            }

            // Log the results
            RCLCPP_INFO(node_->get_logger(),
                        "Vehicle Control Mode: Armed=%d, Offboard=%d, Manual=%d",
                        msg->flag_armed,
                        msg->flag_control_offboard_enabled,
                        msg->flag_control_manual_enabled);

            // Update the shared variable
            is_offboard_enabled.store(msg->flag_control_offboard_enabled);

            // If offboard mode is not enabled, send the command
            if (!msg->flag_control_offboard_enabled)
            {
                publishVehicleCommand(176, 1.0, 6.0); // Command 176 is for offboard mode
            }
        });

    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5); // Set timeout duration

    // Wait until offboard mode is enabled
    while (!is_offboard_enabled.load() && rclcpp::ok())
    {
        rclcpp::spin_some(node_);                                    // Process callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Avoid busy-waiting aka dont stress the pc

        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            std::cout << "Unable to enable offboard mode within the timeout period." << std::endl;
            std::cout << "Do you want to continue in offline mode? (y/n): ";
            std::string input;
            std::getline(std::cin, input);

            if (input == "y")
            {
                RCLCPP_WARN(node_->get_logger(), "Continuing in offline mode.");
                return; // Exit the function and continue in offline mode
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Exiting program as offboard mode could not be enabled.");
                rclcpp::shutdown();
                exit(1); // Exit the program
            }
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Offboard mode enabled!");
}

// The drone needs constant reassurance of which inputs it should take serious
// so we need to send it a command to change the offboard control mode often
void Initialize::publishOffboardControlMode()
{
    if (!can_publish_.load())
    {
        RCLCPP_WARN(node_->get_logger(), "Publishing is disabled. Offboard control mode not sent.");
        return;
    }

    px4_msgs::msg::OffboardControlMode msg{}; // Declare and initialize the message format

    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects timestamp in microseconds
    msg.position = false;                                       // Disable position control
    msg.velocity = false;                                       // Disable velocity control
    msg.acceleration = false;                                   // Disable acceleration control
    msg.attitude = true;                                        // Enable attitude control
    msg.body_rate = false;                                      // Disable body rate control
    msg.thrust_and_torque = false;                              // Disable thrust control
    msg.direct_actuator = false;                                // Disable direct actuator control

    ros_offboard_control_mode_pub_->publish(msg);
}

// Because of both arm and the offboardcontrolmode should be sent often we put them in a function for easier use
void Initialize::turn_on_drone()
{
    publishOffboardControlMode();
    arm();
}

void Initialize::disablePublishing()
{
    can_publish_.store(false);
    RCLCPP_WARN(node_->get_logger(), "Publishing to ROS topics has been disabled for safety.");
}