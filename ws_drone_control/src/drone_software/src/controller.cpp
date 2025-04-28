#include "controller.hpp"
#include <iomanip> // For std::setprecision and std::fixed

Controller::Controller(rclcpp::Node::SharedPtr node) 
    : vicon_position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      vicon_velocity_{0.0f, 0.0f, 0.0f},
      prev_vicon_time_{0} {
    node_ = node;

    // Set global precision for std::cout
    std::cout << std::fixed << std::setprecision(2);
}

// Getter for vicon_position_
std::array<float, 6> Controller::getViconPosition() const {
    std::lock_guard<std::mutex> lock(vicon_mutex_); // Ensure thread-safe access
    return vicon_position_;
}

// Getter for vicon_updated_
bool Controller::isViconUpdated() const {
    std::lock_guard<std::mutex> lock(vicon_mutex_); // Ensure thread-safe access
    return vicon_updated_;
}

// Setter for vicon_updated_
void Controller::resetViconUpdated() {
    std::lock_guard<std::mutex> lock(vicon_mutex_); // Ensure thread-safe access
    vicon_updated_ = false;
}

void Controller::viconCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 8) {
        std::unique_lock<std::mutex> lock(vicon_mutex_);

        // Save previous position and time
        std::array<float, 3> prev_pos = {vicon_position_[0], vicon_position_[1], vicon_position_[2]};
        rclcpp::Time prev_time = prev_vicon_time_;

        // Extract the timestamp from the message (assuming it's in the first two elements)
        int current_seconds = static_cast<int>(msg->data[0]);  // Seconds part of the timestamp
        int current_milliseconds = static_cast<int>(msg->data[1]);  // Milliseconds part of the timestamp
        rclcpp::Time current_time(current_seconds, current_milliseconds * 1e6);  // Convert to rclcpp::Time

        // Update position
        vicon_position_[0] = msg->data[2];
        vicon_position_[1] = msg->data[3];
        vicon_position_[2] = msg->data[4];

        // Compute velocity and update dt
        float dt = (current_time - prev_time).seconds();  // Use the timestamp difference
        if (dt > 0.001f && prev_time.nanoseconds() != 0) {
            vicon_velocity_[0] = (vicon_position_[0] - prev_pos[0]) / dt;
            vicon_velocity_[1] = (vicon_position_[1] - prev_pos[1]) / dt;
            vicon_velocity_[2] = (vicon_position_[2] - prev_pos[2]) / dt;
            prev_vicon_time_ = current_time;  // Update previous time
            vicon_dt_ = dt;  // Update the shared dt
        } else {
            std::cerr << "Invalid dt detected: " << dt << " seconds. Skipping velocity update." << std::endl;
        }

        // Update orientation
        vicon_position_[3] = msg->data[5];
        vicon_position_[4] = msg->data[6];
        vicon_position_[5] = msg->data[7];

        vicon_updated_ = true; // Set the update flag
        lock.unlock();
        vicon_update_cv_.notify_one(); // Notify the waiting thread

        std::cout << "Vicon update: x=" << vicon_position_[0] << ", y=" << vicon_position_[1]
                  << ", z=" << vicon_position_[2] << ", vx=" << vicon_velocity_[0]
                  << ", vy=" << vicon_velocity_[1] << ", vz=" << vicon_velocity_[2] << std::endl;
    }
}

// Initialize ROS topics
void Controller::initialize(rclcpp::Node::SharedPtr node) {
    node_ = node;
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    ros_attitude_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/fmu/in/vehicle_attitude_setpoint", 10);

    ros_vehicle_attitude_sub_ = node_->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos,
        std::bind(&Controller::vehicleAttitudeCallback, this, std::placeholders::_1));

    // Create the Vicon subscription and use the viconCallback function
    ros_vicon_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Vicon", qos,
        std::bind(&Controller::viconCallback, this, std::placeholders::_1));
}

void Controller::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleAttitude message");
}

void Controller::publishVehicleAttitudeSetpoint(float roll, float pitch, float thrust, float yaw) {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects µs
    msg.q_d = rpyToQuaternion(roll, pitch, yaw); // Convert roll, pitch, yaw to quaternion
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust}; // Apply thrust in the z direction

    // Publish the message
    ros_attitude_setpoint_pub_->publish(msg);

    // Log the published values
    std::cout << "Published VehicleAttitudeSetpoint: roll=" << roll
              << ", pitch=" << pitch << ", yaw=" << yaw
              << ", thrust=" << thrust << std::endl;
}

std::array<float, 4> Controller::rpyToQuaternion(float roll, float pitch, float yaw) {
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    float w = cy * cp * cr + sy * sp * sr;
    float x = cy * cp * sr - sy * sp * cr;
    float y = sy * cp * sr + cy * sp * cr;
    float z = sy * cp * cr - cy * sp * sr;

    // std::cout << "Quaternion: w=" << w << ", x=" << x << ", y=" << y << ", z=" << z << std::endl;

    return {w, x, y, z};
}

std::array<float, 2> Controller::xyToRollPitch(float x_error, float y_error, float x_velocity, float y_velocity, float dt) {
    float Kp_xy_outer = 0.111f;
    float Kd_xy_outer = 0.1804f;
    float Kp_xy_inner = 0.1f;
    float Kd_xy_inner = 0.05f;

    static float prev_x_error = 0.0f;
    static float prev_y_error = 0.0f;
    static float prev_vx_error = 0.0f;
    static float prev_vy_error = 0.0f;

    // Outer loop: position error to velocity command
    float dx = (x_error - prev_x_error) / dt;
    float dy = (y_error - prev_y_error) / dt;
    float vx_cmd = Kp_xy_outer * x_error + Kd_xy_outer * dx;
    float vy_cmd = Kp_xy_outer * y_error + Kd_xy_outer * dy;

    // Error calculation
    float vx_error = vx_cmd - x_velocity;
    float vy_error = vy_cmd - y_velocity;

    // Inner loop: velocity command to roll/pitch
    float dx_inner = (vx_error - prev_vx_error) / dt;
    float dy_inner = (vy_error - prev_vy_error) / dt;
    float roll_desired = -(Kp_xy_inner * vy_error + Kd_xy_inner * dy_inner);
    float pitch_desired = Kp_xy_inner * vx_error + Kd_xy_inner * dx_inner;

    float roll_desired_clamped = std::clamp(roll_desired, -0.2f, 0.2f);
    float pitch_desired_clamped = std::clamp(pitch_desired, -0.2f, 0.2f);

    prev_x_error = x_error;
    prev_y_error = y_error;
    prev_vx_error = vx_error;
    prev_vy_error = vy_error;

    return {roll_desired_clamped, pitch_desired_clamped};
}

float Controller::zToThrust(float z_error, float dt) {
    float Kp_z = 0.8173f;
    float Kd_z = 2.214f;

    static float prev_z_error = 0.0f;

    float z_derivative = (z_error - prev_z_error) / dt;
    float thrust = (Kp_z * z_error + Kd_z * z_derivative);
    float thrust_clamped = std::clamp(thrust, 0.0f, 1.0f); // Clamp thrust to [0, 1.0]
    std::cout << "Thrust (lim): " << thrust_clamped << " (" << thrust << ")" << std::endl;

    prev_z_error = z_error;

    return thrust_clamped;
}

void Controller::startGoalPositionThread(const std::array<float, 3>& goal_position) {
    stop_thread_.store(false);
    goal_position_thread_ = std::thread([this, goal_position]() {
        std::cout << "Starting goalPosition thread." << std::endl;

        while (!stop_thread_.load()) {
            // Local copies of Vicon data
            std::array<float, 6> l_vicon_position;
            std::array<float, 3> l_vicon_velocity;
            float l_vicon_dt;

            // Lock the mutex only to copy the shared data
            {
                std::unique_lock<std::mutex> lock(vicon_mutex_);
                vicon_update_cv_.wait(lock, [this]() { return vicon_updated_ || stop_thread_.load(); });
                if (stop_thread_.load()) {
                    break; // Exit if the thread is stopped
                }
                vicon_updated_ = false; // Reset the update flag
                l_vicon_position = vicon_position_; // Copy the shared Vicon position
                l_vicon_velocity = vicon_velocity_; // Copy the shared Vicon velocity
                l_vicon_dt = vicon_dt_;             // Copy the shared dt
            }

            // Perform calculations using the local copies
            float x_error_global = goal_position[0] - l_vicon_position[0];
            float y_error_global = goal_position[1] - l_vicon_position[1];
            float z_error = goal_position[2] - l_vicon_position[2];

            // Convert yaw from degrees to radians for trigonometric functions
            float yaw_radians = l_vicon_position[5] * M_PI / 180.0f;
            float x_error_local = cos(yaw_radians) * x_error_global + sin(yaw_radians) * y_error_global;
            float y_error_local = -sin(yaw_radians) * x_error_global + cos(yaw_radians) * y_error_global;

            auto roll_pitch = xyToRollPitch(x_error_local, y_error_local, l_vicon_velocity[0], l_vicon_velocity[1], l_vicon_dt);
            float thrust = zToThrust(z_error, l_vicon_dt);

            // Publish the calculated setpoint
            publishVehicleAttitudeSetpoint(roll_pitch[0], roll_pitch[1], thrust, 0.0f);

            if (std::abs(x_error_local) < 0.01f && std::abs(y_error_local) < 0.01f && std::abs(z_error) < 0.01f) {
                std::cout << "Goal position reached." << std::endl;
                break;
            }
        }

        std::cout << "Exiting goalPosition thread." << std::endl;
    });
}

void Controller::stopGoalPositionThread() {
    if (goal_position_thread_.joinable()) {
        stop_thread_.store(true);  // Signal the thread to stop
        goal_position_thread_.join();  // Wait for the thread to finish
    }
}

void Controller::simulateDroneCommands(const std::array<float, 3>& xyz_error, float yaw) {
    // Print Vicon velocity
    std::cout << "Vicon Velocity: vx=" << vicon_velocity_[0] << ", vy=" << vicon_velocity_[1]
              << ", vz=" << vicon_velocity_[2] << std::endl;

    float dt = vicon_dt_; // Use the shared dt or a local copy

    // Calculate roll and pitch
    auto roll_pitch = xyToRollPitch(xyz_error[0], xyz_error[1], vicon_velocity_[0], vicon_velocity_[1], dt);

    // Calculate thrust
    float thrust = zToThrust(xyz_error[2], dt);

    // Simulate the quaternion calculation for roll, pitch, and yaw
    auto quaternion = rpyToQuaternion(roll_pitch[0], roll_pitch[1], yaw);

    // Log the simulated commands
    std::cout << "Simulated Commands: Roll=" << roll_pitch[0] << ", Pitch=" << roll_pitch[1]
              << ", Yaw=" << yaw << ", Thrust=" << thrust << std::endl;
    std::cout << "Simulated Quaternion: w=" << quaternion[0] << ", x=" << quaternion[1]
              << ", y=" << quaternion[2] << ", z=" << quaternion[3] << std::endl;
}

void Controller::manualMotorSet(float T) {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects µs
    msg.q_d = {1.0f, 0.0f, 0.0f, 0.0f}; // No rotation
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -T};
    ros_attitude_setpoint_pub_->publish(msg);
    std::cout << "Published manual motor setpoint: thrust=" << T << std::endl;
}

void Controller::zControlMode(float z_offset, float max_z_thrust) {
    target_z_.store(z_offset); // Update the target z position
    max_z_thrust_.store(max_z_thrust); // Update the maximum z thrust

    if (z_control_thread_.joinable()) {
        // If the thread is already running, just update the target and return
        return;
    }

    stop_z_control_.store(false); // Reset the stop flag
    z_control_thread_ = std::thread([this]() {
        std::cout << "Starting zControlMode thread." << std::endl;

        while (!stop_z_control_.load()) {
            std::unique_lock<std::mutex> lock(vicon_mutex_);
            float current_z = vicon_position_[2];  // Get current z position from Vicon
            lock.unlock();

            float target_z = target_z_.load();  // Load the current target z position
            float z_error = target_z - current_z;

            float dt = vicon_dt_; // Use the shared dt or a local copy
            float thrust = zToThrust(z_error, dt); // Pass the second argument
            thrust = std::clamp(thrust, 0.0f, max_z_thrust_.load());  // Clamp thrust to user-provided max value

            px4_msgs::msg::VehicleAttitudeSetpoint msg{};
            msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects µs
            msg.q_d = {1.0f, 0.0f, 0.0f, 0.0f};  // Quaternion for no rotation
            msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust};  // Apply thrust in the z direction
            
            // print vicon position and z thrust
            std::cout << "Vicon Position: x=" << vicon_position_[0] << ", y=" << vicon_position_[1]
                      << ", z=" << current_z << ", thrust=" << thrust << std::endl;

            // ros_attitude_setpoint_pub_->publish(msg);

            std::cout << "Published zcon setpoint: target_z=" << target_z
                      << ", current_z=" << current_z
                      << ", thrust=" << thrust << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Adjust loop frequency as needed
        }

        std::cout << "Exiting zControlMode thread." << std::endl;
    });
}

void Controller::stopZControlMode() {
    if (z_control_thread_.joinable()) {
        stop_z_control_.store(true);  // Signal the thread to stop
        z_control_thread_.join();  // Wait for the thread to finish
    }
}
