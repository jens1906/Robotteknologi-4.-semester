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

    // Create the Vicon subscription ONCE here
    ros_vicon_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Vicon", qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 8) {
                std::unique_lock<std::mutex> lock(vicon_mutex_);
                // Save previous position and time
                std::array<float, 3> prev_pos = {vicon_position_[0], vicon_position_[1], vicon_position_[2]};
                rclcpp::Time prev_time = prev_vicon_time_;
                // Update position
                vicon_position_[0] = msg->data[2];
                vicon_position_[1] = msg->data[3];
                vicon_position_[2] = msg->data[4];
                // Compute velocity
                rclcpp::Time now = rclcpp::Clock().now();
                float dt = (now - prev_time).seconds();
                if (dt > 0.001f && prev_time.nanoseconds() != 0) {
                    vicon_velocity_[0] = (vicon_position_[0] - prev_pos[0]) / dt;
                    vicon_velocity_[1] = (vicon_position_[1] - prev_pos[1]) / dt;
                    vicon_velocity_[2] = (vicon_position_[2] - prev_pos[2]) / dt;
                }
                prev_vicon_time_ = now;
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
                std::cout << "ViconUpdate" << std::endl;
            }
        });
}

void Controller::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("offboard_control_node"), "Received VehicleAttitude message");
}

void Controller::publishVehicleAttitudeSetpoint(const std::array<float, 3>& xyz_error, float yaw) {
    float x_vel, y_vel;
    {
        std::lock_guard<std::mutex> lock(vicon_mutex_);
        // Transform global velocity to local frame using yaw if needed
        float yaw_radians = vicon_position_[5] * M_PI / 180.0f; // Convert degrees to radians
        x_vel = cos(yaw_radians) * vicon_velocity_[0] + sin(yaw_radians) * vicon_velocity_[1];
        y_vel = -sin(yaw_radians) * vicon_velocity_[0] + cos(yaw_radians) * vicon_velocity_[1];
    }
    auto roll_pitch = xyToRollPitch(xyz_error[0], xyz_error[1], x_vel, y_vel);
    float thrust = zToThrust(xyz_error[2]);
    yaw = 0.0f;

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects µs
    msg.q_d = rpyToQuaternion(roll_pitch[0], roll_pitch[1], yaw);
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust};
    // std::cout << "Publishing VehicleAttitudeSetpoint: roll=" << roll_pitch[0]
    // << ", pitch=" << roll_pitch[1] << ", thrust=" << thrust << std::endl;

    // --- Compute and log motor outputs (custom order) ---
    // M1 = Front right, M2 = behind Left, M3 = Front left, M4 = Behind right
    float roll = roll_pitch[0];
    float pitch = roll_pitch[1];
    float yaw_rate = 0.0f; // If you have yaw control, use it here

    float m1 = thrust + pitch - roll + yaw_rate; // Front right
    float m2 = thrust - pitch + roll + yaw_rate; // Behind left
    float m3 = thrust + pitch + roll - yaw_rate; // Front left
    float m4 = thrust - pitch - roll - yaw_rate; // Behind right

    m1 = std::clamp(m1, 0.0f, 1.0f);
    m2 = std::clamp(m2, 0.0f, 1.0f);
    m3 = std::clamp(m3, 0.0f, 1.0f);
    m4 = std::clamp(m4, 0.0f, 1.0f);

    // std::cout << "Motor outputs: M1=" << m1 << " (Front right), M2=" << m2
    //       << " (Behind left), M3=" << m3 << " (Front left), M4=" << m4
    //       << " (Behind right)" << std::endl;

    // ---------------------------------------------------

    ros_attitude_setpoint_pub_->publish(msg);
    // std::cout << "Published VehicleAttitudeSetpoint: thrust=" << thrust << std::endl;
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

std::array<float, 2> Controller::xyToRollPitch(float x_error, float y_error, float x_velocity, float y_velocity) {
    // Outer loop gains (position to velocity)
    float Kp_xy_outer = 0.111f;
    float Kd_xy_outer = 0.1804f;
    // Inner loop gains (velocity to attitude)
    float Kp_xy_inner = 0.1f;
    float Kd_xy_inner = 0.05f;

    static float prev_x_error = 0.0f;
    static float prev_y_error = 0.0f;
    static float prev_vx_error = 0.0f;
    static float prev_vy_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now();

    rclcpp::Time current_time = rclcpp::Clock().now();
    float dt = (current_time - prev_time).seconds();
    if (dt < 0.01f) { dt = 0.01f; }
    prev_time = current_time;

    // Outer loop: position error to velocity command
    float dx = (x_error - prev_x_error) / dt;
    float dy = (y_error - prev_y_error) / dt;
    float vx_cmd = Kp_xy_outer * x_error + Kd_xy_outer * dx;
    float vy_cmd = Kp_xy_outer * y_error + Kd_xy_outer * dy;

    //Error calculation
    float vx_error = vx_cmd - x_velocity;
    float vy_error = vy_cmd - y_velocity;

    // Inner loop: velocity command to roll/pitch
    float dx_inner = (vx_error - prev_vx_error) / dt;
    float dy_inner = (vy_error - prev_vy_error) / dt;
    float roll_desired  = -(Kp_xy_inner * vy_error + Kd_xy_inner * dy_inner);
    float pitch_desired = Kp_xy_inner * vx_error + Kd_xy_inner * dx_inner;

    float roll_desired_clamped = std::clamp(roll_desired, -0.2f, 0.2f);
    float pitch_desired_clamped = std::clamp(pitch_desired, -0.2f, 0.2f);

    prev_x_error = x_error;
    prev_y_error = y_error;
    prev_vx_error = vx_error;
    prev_vy_error = vy_error;

    std::cout << "Roll desired (lim): " << roll_desired_clamped << " (" << roll_desired << "), "
              << "Pitch desired (lim): " << pitch_desired_clamped << " (" << pitch_desired << ")" << std::endl; 
    return {roll_desired_clamped, pitch_desired_clamped};
}

float Controller::zToThrust(float z_error) {
    float Kp_z = 0.8173f;
    float Kd_z = 2.214f;

    static float prev_z_error = 0.0f;
    static rclcpp::Time prev_time = rclcpp::Clock().now();

    rclcpp::Time current_time = rclcpp::Clock().now();
    float dt = (current_time - prev_time).seconds();
    if (dt < 0.01f) {
        std::cout << "Invalid dt detected, using fallback value." << std::endl;
        dt = 0.01f;
    } // Avoid division by zero
    prev_time = current_time;

    float z_derivative = (z_error - prev_z_error) / dt;
    float thrust = (Kp_z * z_error + Kd_z * z_derivative);
    float thrust_clamped = std::clamp(thrust, 0.0f, 0.8f); // Clamp thrust to [0, 0.8]
    std::cout << "Thrust (lim): " << thrust_clamped << " (" << thrust << ")" << std::endl;

    prev_z_error = z_error;

    return thrust_clamped;
}

//This is not in use
void Controller::goalPosition(const std::array<float, 3>& goal_position) {
    // Wait for at least one Vicon update (optional: add a timeout)
    rclcpp::Time start_time = rclcpp::Clock().now();
    while (vicon_position_[0] == 0.0f && (rclcpp::Clock().now() - start_time).seconds() < 1.0) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "goalPosition called with goal: x=" << goal_position[0]
          << ", y=" << goal_position[1] << ", z=" << goal_position[2] << std::endl;

    float x_error_global = goal_position[0] - vicon_position_[0];
    float y_error_global = goal_position[1] - vicon_position_[1];
    float z_error = goal_position[2] - vicon_position_[2];

    float yaw_radians = vicon_position_[5] * M_PI / 180.0f; // Convert degrees to radians
    float x_error_local = cos(yaw_radians) * x_error_global + sin(yaw_radians) * y_error_global;
    float y_error_local = -sin(yaw_radians) * x_error_global + cos(yaw_radians) * y_error_global;

    std::cout << "Errors (global): x=" << x_error_global << ", y=" << y_error_global
          << ", z=" << z_error << std::endl;
    std::cout << "Errors (local): x=" << x_error_local << ", y=" << y_error_local << std::endl;

    publishVehicleAttitudeSetpoint({x_error_local, y_error_local, z_error}, 0.0f);
}
 

void Controller::startGoalPositionThread(const std::array<float, 3>& goal_position) {
    stop_thread_.store(false);
    goal_position_thread_ = std::thread([this, goal_position]() {
        std::cout << "Starting goalPosition thread." << std::endl;

        while (!stop_thread_.load()) {
            std::unique_lock<std::mutex> lock(vicon_mutex_);
            
            rclcpp::Time start_time = rclcpp::Clock().now();
            while (vicon_position_[0] == 0.0f && (rclcpp::Clock().now() - start_time).seconds() < 1.0) {
                rclcpp::spin_some(node_);

                //std::this_thread::sleep_for(std::chrono::milliseconds(9));
            }
            std::cout << "ViconUpdate" << std::endl;
            rclcpp::spin_some(node_);

            // Use the latest vicon_position_ directly
            std::cout << "Current Vicon position: x=" << vicon_position_[0]
            << ", y=" << vicon_position_[1] << ", z=" << vicon_position_[2] 
            << ", yaw(deg)=" << vicon_position_[5] << std::endl;
            
            float x_error_global = goal_position[0] - vicon_position_[0];
            float y_error_global = goal_position[1] - vicon_position_[1];
            float z_error = goal_position[2] - vicon_position_[2];

            // Convert yaw from degrees to radians for trigonometric functions
            float yaw_radians = vicon_position_[5] * M_PI / 180.0f;
            float x_error_local = cos(yaw_radians) * x_error_global + sin(yaw_radians) * y_error_global;
            float y_error_local = -sin(yaw_radians) * x_error_global + cos(yaw_radians) * y_error_global;

            std::cout << "Yaw (degrees): " << vicon_position_[5] << ", Yaw (radians): " << yaw_radians << std::endl;
            std::cout << "Errors (global): x=" << x_error_global << ", y=" << y_error_global
            << ", z=" << z_error << std::endl;
            std::cout << "Errors (local): x=" << x_error_local << ", y=" << y_error_local << std::endl;

            publishVehicleAttitudeSetpoint({x_error_local, y_error_local, z_error}, 0.0f);

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
    // print vicon velocity
    std::cout << "Vicon Velocity: vx=" << vicon_velocity_[0] << ", vy=" << vicon_velocity_[1]
          << ", vz=" << vicon_velocity_[2] << std::endl;
    auto roll_pitch = xyToRollPitch(xyz_error[0], xyz_error[1], vicon_velocity_[0], vicon_velocity_[1]);
    float thrust = zToThrust(xyz_error[2]);

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

            float thrust = zToThrust(z_error);  // Calculate thrust using z control system
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
