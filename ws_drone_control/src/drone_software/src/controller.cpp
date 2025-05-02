#include "controller.hpp"
#include <iomanip> // For std::setprecision and std::fixed

Controller::Controller(rclcpp::Node::SharedPtr node)
    : vicon_position_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      vicon_velocity_{0.0f, 0.0f, 0.0f},
      prev_vicon_time_(rclcpp::Time(0, 0)) { // Initialize to 0 seconds and 0 milliseconds
    node_ = node;
    std::cout << std::fixed << std::setprecision(2); // Set precision for output
}

void Controller::viconCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 8) {
        std::unique_lock<std::mutex> lock(vicon_mutex_);

        // Save previous time
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
            vicon_velocity_[0] = (vicon_position_[0] - prev_pos_[0]) / dt;
            vicon_velocity_[1] = (vicon_position_[1] - prev_pos_[1]) / dt;
            vicon_velocity_[2] = (vicon_position_[2] - prev_pos_[2]) / dt;
        } else {
            std::cerr << "Invalid dt detected: " << dt << " seconds. Skipping velocity update." << std::endl;
        }
        // Update previous position and time
        prev_pos_ = {vicon_position_[0], vicon_position_[1], vicon_position_[2]};
        prev_vicon_time_ = current_time;  // Update previous time
        vicon_dt_ = dt;  // Update the shared dt
        // std::cerr << "prev_time: " << prev_time.nanoseconds() << ", current_time: " << current_time.nanoseconds() << ", dt: " << dt << std::endl;

        // Update orientation
        vicon_position_[3] = msg->data[5];
        vicon_position_[4] = msg->data[6];
        vicon_position_[5] = msg->data[7];

        vicon_updated_ = true; // Set the update flag
        lock.unlock();
        vicon_update_cv_.notify_one(); // Notify the waiting thread

        //std::cout << "Vicon update: x=" << vicon_position_[0] << ", y=" << vicon_position_[1]
        //          << ", z=" << vicon_position_[2] << ", vx=" << vicon_velocity_[0]
        //          << ", vy=" << vicon_velocity_[1] << ", vz=" << vicon_velocity_[2] << std::endl;
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

    ros_debug_pub_ = node_->create_publisher<drone_software::msg::DebugVariables>("/debug_variables", 10); // Initialize debug publisher

    // Sleep for a short duration to allow the subscriber to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Calculate the initial yaw offset
    {
        std::unique_lock<std::mutex> lock(vicon_mutex_);
        float imu_yaw = atan2(2.0f * (vehicle_attitude_quaternion_[0] * vehicle_attitude_quaternion_[3] +
                                      vehicle_attitude_quaternion_[1] * vehicle_attitude_quaternion_[2]),
                              .0f - 2.0f * (vehicle_attitude_quaternion_[2] * vehicle_attitude_quaternion_[2] +
                                            vehicle_attitude_quaternion_[3] * vehicle_attitude_quaternion_[3]));
        float vicon_yaw = -vicon_position_[5]; // Assuming yaw is stored in vicon_position_[5]
        initial_yaw_offset_ = imu_yaw - vicon_yaw;
        std::cout << "Initial Yaw Offset: " << initial_yaw_offset_ << std::endl;
        }
}

void Controller::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    vehicle_attitude_quaternion_ = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
}

void Controller::publishVehicleAttitudeSetpoint(float roll, float pitch, float thrust, float yaw) {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000; // PX4 expects µs
    msg.q_d = rpyToQuaternion(roll, pitch, yaw); // Convert roll, pitch, yaw to quaternion
    msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust}; // Apply thrust in the z direction

    // Publish the message
    ros_attitude_setpoint_pub_->publish(msg);

    // Log the published values
    /*
    std::cout << "Published VehicleAttitudeSetpoint: roll=" << roll
              << ", pitch=" << pitch << ", yaw=" << yaw
              << ", thrust=" << thrust << std::endl;*/
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
    //PD controller for XY or Roll and Pitch
    float Kp_xy_outer = 0.9804f; //0.111f; //Måske prøve 0.9804f   ?
    float Kd_xy_outer = 0.6128f; //0.1804f; //Måske prøve 0.6128f   ?
    float Kp_xy_inner = 0.2788f; //0.1f; //Måske prøve 0.2788f   ?
    float Kd_xy_inner = 0.0523f; //0.05f; //Måske prøve 0.0523f   ?

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

    //std::cout << "roll_desired: " << roll_desired << ", pitch_desired: " << pitch_desired << std::endl;

    float roll_desired_clamped = std::clamp(roll_desired, -0.2f, 0.2f);
    float pitch_desired_clamped = std::clamp(pitch_desired, -0.2f, 0.2f);

    prev_x_error = x_error;
    prev_y_error = y_error;
    prev_vx_error = vx_error;
    prev_vy_error = vy_error;

    return {roll_desired_clamped, -pitch_desired_clamped}; // Added a minus (-) to the pitch desired to mitigate the drones coordinate system (North, east, down)
}

float Controller::zToThrust(float z_error, float dt) {
    //PD controller for Z or Thrust
    float g_compensation = 0.5; // Gravity compensation to make it hover 68.4% to hover
    float Kp_z = 0.20430f; //Perhaps.8173f (Settling time 10s)
    float Kd_z = 1.107f; //Perhaps 2.214f (Settling time 10s)

    static float prev_z_error = 0.0f;

    float z_derivative = (z_error - prev_z_error) / dt;
    float thrust = Kp_z * z_error + Kd_z * z_derivative + g_compensation;
    float thrust_clamped = std::clamp(thrust, 0.0f, 1.0f); // Clamp thrust to [0, 1.0]
    //std::cout << "Thrust (lim): " << thrust_clamped << " (" << thrust << ")" << std::endl;

    prev_z_error = z_error;

    return thrust_clamped;
}

void Controller::startGoalPositionThread() {
    stop_thread_.store(false);
    goal_position_thread_ = std::thread([this]() { // Remove goal_position from the capture list
        //std::cout << "Starting goalPosition thread." << std::endl;

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
            float x_error_global = this->goal_position[0] - l_vicon_position[0]; // Access goal_position via this
            float y_error_global = this->goal_position[1] - l_vicon_position[1];
            float z_error = this->goal_position[2] - l_vicon_position[2];

            // Calculate drone's local errors and velocity
            float yaw_radians = l_vicon_position[5];
            float x_error_local = cos(yaw_radians) * x_error_global + sin(yaw_radians) * y_error_global;
            float y_error_local = -sin(yaw_radians) * x_error_global + cos(yaw_radians) * y_error_global;

            float x_velocity_local = cos(yaw_radians) * l_vicon_velocity[0] + sin(yaw_radians) * l_vicon_velocity[1];
            float y_velocity_local = -sin(yaw_radians) * l_vicon_velocity[0] + cos(yaw_radians) * l_vicon_velocity[1];

            /*std::cout << "local errors: x=" << x_error_local
                      << ", y=" << y_error_local
                      << ", z=" << z_error
                      << ", vx=" << x_velocity_local
                      << ", vy=" << y_velocity_local
                      << ", dt=" << l_vicon_dt
                      << std::endl;*/

            // Extract the first three elements of l_vicon_position for position
            std::array<float, 3> position = {l_vicon_position[0], l_vicon_position[1], l_vicon_position[2]};

            auto roll_pitch = xyToRollPitch(x_error_local, y_error_local, x_velocity_local, y_velocity_local, l_vicon_dt);
            float thrust = zToThrust(z_error, l_vicon_dt);

            // Convert the input Vicon yaw to the drone's local frame
            float desired_yaw_ned = -goal_yaw - initial_yaw_offset_;

            // Normalize the yaw to the range [-π, π]
            while (desired_yaw_ned > M_PI) desired_yaw_ned -= 2.0f * M_PI;
            while (desired_yaw_ned < -M_PI) desired_yaw_ned += 2.0f * M_PI;

            // Publish the calculated setpoint
            publishVehicleAttitudeSetpoint(roll_pitch[0], roll_pitch[1], thrust, desired_yaw_ned);

            // Publish debug variables
            publishDebugVariables(position, l_vicon_velocity, {x_error_local, y_error_local, z_error}, thrust, {goal_position[0], goal_position[1], goal_position[2]});

            //if (std::abs(x_error_local) < 0.01f && std::abs(y_error_local) < 0.01f && std::abs(z_error) < 0.01f) {
            //    std::cout << "Goal position reached." << std::endl;
            //    break;
            //}
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
        // If the thread is already running, just update the target and max thrust and return
        return;
    }

    stop_z_control_.store(false); // Reset the stop flag
    z_control_thread_ = std::thread([this]() {
        std::cout << "Starting zControlMode thread." << std::endl;

        while (!stop_z_control_.load()) {
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
            
            float current_z = l_vicon_position[2];  // Get current z position from Vicon

            float target_z = target_z_.load();  // Load the current target z position
            float z_error = target_z - current_z;

            float thrust = zToThrust(z_error, l_vicon_dt);  // Calculate thrust using z control system
            thrust = std::clamp(thrust, 0.0f, max_z_thrust_.load());  // Clamp thrust to the current max thrust

            px4_msgs::msg::VehicleAttitudeSetpoint msg{};
            msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;  // PX4 expects µs
            msg.q_d = {1.0f, 0.0f, 0.0f, 0.0f};  // Quaternion for no rotation
            msg.thrust_body = std::array<float, 3>{0.0f, 0.0f, -thrust};  // Apply thrust in the z direction
            
            // Print Vicon position and z thrust
            std::cout << "Vicon Position: x=" << l_vicon_position[0] << ", y=" << l_vicon_position[1]
                      << ", z=" << current_z << ", thrust=" << thrust << std::endl;

            ros_attitude_setpoint_pub_->publish(msg);

            std::cout << "Published zcon setpoint: target_z=" << target_z
                      << ", current_z=" << current_z
                      << ", thrust=" << thrust << std::endl;

            //std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Adjust loop frequency as needed
        }

        std::cout << "Exiting zControlMode thread." << std::endl;
    });
}

// Stop the zControlMode thread
void Controller::stopZControlMode() {
    if (z_control_thread_.joinable()) {
        stop_z_control_.store(true);  // Signal the thread to stop
        z_control_thread_.join();  // Wait for the thread to finish
    }
}

void Controller::publishDebugVariables(const std::array<float, 3>& position, const std::array<float, 3>& velocity, const std::array<float, 3>& errors, float thrust, const std::array<float, 3>& goal_position) {
    //std_msgs::msg::Float64MultiArray debug_msg;
    //debug_msg.data = {position[0], position[1], position[2], velocity[0], velocity[1], velocity[2], errors[0], errors[1], errors[2], thrust, goal_position[0], goal_position[1], goal_position[2], goal_yaw};
    drone_software::msg::DebugVariables debug_msg;
    debug_msg.x = position[0];
    debug_msg.y = position[1];
    debug_msg.z = position[2];
    debug_msg.vx = velocity[0];
    debug_msg.vy = velocity[1];
    debug_msg.vz = velocity[2];
    debug_msg.ex = errors[0];
    debug_msg.ey = errors[1];
    debug_msg.ez = errors[2];
    debug_msg.thrust = thrust;
    debug_msg.goal_x = goal_position[0];
    debug_msg.goal_y = goal_position[1];
    debug_msg.goal_z = goal_position[2];
    debug_msg.goal_yaw = goal_yaw;
    ros_debug_pub_->publish(debug_msg);
}
