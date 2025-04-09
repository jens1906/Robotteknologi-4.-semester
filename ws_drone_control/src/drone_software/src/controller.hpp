#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

class Controller {
public:
    Controller(rclcpp::Node::SharedPtr node);
    void initialize(rclcpp::Node::SharedPtr node);
    void publishVehicleAttitudeSetpoint(float roll, float pitch, float yaw, float thrust);

private:
    rclcpp::Node::SharedPtr node_;  // Store the shared node
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr ros_attitude_setpoint_pub_;
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    std::array<float, 4> rpyToQuaternion(float roll, float pitch, float yaw);
};

#endif  