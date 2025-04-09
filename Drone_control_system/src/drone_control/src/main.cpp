#include <rclcpp/rclcpp.hpp>
#include "drone_control/drone_controller.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto drone_controller = std::make_shared<drone_control::DroneController>();
    rclcpp::spin(drone_controller);
    rclcpp::shutdown();
    return 0;
}