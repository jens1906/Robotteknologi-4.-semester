#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>

// This code is a ROS2 node that subscribes to the actuator_outputs topic and prints the motor outputs to the console.
// It uses the px4_msgs package to define the message type for actuator outputs.
// The node is named "motor_output_reader" and it subscribes to the "/fmu/out/actuator_outputs" topic.
// The callback function motorOutputCallback is called whenever a new message is received on the topic.
// Remove if not needed

class MotorOutputReader : public rclcpp::Node {
public:
    MotorOutputReader() : Node("motor_output_reader") {
        // Subscribe to the actuator_outputs topic
        motor_output_sub_ = this->create_subscription<px4_msgs::msg::ActuatorOutputs>(
            "/fmu/out/actuator_outputs", 10,
            std::bind(&MotorOutputReader::motorOutputCallback, this, std::placeholders::_1));
    }

private:
    void motorOutputCallback(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Timestamp: %ld", msg->timestamp);

        for (size_t i = 0; i < msg->control.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Motor %zu: %.4f", i, msg->control[i]);
        }
    }

    rclcpp::Subscription<px4_msgs::msg::ActuatorOutputs>::SharedPtr motor_output_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorOutputReader>());
    rclcpp::shutdown();
    return 0;
}