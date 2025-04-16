cmake_minimum_required(VERSION 3.5)
project(drone_software)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(src)

# Add the executables
add_executable(offboard_control_node src/main.cpp src/initialize.cpp src/controller.cpp)
add_executable(motor_output_reader src/motor_output_reader.cpp) #Remove me if it doesn't work with motor stuff

# Link dependencies
ament_target_dependencies(offboard_control_node rclcpp px4_msgs std_msgs )
ament_target_dependencies(motor_output_reader rclcpp px4_msgs) #Remove me if it doesn't work with motor stuff

# Install the executables
install(TARGETS offboard_control_node
    DESTINATION lib/${PROJECT_NAME})
install(TARGETS motor_output_reader
    DESTINATION lib/${PROJECT_NAME}) #Remove me if it doesn't work with motor stuff

# Ament package macros
ament_package()