cmake_minimum_required(VERSION 3.5)
project(drone_software)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)

# Add the executable
add_executable(offboard_mode src/offboard_mode.cpp)

# Link dependencies
ament_target_dependencies(offboard_mode rclcpp rclcpp_action px4_msgs)

# Install the executable
install(TARGETS offboard_mode
    DESTINATION lib/${PROJECT_NAME})

# Ament package macros
ament_package()