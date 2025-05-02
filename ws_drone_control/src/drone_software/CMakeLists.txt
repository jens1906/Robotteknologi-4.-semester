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
find_package(rosidl_default_generators REQUIRED)

# Generate the custom message interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DebugVariables.msg"
)

# Include directories
include_directories(src)

# Add the executables
add_executable(offboard_control_node src/main.cpp src/initialize.cpp src/controller.cpp)

# Link dependencies and generated messages
rosidl_target_interfaces(offboard_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(offboard_control_node 
  rclcpp 
  px4_msgs 
  std_msgs
)

# Install the executables
install(TARGETS offboard_control_node
    DESTINATION lib/${PROJECT_NAME})

# Ament package macros
ament_package()