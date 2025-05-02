cmake_minimum_required(VERSION 3.8)
project(saf_workshop_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DebugVariables.msg"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install Python scripts
install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()