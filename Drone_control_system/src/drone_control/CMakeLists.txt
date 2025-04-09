cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the include directory
include_directories(include)

# Add the executable
add_executable(drone_controller src/drone_controller.cpp src/main.cpp)

# Link dependencies
ament_target_dependencies(drone_controller
  rclcpp
  std_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install the include directory (if you have header files)
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs geometry_msgs)

# Export the package
ament_package()