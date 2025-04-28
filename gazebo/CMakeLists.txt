cmake_minimum_required(VERSION 3.5)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
project(drone_gazebo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add the plugin library
add_library(${PROJECT_NAME} SHARED src/plugins/gazebo_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs gazebo_ros)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# Install the plugin
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/gazebo/plugins
)

# Install world files
install(
  DIRECTORY src/worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install model files to Gazebo's model path
install(
  DIRECTORY model
  DESTINATION share/gazebo/models/drone
)

# Install launch files
install(
  DIRECTORY src/launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()