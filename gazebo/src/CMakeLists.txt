cmake_minimum_required(VERSION 3.5)
project(my_gazebo_plugin)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
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
add_library(${PROJECT_NAME} SHARED src/my_gazebo_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# Install the plugin
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install other files (e.g., world files, models)
install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()