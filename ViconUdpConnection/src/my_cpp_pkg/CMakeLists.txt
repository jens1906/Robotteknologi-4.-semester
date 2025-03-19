cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(receiver src/Reciever.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

install(TARGETS
  receiver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
