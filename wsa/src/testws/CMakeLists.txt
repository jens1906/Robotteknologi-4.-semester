cmake_minimum_required(VERSION 3.5)
project(testws)

find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui 3DCore 3DExtras)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(
    include
    ${px4_msgs_INCLUDE_DIRS}
)

# Add the main executable
add_executable(main
    src/main.cpp
)
ament_target_dependencies(main rclcpp px4_msgs)

# Add the GUI executable
add_executable(gui
    src/gui.cpp
)
ament_target_dependencies(gui rclcpp px4_msgs)
target_link_libraries(gui Qt5::Core Qt5::Widgets Qt5::Gui Qt5::3DCore Qt5::3DExtras)

# Install executables
install(TARGETS
  main
  gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
