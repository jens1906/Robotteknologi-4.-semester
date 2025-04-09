# Autonomous Drone Control System

This project implements an autonomous drone control system using ROS2. The system subscribes to various topics related to the drone's position, orientation, speed, and other relevant data to control its flight behavior.

## Project Structure

```
autonomous_drone_control
├── src
│   ├── drone_control
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include
│   │   │   └── drone_control
│   │   │       └── drone_controller.hpp
│   │   ├── src
│   │   │   ├── drone_controller.cpp
│   │   │   └── main.cpp
│   │   └── launch
│   │       └── drone_control.launch.py
├── colcon.meta
└── README.md
```

## Setup Instructions

1. **Install ROS2**: Ensure that you have ROS2 installed on your Ubuntu 22.04.5 subsystem. Follow the official ROS2 installation guide for your version.

2. **Create the Workspace**:
   ```bash
   mkdir -p ~/autonomous_drone_control/src
   cd ~/autonomous_drone_control/src
   ```

3. **Clone the Repository**: If this project is hosted on a version control system, clone it into the `src` directory.

4. **Build the Project**:
   ```bash
   cd ~/autonomous_drone_control
   colcon build
   ```

5. **Source the Setup File**:
   ```bash
   source install/setup.bash
   ```

## Usage

To launch the drone control system, use the following command:

```bash
ros2 launch drone_control drone_control.launch.py
```

## Additional Information

- The `drone_controller.hpp` file contains the declaration of the `DroneController` class, which is responsible for managing the drone's state based on incoming messages.
- The `drone_controller.cpp` file implements the logic for processing the subscribed messages.
- The `main.cpp` file serves as the entry point for the application, initializing the ROS2 node and starting the control system.

For any issues or contributions, please refer to the project's repository or contact the maintainers.