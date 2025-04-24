import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the model directory where your URDF or SDF files are located
    model_path = os.path.join(FindPackageShare('my_drone'), 'model')

    return LaunchDescription([
        # Launch Gazebo simulation with your model
        Node(
            package="gazebo_ros",  # Gazebo package
            executable="gazebo",  # Gazebo executable
            name="gazebo",
            output="screen",
            parameters=[{'use_sim_time': True}],  # Use simulation time
        ),
        # Optionally, add any control nodes for the drone here
        Node(
            package="your_control_package",  # Replace with your control package name
            executable="your_control_node",  # Replace with the actual executable name
            name="drone_control",
            output="screen",
            parameters=[{'use_sim_time': True}],
        ),
    ])
