import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the package share directory
    package_share = get_package_share_directory('drone_gazebo')
    world_path = os.path.join(package_share, 'worlds', 'world_drone.world')
    model_path = os.path.join(package_share, 'models', 'drone')  # Adjust this path

    # Set the Gazebo model path
    os.environ['GAZEBO_MODEL_PATH'] = model_path

    # Launch Gazebo with the specified world
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',  # Full path to gzserver
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so', world_path],
        ),
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',  # Full path to gzclient
            name='gazebo_client',
            output='screen',
        ),
    ])
