from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_control',
            executable='drone_controller',
            name='drone_controller_node',
            output='screen',
            parameters=[
                {'param_name': 'param_value'},  # Replace with actual parameters as needed
            ],
            remappings=[
                ('/input_topic', '/drone/input'),  # Replace with actual topic names
                ('/output_topic', '/drone/output'),
            ],
        ),
    ])