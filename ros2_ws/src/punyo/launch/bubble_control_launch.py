from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='punyo',
            executable='bubble_control',
            name='rs_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pressure_parameter': '/bubble_BCAB9A5050573738352E3120FF07282C/pressure'}
            ]
        )
    ])