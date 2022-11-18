from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='punyo',
            executable='bubble_image_subscriber',
            name='pf_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'topic_parameter': '/royale_camera/gray_image'}
            ]
        )
    ])