from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory('punyo'),
        'config',
        'bubble.yaml'
    )

    return LaunchDescription([
        Node(
            package='punyo',
            executable='bubble_image_subscriber',
            name='image_subscriber_node',
            output='screen',
            emulate_tty=True,
            parameters=[node_params]
            # parameters=[
            #     # {'rgb_topic_parameter': '/bubble_1/color/image_rect_raw'},
            #     # {'depth_topic_parameter': '/bubble_1/depth/image_rect_raw'}
            # ]
        )
    ])
