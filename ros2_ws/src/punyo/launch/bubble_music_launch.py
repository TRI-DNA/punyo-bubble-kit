from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configurable_parameters = [
        {'name': 'pressure_parameter', 'default': '/bubble_sn/pressure', 'description': 'pressure topic'},
        ]

    def declare_configurable_parameters(parameters: List) -> List:
        return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description'])
                for param in parameters]

    def set_configurable_parameters(parameters):
        return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        Node(
            package='punyo',
            executable='bubble_music_subscriber',
            name='rs_node',
            output='screen',
            emulate_tty=True,
            parameters=[set_configurable_parameters(configurable_parameters)
                        ],
        )
    ])
