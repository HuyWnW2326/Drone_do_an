from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '921600'],
            output='screen'
        ),
        Node(
            package='drone_pkg',
            executable='SMC_Circle_node',
            name='SMC_Circle',
            output='screen'
        )
    ])
