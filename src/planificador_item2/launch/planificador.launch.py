from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='planificador_item2', executable='planificador_moveit', output='screen')
    ])
