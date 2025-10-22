from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='javis_dac',
            executable='dac_all_nodes',  # 단일 실행
            name='dac_all_nodes',
            output='screen'
        ),
    ])
