from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    '''테스트용 도비 메인 컨트롤러 실행.'''

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='시뮬레이터 시간 사용 여부',
        ),
        Node(
            package='javis_dmc',
            executable='dmc_node',
            name='javis_dmc_node',
            namespace='dobby_test',
            output='screen',
            parameters=[
                {
                    'robot_namespace': 'dobby_test',
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
