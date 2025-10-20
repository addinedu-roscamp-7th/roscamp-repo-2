from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    '''단일 도비 메인 컨트롤러를 실행하는 LaunchDescription 생성.'''

    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='dobby1',
            description='로봇 네임스페이스 지정',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이터 시간 사용 여부',
        ),
        Node(
            package='javis_dmc',
            executable='dmc_node',
            name='javis_dmc_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                {
                    'robot_namespace': robot_namespace,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
