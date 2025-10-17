from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_dmc_node(namespace, use_sim_time):
    '''여러 도비 인스턴스 생성을 위한 노드 빌더.'''
    return Node(
        package='javis_dmc',
        executable='dmc_node',
        name='javis_dmc_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'robot_namespace': namespace,
                'use_sim_time': use_sim_time,
            },
        ],
    )


def generate_launch_description():
    '''두 대 이상의 도비 메인 컨트롤러를 실행한다.'''

    use_sim_time = LaunchConfiguration('use_sim_time')

    nodes = [
        _create_dmc_node('dobby1', use_sim_time),
        _create_dmc_node('dobby2', use_sim_time),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이터 시간 사용 여부',
        ),
        *nodes,
    ])
