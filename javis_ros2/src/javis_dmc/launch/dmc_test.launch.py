from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    '''테스트용 도비 메인 컨트롤러 실행.'''

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    launch_test_gui = LaunchConfiguration('launch_test_gui')
    use_mock_interfaces = LaunchConfiguration('use_mock_interfaces')
    mock_mode_drive = LaunchConfiguration('mock_mode_drive')
    mock_mode_arm = LaunchConfiguration('mock_mode_arm')
    mock_mode_ai = LaunchConfiguration('mock_mode_ai')
    mock_mode_gui = LaunchConfiguration('mock_mode_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='dobby1',
            description='테스트용 로봇 네임스페이스',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이터 시간 사용 여부',
        ),
        DeclareLaunchArgument(
            'use_mock_interfaces',
            default_value='true',
            description='Mock 인터페이스 사용 여부',
        ),
        DeclareLaunchArgument(
            'launch_test_gui',
            default_value='true',
            description='Test GUI 노드 실행 여부',
        ),
        DeclareLaunchArgument(
            'mock_mode_drive',
            default_value='-1',
            description='드라이브 인터페이스 Mock 모드 (-1:auto, 0:real, 1:mock)',
        ),
        DeclareLaunchArgument(
            'mock_mode_arm',
            default_value='-1',
            description='Arm 인터페이스 Mock 모드 (-1:auto, 0:real, 1:mock)',
        ),
        DeclareLaunchArgument(
            'mock_mode_ai',
            default_value='0',
            description='AI 인터페이스 Mock 모드 (-1:auto, 0:real, 1:mock)',
        ),
        DeclareLaunchArgument(
            'mock_mode_gui',
            default_value='1',
            description='GUI 인터페이스 Mock 모드 (-1:auto, 0:real, 1:mock)',
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
                    'use_mock_interfaces': use_mock_interfaces,
                    'mock_mode_drive': mock_mode_drive,
                    'mock_mode_arm': mock_mode_arm,
                    'mock_mode_ai': mock_mode_ai,
                    'mock_mode_gui': mock_mode_gui,
                },
            ],
        ),
        GroupAction(
            actions=[
                Node(
                    package='javis_dmc',
                    executable='test_gui_node',
                    name='test_gui_node',
                    namespace='',
                    output='screen',
                    parameters=[
                        {
                            'robot_namespace': robot_namespace,
                        },
                    ],
                ),
            ],
            condition=IfCondition(launch_test_gui),
        ),
    ])
