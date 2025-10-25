'''전체 테스트 환경 Launch 파일 - DMC + Mock Bridge + Mock RCS.'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    '''Launch 설정 생성.'''
    # Launch Arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='dobby1',
        description='로봇 네임스페이스',
    )

    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='true',
        description='Mock Bridge 사용 여부 (true/false)',
    )

    # Launch Configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_mock = LaunchConfiguration('use_mock')

    # DMC Node
    dmc_node = Node(
        package='javis_dmc',
        executable='dmc_node',
        namespace=robot_namespace,
        name='dmc_node',
        parameters=[
            {'robot_namespace': robot_namespace},
        ],
        output='screen',
    )

    # Mock Bridge Node
    mock_bridge_node = Node(
        package='javis_dmc_test',
        executable='mock_bridge_node',
        name='mock_bridge_node',
        parameters=[
            {'robot_namespace': robot_namespace},
        ],
        output='screen',
        condition=lambda context: context.launch_configurations['use_mock'] == 'true',
    )

    # Mock RCS Node
    mock_rcs_node = Node(
        package='javis_dmc_test',
        executable='mock_rcs_node',
        name='mock_rcs_node',
        parameters=[
            {'robot_namespace': robot_namespace},
        ],
        output='screen',
    )

    # Test GUI (Simple)
    test_gui_node = Node(
        package='javis_dmc_test',
        executable='test_gui_simple',
        name='test_gui_simple',
        output='screen',
    )

    return LaunchDescription([
        robot_namespace_arg,
        use_mock_arg,
        dmc_node,
        mock_bridge_node,
        mock_rcs_node,
        test_gui_node,
    ])
