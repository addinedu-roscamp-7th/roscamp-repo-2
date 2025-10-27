from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    '''DMC Mock + Test GUI + Status GUI를 함께 실행한다.'''

    package_share = get_package_share_directory('javis_dmc_test')
    dmc_mock_full = os.path.join(package_share, 'launch', 'dmc_mock_full.launch.py')

    mock_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(dmc_mock_full))

    test_gui_node = Node(
        package='javis_dmc_test',
        executable='test_gui',
        name='test_gui',
        output='screen',
    )

    status_gui_node = Node(
        package='javis_dmc_test',
        executable='status_gui_node',
        name='status_gui_node',
        output='screen',
    )

    return LaunchDescription([
        mock_launch,
        test_gui_node,
        status_gui_node,
    ])
