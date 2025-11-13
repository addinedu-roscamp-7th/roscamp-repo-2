from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
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
            description='시뮬레이션 사용 여부',
        ),
        Node(
            package='javis_ais',
            executable='aruco_detect',
            name='aruco_docking_detect',
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