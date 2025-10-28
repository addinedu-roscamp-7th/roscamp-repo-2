from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    OrchestratorNode(robot_control_service)를 실행하는 LaunchDescription 생성.
    이 노드는 여러 로봇의 상태를 모니터링하고 작업을 할당하는 중앙 관제 역할을 합니다.
    """

    # 관리할 로봇의 네임스페이스 목록입니다.
    # robot_control_service.py 파일의 기본값과 동일하게 설정합니다.
    # 실행 시 변경 가능: ros2 launch javis_rcs rcs.launch.py robot_namespaces:="['dobby1', 'dobby2', 'dobby3']"
    robot_namespaces_list = ['dobby1', 'dobby2', 'kreacher']

    return LaunchDescription([
        Node(
            package='javis_rcs',
            executable='robot_control_service',
            name='robot_control_service',
            output='screen',
            parameters=[{
                'robot_namespaces': robot_namespaces_list
            }]
        ),
       
    ])