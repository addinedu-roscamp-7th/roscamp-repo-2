from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    OrchestratorNode와 2대의 MockDmcNode를 함께 실행하는 통합 LaunchDescription.
    javis_dmc 패키지 없이 javis_rcs만으로 작업 할당 테스트를 할 수 있습니다.
    """

    # 관리할 로봇 목록
    robot_namespaces = ['dobby1', 'dobby2']

    # OrchestratorNode 실행 설정
    orchestrator_node = Node(
        package='javis_rcs',
        executable='robot_control_service.py',
        name='robot_control_service',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_namespaces
        }]
    )

    # 각 로봇에 대한 MockDmcNode 실행 설정
    mock_rcs_nodes = [
        Node(
            package='javis_rcs',
            executable='mock_rcs_node.py',
            name=f'mock_rcs_{ns}',
            namespace=ns, # 노드 자체의 네임스페이스
            output='screen',
            parameters=[{'robot_namespace': ns}] # 스크립트 내부에서 사용할 파라미터
        ) for ns in robot_namespaces
    ]

    return LaunchDescription([orchestrator_node] + mock_rcs_nodes)
