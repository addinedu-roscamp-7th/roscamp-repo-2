from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    yolo_detector 노드와 rotation 노드를 동시에 실행하는 런치 파일
    """
    
    # yolo_detector 노드를 실행하기 위한 설정
    yolo_node = Node(
        package='javis_kis',         # 패키지 이름
        executable='yolo_detector',  # setup.py에 정의된 실행 파일 이름
        name='yolo_detector_node',   # 노드에 부여할 이름
        output='screen'              # 노드의 로그(print, logger 등)를 터미널에 출력
    )
    
    # rotation 노드를 실행하기 위한 설정
    rotation_node = Node(
        package='javis_kis',         # 패키지 이름
        executable='rotation',       # setup.py에 정의된 실행 파일 이름
        name='rotation_node',        # 노드에 부여할 이름
        output='screen'
    )
    
    # 실행할 노드 목록을 포함하는 LaunchDescription 객체를 반환
    return LaunchDescription([
        yolo_node,
        rotation_node,
    ])