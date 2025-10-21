from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Hand-Eye Calibration 결과로 얻은 정적 TF 발행
    # 'base_link' -> 'camera_link' 관계
    # 아래 arguments의 값들을 실제 계산된 값으로 반드시 수정해야 합니다.
    # [x, y, z, qx, qy, qz, qw] 순서
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_broadcaster',
        arguments=['0.4', '-0.245', '0.08', '-1.5707', '3.1415', '1.5707', 'base_link', 'camera_link']
    )

    # 2. 직접 작성한 myCobot 동적 TF 발행 노드 실행
    # 'base_link' -> 'end_effector_link' 관계
    mycobot_tf_broadcaster_node = Node(
        package='javis_kc',  # 이 노드가 포함된 패키지 이름
        executable='mycobot_tf_broadcaster',
        name='mycobot_dynamic_tf_broadcaster'
    )
    
    return LaunchDescription([
        static_transform_publisher_node,
        mycobot_tf_broadcaster_node
    ])
