# 이 파일은 실제 실행되는 노드가 아닌, Robot Control Service의
# 인터페이스 구조를 정의하기 위한 예시 또는 스텁(stub) 코드로 보입니다.
# 프로젝트의 코딩 컨벤션에 맞게 수정합니다.

from geometry_msgs.msg import PoseStamped

class RobotControlServiceStub:
    """RCS의 인터페이스를 정의하는 스텁(Stub) 클래스입니다."""

    class Goal:
        def __init__(self):
            self.pose = PoseStamped()
            self.pose.pose.position.x = 0.0
            self.pose.pose.position.y = 0.0

    class Result: pass
    class Feedback: pass
