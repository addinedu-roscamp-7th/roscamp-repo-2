# Mock 노드의 현재 동작 모드 발행 메시지

std_msgs/Header header
string node_name   # Mock 노드 이름 (예: "mock_dvs_tracking_status")
string mode        # 동작 모드 ("active", "error", "on", "off")
