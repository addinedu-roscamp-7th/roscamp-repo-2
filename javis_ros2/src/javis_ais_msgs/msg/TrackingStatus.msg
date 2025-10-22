std_msgs/Header header             # 메시지 헤더
string tracking_id                 # 추적 중인 ID
bool person_detected               # 사람 감지 여부
geometry_msgs/Pose person_pose     # 사람의 위치
float32 distance_to_person         # 사람과의 거리
float32 confidence                 # 감지 신뢰도
bool is_lost                       # 추적 대상을 놓쳤는지 여부
float32 time_since_last_seen       # 마지막으로 감지된 후 경과 시간
