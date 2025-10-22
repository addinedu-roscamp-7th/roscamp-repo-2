DAC <-> DVS



By 김규환

1 min

See views

Add a reaction
From

To

프로토콜

인터페이스 항목

메시지 형식

 

Service
 

DAC

DVS

Service
도서 감지

dobby1/ai/detect_book dobby2/ai/detect_book



# DetectBook.srv
# Request (요청)
int32 book_id
---
# Response (응답)
bool detected
int32 book_id
geometry_msgs/Pose book_pose # 감지된 도서의 위치
float32 confidence # 감지 신뢰도
 

DAC

DVS

Service
보관함 상태 확인

dobby1/ai/check_storage_box dobby2/ai/check_storage_box



# CheckStorageBox.srv
# Request (요청)
int32 storage_box_id #확인할 보관함 ID
---
# Response (응답)
bool is_valid # 유효한 보관함인지 여부
int32 storage_box_id # 확인된 보관함 ID
geometry_msgs/Pose storage_box_pose  # 보관함 위치
bool is_empty # 비어있는지 확인
string status_message
 

DAC

DVS

Service
도서 정위치 확인

dobby1/ai/verify_book_position dobby2/ai/verify_book_position



# VerifyBookPosition.srv
# Request
string book_id  # 확인할 책 ID
---
# Response
bool is_correct_position  # 올바른 위치인지 여부
string book_id            # 확인된 책 ID
string error_type          # 오류 유형
string message             # 결과 메시지
 

DAC

DVS

Service
책장 식별

dobby1/ai/identify_bookshelf dobby2/ai/identify_bookshelf



IdentifyBookshelf.srv
# Request
# (요청 데이터 없음)
---
# Response
bool detected                         # 감지 성공 여부
string bookshelf_id                   # 식별된 책장 ID
geometry_msgs/Pose bookshelf_pose     # 책장 위치
geometry_msgs/Pose[] available_slots  # 책을 꽂을 수 있는 빈 슬롯 위치 목록
 

DAC

DVS

Service
쓰레기 감지

dobby1/ai/detect_trash dobby2/ai/detect_trash



DetectTrash.srv
# Request
geometry_msgs/Pose seat_location   # 확인할 좌석 위치
---
# Response
bool trash_found                   # 쓰레기 발견 여부
int32 trash_count                  # 발견된 쓰레기 수
string[] trash_types               # 쓰레기 종류 목록
geometry_msgs/Pose[] trash_poses   # 쓰레기 위치 목록
float32[] confidence_scores        # 감지 신뢰도 목록
 

 

 

 

 

 

 

 

 

 

 

 

 