# roscamp-repo-2
ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프 2팀 저장소. 자비스 도서관


---

###  프로젝트 규칙
1. dev에서 개발하기
2. ros2ws 루트 폴더에서 colcon build
3. 동영상 이미지 (readme 에서 사용하는 asset은 제외) 지양
4. 폴더 구조대로 개발

---

###  폴더 구조

    gui 
    └── src/
        ├── cafe_order/
        ├── information_desk/
        └── admin/
    service
    └── src/
        ├── llm_service/
        ├── app_service/
    ros2_ws/
    └── src/
        ├── javis_rcs/
        ├── javis_dmc/
        ├── javis_dac/
        ├── javis_ddc/
        ├── javis_dis/
        ├── javis_kc/
        ├── javis_kis/
        └── javis_interfaces/
        ├── msg/
        ├── srv/
        └── action/
    firware
    └── src/
        ├── authentication_controller/
        └── payment_controller/
    readme.md
    .gitignore
    doc/
    ├── develop.md/
    └── .../
    asset/
    ├── image/
    └── video/

---

### commit 규칙


#### 1. 기본 형식

       
```
 <type>(<scope>): <subject>
    예시: feat(dmc): 도서 픽업 기능 추가
```

#### 2. Type (커밋 종류)

```
    Type설명
        feat 새 기능
        fix 버그 수정
        refactor 코드 개선 (기능 변화 없음)
        docs 문서 수정
        est 테스트 추가/수정
        chore 빌드, 설정 변경
        style 코드 포맷팅
```


#### 3. Scope (영향 범위)


```
    GUI
    gui/cafe - 카페 주문 GUI
    gui/info-desk - 안내 데스크 GUI
    gui/admin - 관리자 GUI

    Service
    service/llm - LLM 서비스
    service/app - 애플리케이션 서비스

    ROS2
    rcs - Robot Control Service
    dmc - Dobby Main Controller
    dac - Dobby Arm Controller
    ddc - Dobby Drive Controller
    dis - Dobby Image Service
    kc - Kreacher Controller
    kis - Kreacher Image Service
    interfaces - 메시지/액션 정의

    Firmware
    fw/auth - 인증 컨트롤러
    fw/payment - 결제 컨트롤러

    기타
    docs - 문서
    asset - 이미지/비디오

    ~ - 전체 프로젝트

```


---