## 사용법 

```
# dmc 노드 실행
ros2 launch javis_dmc dmc_single.launch.py

# test mock 서버 실행 
ros2 launch javis_dmc_test dmc_mock_full_launch.py

#상태 관찰 gui 실행
ros2 run javis_dmc_test status_gui_node

#mock 서버 제어 gui 실행
ros2 run javis_dmc_test test_gui

