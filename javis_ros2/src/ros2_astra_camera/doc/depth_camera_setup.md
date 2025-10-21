# 뎁스 카메라 환경 구축 가이드

## 1. 개요
- 대상 장비: Orbbec Dabai 기반 깊이 카메라(`astra_camera::OBCameraNodeFactory` 및 `params/dabai_params.yaml` 사용).
- 목적: `roscamp-repo-2/javis_ros2` 워크스페이스를 클론한 뒤 필요한 시스템/ROS 의존성을 설치하고, 카메라 드라이버를 빌드·실행하는 절차를 정리한다.

## 2. 시스템 패키지 설치
```bash
sudo apt update
sudo apt install \
  libusb-1.0-0-dev libgflags-dev libgoogle-glog-dev libeigen3-dev libjpeg-dev \
  ros-jazzy-image-transport ros-jazzy-image-publisher \
  ros-jazzy-camera-info-manager ros-jazzy-image-geometry \
  ros-jazzy-cv-bridge ros-jazzy-magic-enum
```

## 3. 소스 의존성 준비
### 3.1 공통 환경 변수
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local
```

### 3.2 libuvc 빌드 및 설치
```bash

# 로컬에서 dev_libs폴더를 생성하여 디펜던시 패키지를 보관 후 빌드하는게 좋음..
mkdir -p ~/dev_libs
cd ~/dev_libs
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

### 3.3 magic_enum 설치 (헤더 라이브러리)
- `sudo apt install ros-jazzy-magic-enum` 또는 소스 빌드:
```bash
cd ~/dev_libs
git clone https://github.com/Neargye/magic_enum.git
cd magic_enum
mkdir build && cd build
cmake ..
sudo make install
sudo ldconfig
```

## 4. udev 및 권한 설정
```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2/src/ros2_astra_camera/astra_camera
sudo ./scripts/install.sh          # 벤더 제공 udev 규칙 설치
sudo usermod -aG plugdev "$USER"   # 재로그인 필요
sudo udevadm control --reload-rules
sudo udevadm trigger
```
- 규칙 예시:
  ```
  SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0401", MODE:="0666", GROUP:="plugdev"
  ```

## 5. 워크스페이스 빌드 절차
```bash
cd ~/dev_ws/roscamp-repo-2/javis_ros2
rm -rf build log install
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local
colcon build --packages-select astra_camera astra_camera_msgs
colcon build
```
- 반복 빌드 시 `export CMAKE_PREFIX_PATH=...` 를 생략하지 않는다.
- 빌드 경고 중 `roomie_vs` 관련 오류가 발생하면 `src/javis_dvs/setup.py` 의 잔존 코드 조각을 정리한다.

## 6. 런치 및 시각화
```bash
source install/setup.bash
ros2 launch astra_camera dabai.launch.py
rviz2 -d $(ros2 pkg prefix astra_camera)/share/astra_camera/rviz/pointcloud.rviz
```
- 제공된 RViz 설정은 `/camera/*` 토픽의 QoS를 `Best Effort`로 미리 지정한다.
- 수동 설정 시 `Image`/`PointCloud2` 디스플레이의 Reliability를 `Best Effort`로 맞춘다.

## 7. 트러블슈팅
- `magic_enum.hpp` 없음: `ros-jazzy-magic-enum` 설치 및 `CMAKE_PREFIX_PATH` 확인.
- `image_geometry/pinhole_camera_model.h` 컴파일 오류: ROS Jazzy에서는 `.hpp` 확장자를 사용한다. 다음 파일을 패치한다.
  - `src/ros2_astra_camera/astra_camera/include/astra_camera/point_cloud_proc/point_cloud_xyz.h`
  - `src/ros2_astra_camera/astra_camera/src/point_cloud_proc/point_cloud_xyz.cpp`
  - `src/ros2_astra_camera/astra_camera/src/point_cloud_proc/point_cloud_xyzrgb.cpp`
- `on_set_parameters_callback` 서명 불일치: ROS Jazzy API에 맞춰 `ros_param_backend.h/.cpp`에서 `OnParametersSetCallbackType` → `OnSetParametersCallbackType`으로 변경한다.
- 런타임 경고 `USB events thread - failed to set priority`: 관리자 권한 실행 또는 `setcap cap_sys_nice+ep` 적용으로 해결.
- QoS 불일치 경고: 구독자 코드(예: `src/javis_dvs/javis_dvs/dvs_node.py`)에서 `QoSProfile(sensor_data=True)` 사용.

## 8. 모델 및 파라미터 확인
- 기본 파라미터 파일: `src/ros2_astra_camera/astra_camera/params/dabai_params.yaml`
  - 해상도/프레임률, D2C 정렬, ROI 설정 등을 포함.
- 다른 Orbbec 모델 사용 시 해당 파라미터 파일 수정 후 `colcon build`로 재배포한다.

## 9. 검증 체크리스트
- `ros2 topic list`에서 `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/color/points` 확인.
- RViz에서 컬러/IR/Depth 이미지와 포인트 클라우드가 수신되는지 점검.
- `~/.ros/log/.../astra_camera_container` 로그에 에러가 없는지 확인.

