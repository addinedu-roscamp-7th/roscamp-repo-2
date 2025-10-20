from setuptools import setup
import os
from glob import glob

package_name = 'javis_dvs'

# 패키지 내 OpenNI2 파일 경로 설정
openni2_base_dir = 'openni2_files/x64'
openni2_driver_dir = os.path.join(openni2_base_dir, 'OpenNI2', 'Drivers')

setup(
    name=package_name,
    version='0.0.1', # 버전 업데이트
    packages=[package_name],
    # data_files: 빌드 시 설치 경로로 복사할 파일들 지정
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 기존 launch 파일, 모델 파일 등은 그대로 유지
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'training', 'button_cnn'), glob('training/button_cnn/*.*')),
        # ... (기타 필요한 파일들) ...

        # --- 👇 OpenNI2 파일 설치 설정 추가 👇 ---
        # 1. 기본 라이브러리 및 ini 파일 설치
        #    -> install/javis_dvs/lib/javis_dvs/openni2_redist/ 로 복사됨
        (os.path.join('lib', package_name, 'openni2_redist'), [
             os.path.join(openni2_base_dir, 'libOpenNI2.so'),
             os.path.join(openni2_base_dir, 'OpenNI.ini')
        ]),
        # 2. 드라이버 파일 설치 (상대 경로 유지)
        #    -> install/javis_dvs/lib/javis_dvs/openni2_redist/OpenNI2/Drivers/ 로 복사됨
        (os.path.join('lib', package_name, 'openni2_redist', 'OpenNI2', 'Drivers'), [
             os.path.join(openni2_driver_dir, 'liborbbec.so'),
             os.path.join(openni2_driver_dir, 'orbbec.ini'),
             # libOniFile.so가 있다면 아래 줄 주석 해제
             # os.path.join(openni2_driver_dir, 'libOniFile.so')
        ]),
        # --- 👆 OpenNI2 파일 설치 설정 끝 👆 ---
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # 정보 업데이트
    maintainer_email='your@email.com', # 정보 업데이트
    description='Roomie Vision System Package', # 정보 업데이트
    license='Apache License 2.0', # 라이선스 확인
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvs_node = javis_dvs.dvs_node:main'
        ],
    },
)