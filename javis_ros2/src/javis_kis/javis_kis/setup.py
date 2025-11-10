from setuptools import setup
import os
from glob import glob # glob 임포트 추가

package_name = 'javis_kis'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리를 설치 경로에 포함시키는 부분 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='YOLO detector for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = javis_kis.yolo_detector:main',
            'rotation = javis_kis.rotation:main',
        ],
    },
)