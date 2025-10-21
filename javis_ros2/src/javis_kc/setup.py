from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'javis_kc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일을 포함시키기 위한 경로 추가
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # 'console_scripts'에 실행할 노드를 등록
    entry_points={
        'console_scripts': [
            'mycobot_tf_broadcaster = javis_kc.mycobot_tf_broadcaster:main',
        ],
    },
)
