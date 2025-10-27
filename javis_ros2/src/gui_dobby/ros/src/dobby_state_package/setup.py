from setuptools import setup
import os
from glob import glob # glob을 임포트합니다.

package_name = 'dobby_state_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'ui'), glob('ui/dobby_ui.ui')),
        
        (os.path.join('share', package_name, 'image'), glob('image/*.gif')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='jmk070994@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 📌 Case 1: 스크립트 확장자 없는 노드명 등록 (권장)
            'dobby_state_node = dobby_state_package.dobby_state_node:main', 
            
            # 📌 Case 2: 스크립트 확장자 포함하여 노드명 등록
            'dobby_state_node.py = dobby_state_package.dobby_state_node:main',
        ],
    },
)
