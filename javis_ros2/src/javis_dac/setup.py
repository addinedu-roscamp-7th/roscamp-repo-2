from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'javis_dac'

setup(
    name=package_name,
    version='0.0.0',
    # ✅ common/ 같은 하위 모듈까지 전부 포함
    packages=find_packages(include=['javis_dac', 'javis_dac.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='addin@example.com',
    description='Dobby / Pinky automation control package (singleton MyCobot)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_book_action_server = javis_dac.pick_book_action_server:main',
            'place_book_action_server = javis_dac.place_book_action_server:main',
            'dac_all_nodes = javis_dac.dac_all_nodes:main',
        ],
    },
)
