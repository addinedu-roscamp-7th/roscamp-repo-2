from setuptools import setup
from glob import glob
import os

package_name = 'javis_dac'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='jetcobot@todo.todo',
    description='JAVIS DAC integrated ROS2 package with actions and services',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_action_server = javis_dac.arm_action_server:main',
            'pose_service_server = javis_dac.pose_service_server:main',
        ],
    },
)
