from setuptools import find_packages, setup

package_name = 'javis_ddc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # javis_ddc/setup.py (일부)
    install_requires=['setuptools', 'rclpy', 'cv_bridge', 'numpy', 'opencv-python', 'pyyaml', 'nav2_simple_commander', 'torch', 'torchvision', 'dlib'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ddc_node = javis_ddc.ddc_node:main',
            'my_subscriber = javis_ddc.my_subscriber:main',
            'pose_publisher = javis_ddc.pose_publisher:main',
            'move_to_bookshelf = javis_ddc.move_to_bookshelf:main',
            'detectTracking = javis_ddc.detectTracking:main',
            'tracking_test_server = javis_ddc.tracking_test_server:main',
            'tracking_test_client = javis_ddc.tracking_test_client:main',
        ],
    },
)
