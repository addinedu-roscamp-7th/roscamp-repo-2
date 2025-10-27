from setuptools import setup
import os
from glob import glob # glob을 임포트합니다.

package_name = 'admin_gui_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'ui'), glob('ui/admin_ui.ui')),
        
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
            'admin_gui_node = admin_gui_package.admin_gui_node:main',
        ],
    },
)
