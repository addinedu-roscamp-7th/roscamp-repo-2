from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'javis_dmc_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim jong myung',
    maintainer_email='jongbob1918@gmail.com',
    description='JAVIS DMC Test Tools - Mock Bridge and Test GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_bridge_node = javis_dmc_test.mock_bridge_node:main',
            'mock_rcs_node = javis_dmc_test.mock_rcs_node:main',
            'test_gui_simple = javis_dmc_test.test_gui_simple:main',
        ],
    },
)
