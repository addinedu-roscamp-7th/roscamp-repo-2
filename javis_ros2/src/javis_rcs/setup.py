from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'javis_rcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='danny981027@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pickup_book = javis_rcs.pickup_book:main',
            'clean_seat = javis_rcs.clean_seate:main',
            'guide_person = javis_rcs.guide_person:main',
            'kreacher_perform_task = javis_rcs.kreacher_perform_task:main'
        ],
    },
)
