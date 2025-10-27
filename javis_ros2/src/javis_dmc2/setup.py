from setuptools import find_packages, setup

package_name = 'javis_dmc2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros'],
    zip_safe=True,
    maintainer='mac',
    maintainer_email='jongbob1918@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dmc2_node = javis_dmc2.dmc2_node:main'
        ],
    },
)
