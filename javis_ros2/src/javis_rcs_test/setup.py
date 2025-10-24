from setuptools import find_packages, setup

package_name = 'robot_control_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'pickup_client = robot_control_service.dobby_pickup_client:main',
            'pickup_server = robot_control_service.dobby_pickup_server:main',
            'reshelving_server = robot_control_service.dobby_reshelving_book_server:main',
            'reshelving_client = robot_control_service.dobby_reshelving_book_client:main',
            'guide_server = robot_control_service.guide_person_server:main',
            'guide_client = robot_control_service.guide_person_client:main',
            'clean_seat_server = robot_control_service.clean_seat_server:main',
            'clean_seat_client = robot_control_service.clean_seat_client:main',
            'json_test_server = robot_control_service.as_protocal_test:main',
            'json_test_client = robot_control_service.gateway_http_to_service:main',
            'json_test_client_get = robot_control_service.gateway_http_to_service_get:main',
            'topic_test = robot_control_service.topicp_test:main',
            
            

        ],
    },
)
