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
        (os.path.join('share', package_name, 'resource'), glob('resource/*.yaml')),
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
            # Test GUI (PyQt6 기반 Mock 제어 GUI)
            'test_gui = javis_dmc_test.gui.gui_node:main',
            
            # Status GUI (기존 - Tkinter 기반 DMC 상태 모니터링)
            'status_gui_node = javis_dmc_test.status_gui.status_gui_node:main',
            
            # Phase 1: 필수 Mock 노드 (11개) - 각각 독립 실행
            # RCS Mock (1개)
            'mock_rcs_create_user_guide = javis_dmc_test.nodes.mock_rcs_create_user_guide:main',
            
            # DDC Mock (3개)
            'mock_ddc_move_to_target = javis_dmc_test.nodes.mock_ddc_move_to_target:main',
            'mock_ddc_guide_navigation = javis_dmc_test.nodes.mock_ddc_guide_navigation:main',
            'mock_ddc_control_command = javis_dmc_test.nodes.mock_ddc_control_command:main',
            
            # DVS Mock (2개)
            'mock_dvs_change_tracking_mode = javis_dmc_test.nodes.mock_dvs_change_tracking_mode:main',
            'mock_dvs_tracking_status = javis_dmc_test.nodes.mock_dvs_tracking_status:main',
            
            # GUI Mock (2개)
            'mock_gui_query_location_info = javis_dmc_test.nodes.mock_gui_query_location_info:main',
            'mock_gui_request_guidance = javis_dmc_test.nodes.mock_gui_request_guidance:main',
            
            # VRC Mock (2개)
            'mock_vrc_set_listening_mode = javis_dmc_test.nodes.mock_vrc_set_listening_mode:main',
            'mock_vrc_stt_result = javis_dmc_test.nodes.mock_vrc_stt_result:main',
            
            # Phase 2: 추가 Mock 노드 (9개) - 선택사항
            # DVS Mock (1개)
            'mock_dvs_detect_trash = javis_dmc_test.nodes.mock_dvs_detect_trash:main',
            
            # DAC Mock (8개)
            'mock_dac_pick_book = javis_dmc_test.nodes.mock_dac_pick_book:main',
            'mock_dac_place_book = javis_dmc_test.nodes.mock_dac_place_book:main',
            'mock_dac_collect_returned_books = javis_dmc_test.nodes.mock_dac_collect_returned_books:main',
            'mock_dac_sort_book = javis_dmc_test.nodes.mock_dac_sort_book:main',
            'mock_dac_clean_desk = javis_dmc_test.nodes.mock_dac_clean_desk:main',
            'mock_dac_collect_trash = javis_dmc_test.nodes.mock_dac_collect_trash:main',
            'mock_dac_dispose_trash = javis_dmc_test.nodes.mock_dac_dispose_trash:main',
            'mock_dac_change_pose = javis_dmc_test.nodes.mock_dac_change_pose:main',
        ],
    },
)
