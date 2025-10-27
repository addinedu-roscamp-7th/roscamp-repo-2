#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DMC Mock 시스템 전체 런치 파일 (Phase 1 + Phase 2)

Phase 1 (필수) + Phase 2 (선택) 모든 Mock 노드를 실행합니다.
총 19개 Mock 노드가 독립적으로 실행됩니다.

실행 방법:
    ros2 launch javis_dmc_test dmc_mock_full.launch.py

구성:
- Phase 1 (11개): RCS 1, DDC 3, DVS 2, GUI 2, VRC 2
- Phase 2 (8개): DVS 1, DAC 7 (SortBook 제외)

개별 제어 예시:
    ros2 param set /mock_dac_pick_book mode error
    ros2 param set /mock_dvs_detect_trash mode active
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []
    
    # ========================================
    # Phase 1: 필수 Mock 노드 (11개)
    # ========================================
    
    # RCS Mock (1개)
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_rcs_create_user_guide',
        name='mock_rcs_create_user_guide',
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))
    
    # DDC Mock (3개)
    ddc_mocks = [
        'mock_ddc_navigate_to_pose',
        'mock_ddc_guide_navigation',
        'mock_ddc_control_command',
    ]
    for mock in ddc_mocks:
        nodes.append(Node(
            package='javis_dmc_test',
            executable=mock,
            name=mock,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    
    # DVS Mock (2개)
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_dvs_change_tracking_mode',
        name='mock_dvs_change_tracking_mode',
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_dvs_tracking_status',
        name='mock_dvs_tracking_status',
        output='screen',
        parameters=[{'use_sim_time': False, 'mode': 'off'}],
    ))
    
    # GUI Mock (2개)
    gui_mocks = [
        'mock_gui_query_location_info',
        'mock_gui_request_guidance',
    ]
    for mock in gui_mocks:
        nodes.append(Node(
            package='javis_dmc_test',
            executable=mock,
            name=mock,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    
    # VRC Mock (2개)
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_vrc_set_listening_mode',
        name='mock_vrc_set_listening_mode',
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_vrc_stt_result',
        name='mock_vrc_stt_result',
        output='screen',
        parameters=[{'use_sim_time': False, 'mode': 'off'}],
    ))
    
    # ========================================
    # Phase 2: 선택 Mock 노드 (9개)
    # ========================================
    
    # DVS Mock (1개)
    nodes.append(Node(
        package='javis_dmc_test',
        executable='mock_dvs_detect_trash',
        name='mock_dvs_detect_trash',
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))
    
    # DAC Mock (7개 - SortBook 제외)
    dac_mocks = [
        'mock_dac_pick_book',
        'mock_dac_place_book',
        'mock_dac_collect_returned_books',
        # 'mock_dac_sort_book',  # 인터페이스 미정의로 제외 (RearrangeBook으로 대체)
        'mock_dac_clean_desk',
        'mock_dac_collect_trash',
        'mock_dac_dispose_trash',
        'mock_dac_change_pose',  # Service로 수정 완료
    ]
    for mock in dac_mocks:
        nodes.append(Node(
            package='javis_dmc_test',
            executable=mock,
            name=mock,
            output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    
    return LaunchDescription(nodes)
