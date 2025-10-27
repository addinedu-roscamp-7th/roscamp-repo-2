#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DMC Mock 시스템 통합 런치 파일 (v4.0)

Phase 1 필수 Mock 노드들을 개별적으로 실행합니다.
각 Mock 노드는 독립적으로 파라미터 제어 가능합니다.

실행 방법:
    ros2 launch javis_dmc_test dmc_mock.launch.py

Phase 1 구성 (11개 노드):
- Mock RCS (1개): CreateUserTask Service Server
- Mock DDC (3개): GuideNavigation, NavigateToPose, ControlCommand
- Mock DVS (2개): ChangeTrackingMode, TrackingStatus
- Mock GUI (2개): QueryLocationInfo, RequestGuidance
- Mock VRC (2개): SetListeningMode, STTResult

개별 제어 예시:
    ros2 param set /mock_ddc_navigate_to_pose mode error
    ros2 param set /mock_dvs_tracking_status mode on
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ========================================
        # RCS Mock (1개)
        # ========================================
        Node(
            package='javis_dmc_test',
            executable='mock_rcs_create_user_guide',
            name='mock_rcs_create_user_guide',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        
        # ========================================
        # DDC Mock (3개) - 독립 실행
        # ========================================
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_navigate_to_pose',
            name='mock_ddc_navigate_to_pose',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_guide_navigation',
            name='mock_ddc_guide_navigation',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_ddc_control_command',
            name='mock_ddc_control_command',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        
        # ========================================
        # DVS Mock (2개) - 독립 실행
        # ========================================
        Node(
            package='javis_dmc_test',
            executable='mock_dvs_change_tracking_mode',
            name='mock_dvs_change_tracking_mode',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_dvs_tracking_status',
            name='mock_dvs_tracking_status',
            output='screen',
            parameters=[{'use_sim_time': False, 'mode': 'off'}],
        ),
        
        # ========================================
        # GUI Mock (2개) - v4.0 신규
        # ========================================
        Node(
            package='javis_dmc_test',
            executable='mock_gui_query_location_info',
            name='mock_gui_query_location_info',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_gui_request_guidance',
            name='mock_gui_request_guidance',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        
        # ========================================
        # VRC Mock (2개) - v4.0 신규
        # ========================================
        Node(
            package='javis_dmc_test',
            executable='mock_vrc_set_listening_mode',
            name='mock_vrc_set_listening_mode',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='javis_dmc_test',
            executable='mock_vrc_stt_result',
            name='mock_vrc_stt_result',
            output='screen',
            parameters=[{'use_sim_time': False, 'mode': 'off'}],
        ),
    ])
