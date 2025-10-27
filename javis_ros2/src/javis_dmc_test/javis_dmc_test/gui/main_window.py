#!/usr/bin/env python3
"""
Main Window
Test GUI의 메인 윈도우
"""

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QTextEdit, QSplitter
)
from PyQt6.QtCore import Qt, pyqtSlot

from .widgets.widget_rcs import WidgetRCS
from .widgets.widget_vrc import WidgetVRC
from .widgets.widget_guidance import WidgetGuidance
from .widgets.widget_tasks import WidgetTasks
from .widgets.widget_ddc import WidgetDDC
from .widgets.widget_dac import WidgetDAC
from .widgets.widget_dvs import WidgetDVS


class MainWindow(QMainWindow):
    """Test GUI 메인 윈도우"""
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        
        # 윈도우 설정
        self.setWindowTitle('JAVIS DMC Test GUI')
        self.setGeometry(100, 100, 1400, 900)
        
        # 메인 위젯 생성
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 메인 레이아웃 (좌우 분할)
        main_layout = QHBoxLayout(main_widget)
        
        # Splitter 생성 (좌: 탭, 우: 로그)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # 좌측: 탭 위젯
        self.tab_widget = QTabWidget()
        
        # 탭 1: RCS (Robot Control Service)
        self.widget_rcs = WidgetRCS(gui_node)
        self.tab_widget.addTab(self.widget_rcs, 'RCS (작업 제어)')
        
        # 탭 2: VRC (Voice Recognition Controller)
        self.widget_vrc = WidgetVRC(gui_node)
        self.tab_widget.addTab(self.widget_vrc, 'VRC (음성 인식)')
        
        # 탭 3: GUI Guidance (길안내 테스트)
        self.widget_guidance = WidgetGuidance(gui_node)
        self.tab_widget.addTab(self.widget_guidance, 'GUI Guidance (길안내)')
        
        # 탭 4: Task Test (작업 테스트)
        self.widget_tasks = WidgetTasks(gui_node)
        self.tab_widget.addTab(self.widget_tasks, 'Task Test (작업 실행)')
        
        # 탭 5: DDC (Device Drive Controller)
        self.widget_ddc = WidgetDDC(gui_node)
        self.tab_widget.addTab(self.widget_ddc, 'DDC (주행 제어)')
        
        # 탭 6: DAC (Device Arm Controller)
        self.widget_dac = WidgetDAC(gui_node)
        self.tab_widget.addTab(self.widget_dac, 'DAC (로봇팔 제어)')
        
        # 탭 7: DVS (Device Vision System)
        self.widget_dvs = WidgetDVS(gui_node)
        self.tab_widget.addTab(self.widget_dvs, 'DVS (비전 시스템)')
        
        # 우측: 로그 뷰
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumWidth(500)
        self.log_view.setPlaceholderText('ROS 2 로그 메시지가 여기에 표시됩니다...')
        
        # Splitter에 추가
        splitter.addWidget(self.tab_widget)
        splitter.addWidget(self.log_view)
        splitter.setStretchFactor(0, 3)  # 탭: 75%
        splitter.setStretchFactor(1, 1)  # 로그: 25%
        
        # 메인 레이아웃에 추가
        main_layout.addWidget(splitter)
        
        # Signal 연결
        self.gui_node.status_signal.connect(self.on_status_update)
        self.gui_node.log_signal.connect(self.on_log_message)
        
        # 초기 메시지
        self.log_view.append('[INFO] Test GUI started. Waiting for Mock nodes...')
    
    @pyqtSlot(str, str)
    def on_status_update(self, node_name: str, mode: str):
        """
        MockStatus 업데이트 슬롯
        각 위젯에 상태 업데이트 전달
        """
        # 각 탭의 위젯에 상태 업데이트 알림
        if node_name.startswith('mock_ddc'):
            self.widget_ddc.update_status(node_name, mode)
        elif node_name.startswith('mock_dac'):
            self.widget_dac.update_status(node_name, mode)
        elif node_name.startswith('mock_dvs'):
            self.widget_dvs.update_status(node_name, mode)
        elif node_name.startswith('mock_vrc'):
            self.widget_vrc.update_status(node_name, mode)
        elif node_name.startswith('mock_rcs'):
            self.widget_rcs.update_status(node_name, mode)
        elif node_name.startswith('mock_gui'):
            self.widget_guidance.update_status(node_name, mode)
    
    @pyqtSlot(str, int, str, str)
    def on_log_message(self, name: str, level: int, msg: str, timestamp: str):
        """
        /rosout 로그 메시지 슬롯
        
        level:
            10 = DEBUG
            20 = INFO
            30 = WARN
            40 = ERROR
            50 = FATAL
        """
        # 레벨에 따른 색상
        level_colors = {
            10: 'gray',
            20: 'black',
            30: 'orange',
            40: 'red',
            50: 'darkred'
        }
        level_names = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARN',
            40: 'ERROR',
            50: 'FATAL'
        }
        
        color = level_colors.get(level, 'black')
        level_name = level_names.get(level, 'UNKNOWN')
        
        # HTML 포맷으로 로그 추가
        log_html = f'<span style="color:{color};">[{level_name}] {name}: {msg}</span>'
        self.log_view.append(log_html)
        
        # 자동 스크롤 (최신 로그가 보이도록)
        scrollbar = self.log_view.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
