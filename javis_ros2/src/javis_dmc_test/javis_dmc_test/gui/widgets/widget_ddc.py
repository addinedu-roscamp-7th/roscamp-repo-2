#!/usr/bin/env python3
"""
DDC (Device Drive Controller) 위젯
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QScrollArea
)


class WidgetDDC(QWidget):
    """DDC Mock 제어 위젯"""
    
    # DDC Mock 노드 목록
    MOCK_NODES = [
        ('mock_ddc_move_to_target', 'MoveToTarget Action'),
        ('mock_ddc_guide_navigation', 'GuideNavigation Action'),
        ('mock_ddc_control_command', 'DriveControlCommand Service'),
    ]
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        self.status_labels = {}
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>DDC: Device Drive Controller Mock</h2>')
        main_layout.addWidget(title)
        
        # 스크롤 영역
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Mock 상태 표시 그룹
        status_group = QGroupBox('Mock 노드 상태')
        status_layout = QVBoxLayout(status_group)
        
        for node_name, description in self.MOCK_NODES:
            node_layout = QHBoxLayout()
            node_layout.addWidget(QLabel(f'<b>{node_name}:</b>'))
            
            status_label = QLabel('STATUS: UNKNOWN')
            status_label.setStyleSheet('color: gray;')
            self.status_labels[node_name] = status_label
            node_layout.addWidget(status_label)
            
            node_layout.addWidget(QLabel(f'({description})'))
            node_layout.addStretch()
            
            btn_active = QPushButton('Set Active')
            btn_active.clicked.connect(lambda checked, n=node_name: self.set_mode(n, 'active'))
            node_layout.addWidget(btn_active)
            
            btn_error = QPushButton('Set Error')
            btn_error.clicked.connect(lambda checked, n=node_name: self.set_mode(n, 'error'))
            node_layout.addWidget(btn_error)
            
            status_layout.addLayout(node_layout)
        
        layout.addWidget(status_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(150)
        desc.setHtml('''
        <h3>DDC Mock 제어</h3>
        <ul>
            <li><b>MoveToTarget:</b> 목표 위치로 주행 (진행률 0→100%)</li>
            <li><b>GuideNavigation:</b> 사람 추종 주행 (길안내 모드)</li>
            <li><b>DriveControlCommand:</b> 주행 제어 (STOP/RESUME)</li>
        </ul>
        ''')
        layout.addWidget(desc)
        layout.addStretch()
        
        scroll.setWidget(scroll_widget)
        main_layout.addWidget(scroll)
    
    def set_mode(self, node_name: str, mode: str):
        """Mock 노드의 mode 파라미터 변경"""
        self.gui_node.set_mock_mode(node_name, mode)
    
    def update_status(self, node_name: str, mode: str):
        """상태 라벨 업데이트"""
        if node_name in self.status_labels:
            label = self.status_labels[node_name]
            label.setText(f'STATUS: {mode.upper()}')
            
            if mode == 'active':
                label.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                label.setStyleSheet('color: red; font-weight: bold;')
            else:
                label.setStyleSheet('color: gray;')
