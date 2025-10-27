#!/usr/bin/env python3
"""
DVS (Device Vision System) 위젯
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QScrollArea
)


class WidgetDVS(QWidget):
    """DVS Mock 제어 위젯"""
    
    # DVS Mock 노드 목록
    MOCK_NODES_SERVICE = [
        ('mock_dvs_change_tracking_mode', 'ChangeTrackingMode Service'),
        ('mock_dvs_detect_trash', 'DetectTrash Service (Phase 2)'),
    ]
    
    MOCK_NODES_PUBLISHER = [
        ('mock_dvs_tracking_status', 'TrackingStatus Publisher'),
    ]
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        self.status_labels = {}
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>DVS: Device Vision System Mock</h2>')
        main_layout.addWidget(title)
        
        # 스크롤 영역
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Service Mock 그룹
        service_group = QGroupBox('Service Mock 노드')
        service_layout = QVBoxLayout(service_group)
        
        for node_name, description in self.MOCK_NODES_SERVICE:
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
            
            service_layout.addLayout(node_layout)
        
        layout.addWidget(service_group)
        
        # Publisher Mock 그룹
        publisher_group = QGroupBox('Publisher Mock 노드')
        publisher_layout = QVBoxLayout(publisher_group)
        
        for node_name, description in self.MOCK_NODES_PUBLISHER:
            node_layout = QHBoxLayout()
            node_layout.addWidget(QLabel(f'<b>{node_name}:</b>'))
            
            status_label = QLabel('STATUS: UNKNOWN')
            status_label.setStyleSheet('color: gray;')
            self.status_labels[node_name] = status_label
            node_layout.addWidget(status_label)
            
            node_layout.addWidget(QLabel(f'({description})'))
            node_layout.addStretch()
            
            btn_on = QPushButton('Set ON')
            btn_on.clicked.connect(lambda checked, n=node_name: self.set_mode(n, 'on'))
            node_layout.addWidget(btn_on)
            
            btn_off = QPushButton('Set OFF')
            btn_off.clicked.connect(lambda checked, n=node_name: self.set_mode(n, 'off'))
            node_layout.addWidget(btn_off)
            
            publisher_layout.addLayout(node_layout)
        
        layout.addWidget(publisher_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(150)
        desc.setHtml('''
        <h3>DVS Mock 제어</h3>
        <ul>
            <li><b>ChangeTrackingMode:</b> 피안내자 등록/추적 모드 전환</li>
            <li><b>TrackingStatus:</b> 피안내자 추적 상태 발행 (ON/OFF 제어)</li>
            <li><b>DetectTrash:</b> 쓰레기 감지 (Phase 2)</li>
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
            
            if mode == 'on':
                label.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'off':
                label.setStyleSheet('color: blue; font-weight: bold;')
            elif mode == 'active':
                label.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                label.setStyleSheet('color: red; font-weight: bold;')
            else:
                label.setStyleSheet('color: gray;')
