#!/usr/bin/env python3
"""
DAC (Device Arm Controller) 위젯
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QScrollArea
)


class WidgetDAC(QWidget):
    """DAC Mock 제어 위젯"""
    
    # DAC Mock 노드 목록
    MOCK_NODES = [
        ('mock_dac_pick_book', 'PickBook Action'),
        ('mock_dac_place_book', 'PlaceBook Action'),
        ('mock_dac_collect_returned_books', 'CollectReturnedBooks Action'),
        ('mock_dac_sort_book', 'SortBook Action'),
        ('mock_dac_clean_desk', 'CleanDesk Action'),
        ('mock_dac_collect_trash', 'CollectTrash Action'),
        ('mock_dac_dispose_trash', 'DisposeTrash Action'),
        ('mock_dac_change_pose', 'ChangePose Action'),
    ]
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        self.status_labels = {}
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>DAC: Device Arm Controller Mock</h2>')
        main_layout.addWidget(title)
        
        # 스크롤 영역
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        layout = QVBoxLayout(scroll_widget)
        
        # Mock 상태 표시 그룹
        status_group = QGroupBox('Mock 노드 상태 (Phase 2)')
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
        desc.setMaximumHeight(200)
        desc.setHtml('''
        <h3>DAC Mock 제어 (Phase 2 - 선택사항)</h3>
        <ul>
            <li><b>PickBook / PlaceBook:</b> 책 픽업 및 배치</li>
            <li><b>CollectReturnedBooks:</b> 반납도서 수거</li>
            <li><b>SortBook:</b> 도서 분류</li>
            <li><b>CleanDesk:</b> 책상 청소</li>
            <li><b>CollectTrash / DisposeTrash:</b> 쓰레기 수거 및 처리</li>
            <li><b>ChangePose:</b> 로봇팔 자세 변경</li>
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
