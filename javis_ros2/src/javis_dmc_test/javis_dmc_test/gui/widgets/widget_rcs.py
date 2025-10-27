#!/usr/bin/env python3
"""
RCS (Robot Control Service) 위젯
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit
)
from PyQt6.QtCore import Qt


class WidgetRCS(QWidget):
    """RCS Mock 제어 위젯"""
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        
        # 메인 레이아웃
        layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>RCS: Robot Control Service Mock</h2>')
        layout.addWidget(title)
        
        # Mock 상태 표시 그룹
        status_group = QGroupBox('Mock 노드 상태')
        status_layout = QVBoxLayout(status_group)
        
        # mock_rcs_create_user_guide 상태
        rcs_layout = QHBoxLayout()
        rcs_layout.addWidget(QLabel('<b>mock_rcs_create_user_guide:</b>'))
        self.status_label_rcs = QLabel('STATUS: UNKNOWN')
        self.status_label_rcs.setStyleSheet('color: gray;')
        rcs_layout.addWidget(self.status_label_rcs)
        rcs_layout.addStretch()
        
        btn_active = QPushButton('Set Active')
        btn_active.clicked.connect(lambda: self.set_mode('mock_rcs_create_user_guide', 'active'))
        rcs_layout.addWidget(btn_active)
        
        btn_error = QPushButton('Set Error')
        btn_error.clicked.connect(lambda: self.set_mode('mock_rcs_create_user_guide', 'error'))
        rcs_layout.addWidget(btn_error)
        
        status_layout.addLayout(rcs_layout)
        layout.addWidget(status_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(200)
        desc.setHtml('''
        <h3>RCS Mock 제어</h3>
        <p><b>CreateUserGuide Service:</b> GUI/VRC에서 작업 요청 시 DMC가 호출하는 서비스입니다.</p>
        <p><b>지원 작업 타입 (5개):</b></p>
        <ul>
            <li><b>guide_person:</b> 길안내 → GuidePerson Action</li>
            <li><b>pickup_book:</b> 도서 픽업 → PickupBook Action</li>
            <li><b>reshelving_book:</b> 반납 도서 정리 → ReshelvingBook Action</li>
            <li><b>clean_seat:</b> 좌석 청소 → CleanSeat Action</li>
            <li><b>rearrange_book:</b> 서가 정리 → RearrangeBook Action</li>
        </ul>
        <p><b>모드:</b> Active (정상 동작) / Error (작업 생성 실패)</p>
        ''')
        layout.addWidget(desc)
        
        layout.addStretch()
    
    def set_mode(self, node_name: str, mode: str):
        """Mock 노드의 mode 파라미터 변경"""
        self.gui_node.set_mock_mode(node_name, mode)
    
    def update_status(self, node_name: str, mode: str):
        """상태 라벨 업데이트"""
        if node_name == 'mock_rcs_create_user_guide':
            self.status_label_rcs.setText(f'STATUS: {mode.upper()}')
            
            # 모드별 색상 변경
            if mode == 'active':
                self.status_label_rcs.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                self.status_label_rcs.setStyleSheet('color: red; font-weight: bold;')
            else:
                self.status_label_rcs.setStyleSheet('color: gray;')
