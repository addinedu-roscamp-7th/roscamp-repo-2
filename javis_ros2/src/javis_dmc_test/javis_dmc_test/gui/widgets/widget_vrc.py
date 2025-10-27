#!/usr/bin/env python3
"""
VRC (Voice Recognition Controller) 위젯
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit
)


class WidgetVRC(QWidget):
    """VRC Mock 제어 위젯"""
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        
        # 메인 레이아웃
        layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>VRC: Voice Recognition Controller Mock</h2>')
        layout.addWidget(title)
        
        # Mock 상태 표시 그룹
        status_group = QGroupBox('Mock 노드 상태')
        status_layout = QVBoxLayout(status_group)
        
        # mock_vrc_set_listening_mode 상태
        listening_layout = QHBoxLayout()
        listening_layout.addWidget(QLabel('<b>mock_vrc_set_listening_mode:</b>'))
        self.status_label_listening = QLabel('STATUS: UNKNOWN')
        self.status_label_listening.setStyleSheet('color: gray;')
        listening_layout.addWidget(self.status_label_listening)
        listening_layout.addStretch()
        
        btn_active_1 = QPushButton('Set Active')
        btn_active_1.clicked.connect(lambda: self.set_mode('mock_vrc_set_listening_mode', 'active'))
        listening_layout.addWidget(btn_active_1)
        
        btn_error_1 = QPushButton('Set Error')
        btn_error_1.clicked.connect(lambda: self.set_mode('mock_vrc_set_listening_mode', 'error'))
        listening_layout.addWidget(btn_error_1)
        
        status_layout.addLayout(listening_layout)
        
        # mock_vrc_stt_result 상태 (Publisher)
        stt_layout = QHBoxLayout()
        stt_layout.addWidget(QLabel('<b>mock_vrc_stt_result (Publisher):</b>'))
        self.status_label_stt = QLabel('STATUS: UNKNOWN')
        self.status_label_stt.setStyleSheet('color: gray;')
        stt_layout.addWidget(self.status_label_stt)
        stt_layout.addStretch()
        
        btn_on = QPushButton('Set ON')
        btn_on.clicked.connect(lambda: self.set_mode('mock_vrc_stt_result', 'on'))
        stt_layout.addWidget(btn_on)
        
        btn_off = QPushButton('Set OFF')
        btn_off.clicked.connect(lambda: self.set_mode('mock_vrc_stt_result', 'off'))
        stt_layout.addWidget(btn_off)
        
        status_layout.addLayout(stt_layout)
        layout.addWidget(status_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(200)
        desc.setHtml('''
        <h3>VRC Mock 제어</h3>
        <p><b>SetListeningMode Service:</b> 음성 인식 모드 활성화/비활성화</p>
        <ul>
            <li><b>Active 모드:</b> 정상적으로 음성 인식 모드 전환</li>
            <li><b>Error 모드:</b> 음성 인식 모드 전환 실패</li>
        </ul>
        
        <p><b>STTResult Publisher:</b> 음성 인식 결과 발행 (Mock 테스트 문장)</p>
        <ul>
            <li><b>ON 모드:</b> 5초마다 테스트 문장 발행 ("화장실 가고 싶어", "카페로 안내해줘" 등)</li>
            <li><b>OFF 모드:</b> 발행 중지</li>
        </ul>
        ''')
        layout.addWidget(desc)
        
        layout.addStretch()
    
    def set_mode(self, node_name: str, mode: str):
        """Mock 노드의 mode 파라미터 변경"""
        self.gui_node.set_mock_mode(node_name, mode)
    
    def update_status(self, node_name: str, mode: str):
        """상태 라벨 업데이트"""
        if node_name == 'mock_vrc_set_listening_mode':
            self.status_label_listening.setText(f'STATUS: {mode.upper()}')
            
            if mode == 'active':
                self.status_label_listening.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                self.status_label_listening.setStyleSheet('color: red; font-weight: bold;')
            else:
                self.status_label_listening.setStyleSheet('color: gray;')
        
        elif node_name == 'mock_vrc_stt_result':
            self.status_label_stt.setText(f'STATUS: {mode.upper()}')
            
            if mode == 'on':
                self.status_label_stt.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'off':
                self.status_label_stt.setStyleSheet('color: blue; font-weight: bold;')
            else:
                self.status_label_stt.setStyleSheet('color: gray;')
