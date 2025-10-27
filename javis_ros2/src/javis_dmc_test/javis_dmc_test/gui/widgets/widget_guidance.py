#!/usr/bin/env python3
"""
GUI Guidance 위젯 (길안내 테스트)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QComboBox, QLineEdit, QMessageBox
)
from PyQt6.QtCore import pyqtSlot


# 목적지 좌표 매핑 (test_locations.yaml과 일치)
LOCATION_COORDS = {
    '화장실': (10.5, -5.0, 1.57),
    '카페': (15.0, 8.0, 3.14),
    '출입구': (0.0, 0.0, 0.0),
    '안내데스크': (2.0, 1.0, 0.0),
    '열람실': (5.0, 5.0, 0.0),
}


class WidgetGuidance(QWidget):
    """GUI 길안내 테스트 위젯"""
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        
        # 메인 레이아웃
        layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>GUI Guidance: 길안내 기능 테스트</h2>')
        layout.addWidget(title)
        
        # Mock 상태 표시 그룹
        status_group = QGroupBox('Mock 노드 상태')
        status_layout = QVBoxLayout(status_group)
        
        # mock_gui_query_location_info 상태
        query_layout = QHBoxLayout()
        query_layout.addWidget(QLabel('<b>mock_gui_query_location_info:</b>'))
        self.status_label_query = QLabel('STATUS: UNKNOWN')
        self.status_label_query.setStyleSheet('color: gray;')
        query_layout.addWidget(self.status_label_query)
        query_layout.addStretch()
        
        btn_active_1 = QPushButton('Set Active')
        btn_active_1.clicked.connect(lambda: self.set_mode('mock_gui_query_location_info', 'active'))
        query_layout.addWidget(btn_active_1)
        
        btn_error_1 = QPushButton('Set Error')
        btn_error_1.clicked.connect(lambda: self.set_mode('mock_gui_query_location_info', 'error'))
        query_layout.addWidget(btn_error_1)
        
        status_layout.addLayout(query_layout)
        
        # mock_gui_request_guidance 상태
        request_layout = QHBoxLayout()
        request_layout.addWidget(QLabel('<b>mock_gui_request_guidance:</b>'))
        self.status_label_request = QLabel('STATUS: UNKNOWN')
        self.status_label_request.setStyleSheet('color: gray;')
        request_layout.addWidget(self.status_label_request)
        request_layout.addStretch()
        
        btn_active_2 = QPushButton('Set Active')
        btn_active_2.clicked.connect(lambda: self.set_mode('mock_gui_request_guidance', 'active'))
        request_layout.addWidget(btn_active_2)
        
        btn_error_2 = QPushButton('Set Error')
        btn_error_2.clicked.connect(lambda: self.set_mode('mock_gui_request_guidance', 'error'))
        request_layout.addWidget(btn_error_2)
        
        status_layout.addLayout(request_layout)
        layout.addWidget(status_group)
        
        # 테스트 기능 그룹
        test_group = QGroupBox('길안내 기능 테스트')
        test_layout = QVBoxLayout(test_group)
        
        # 위치 정보 조회 섹션
        query_section_layout = QHBoxLayout()
        query_section_layout.addWidget(QLabel('위치 이름:'))
        self.location_input = QLineEdit()
        self.location_input.setPlaceholderText('예: 화장실')
        query_section_layout.addWidget(self.location_input)
        
        btn_query = QPushButton('Query Location Info')
        btn_query.clicked.connect(self.query_location)
        query_section_layout.addWidget(btn_query)
        
        test_layout.addLayout(query_section_layout)
        
        # 길안내 요청 섹션
        guidance_section_layout = QHBoxLayout()
        guidance_section_layout.addWidget(QLabel('목적지 선택:'))
        
        self.destination_combo = QComboBox()
        self.destination_combo.addItems(['화장실', '카페', '출입구', '안내데스크', '열람실'])
        guidance_section_layout.addWidget(self.destination_combo)
        
        btn_request_gui = QPushButton('Request Guidance (GUI)')
        btn_request_gui.clicked.connect(lambda: self.request_guidance('gui'))
        guidance_section_layout.addWidget(btn_request_gui)
        
        test_layout.addLayout(guidance_section_layout)
        
        # 빠른 테스트 버튼
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel('<b>빠른 테스트:</b>'))
        
        for dest in ['화장실', '카페', '출입구']:
            btn = QPushButton(f'{dest} 안내')
            btn.clicked.connect(lambda checked, d=dest: self.quick_guidance(d))
            quick_layout.addWidget(btn)
        
        quick_layout.addStretch()
        test_layout.addLayout(quick_layout)
        
        layout.addWidget(test_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(150)
        desc.setHtml('''
        <h3>GUI 길안내 Mock 제어</h3>
        <p><b>QueryLocationInfo:</b> 위치 정보 조회 (목적지 목록, 좌표 등)</p>
        <p><b>RequestGuidance:</b> 길안내 작업 요청 (GUI/Voice 통합 인터페이스)</p>
        <p>이 위젯은 실제 GUI가 DMC와 통신하는 방식을 시뮬레이션합니다.</p>
        ''')
        layout.addWidget(desc)
        
        # 결과 표시 영역
        self.result_display = QTextEdit()
        self.result_display.setReadOnly(True)
        self.result_display.setMaximumHeight(100)
        self.result_display.setPlaceholderText('서비스 호출 결과가 여기에 표시됩니다...')
        layout.addWidget(self.result_display)
        
        layout.addStretch()
        
        # Signal 연결
        self.gui_node.query_location_signal.connect(self.on_query_result)
        self.gui_node.request_guidance_signal.connect(self.on_guidance_result)
    
    def set_mode(self, node_name: str, mode: str):
        """Mock 노드의 mode 파라미터 변경"""
        self.gui_node.set_mock_mode(node_name, mode)
    
    def query_location(self):
        """위치 정보 조회 (실제 서비스 호출)"""
        location_name = self.location_input.text()
        self.gui_node.get_logger().info(f'Calling QueryLocationInfo: {location_name}')
        self.result_display.append(f'<b>요청:</b> QueryLocationInfo("{location_name}")')
        self.gui_node.call_query_location_info(location_name)
    
    @pyqtSlot(bool, str, float, float, float, str)
    def on_query_result(self, found: bool, location_id: str, x: float, y: float, theta: float, message: str):
        """QueryLocationInfo 응답 처리"""
        if found:
            result_html = f'''<b style="color:green;">✓ 위치 발견</b>
            <br>ID: {location_id}
            <br>좌표: x={x:.2f}, y={y:.2f}, theta={theta:.2f}
            <br>메시지: {message}'''
        else:
            result_html = f'<b style="color:red;">✗ 위치를 찾을 수 없음</b><br>메시지: {message}'
        
        self.result_display.append(result_html)
    
    def request_guidance(self, source: str):
        """길안내 요청 (실제 서비스 호출)"""
        destination = self.destination_combo.currentText()
        
        # 좌표 가져오기
        if destination not in LOCATION_COORDS:
            QMessageBox.warning(self, '오류', f'목적지 "{destination}"의 좌표를 찾을 수 없습니다.')
            return
        
        x, y, theta = LOCATION_COORDS[destination]
        
        self.gui_node.get_logger().info(f'Calling RequestGuidance: {destination} (source: {source})')
        self.result_display.append(f'<b>요청:</b> RequestGuidance("{destination}", source={source})')
        self.gui_node.call_request_guidance(destination, x, y, theta, source)
    
    @pyqtSlot(bool, str, str)
    def on_guidance_result(self, success: bool, task_id: str, message: str):
        """RequestGuidance 응답 처리"""
        if success:
            result_html = f'''<b style="color:green;">✓ 길안내 요청 성공</b>
            <br>Task ID: {task_id}
            <br>메시지: {message}'''
        else:
            result_html = f'<b style="color:red;">✗ 길안내 요청 실패</b><br>메시지: {message}'
        
        self.result_display.append(result_html)
    
    def quick_guidance(self, destination: str):
        """빠른 길안내 테스트"""
        # ComboBox 선택 변경
        index = self.destination_combo.findText(destination)
        if index >= 0:
            self.destination_combo.setCurrentIndex(index)
        
        # 길안내 요청
        self.request_guidance('gui')
    
    def update_status(self, node_name: str, mode: str):
        """상태 라벨 업데이트"""
        if node_name == 'mock_gui_query_location_info':
            self.status_label_query.setText(f'STATUS: {mode.upper()}')
            
            if mode == 'active':
                self.status_label_query.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                self.status_label_query.setStyleSheet('color: red; font-weight: bold;')
            else:
                self.status_label_query.setStyleSheet('color: gray;')
        
        elif node_name == 'mock_gui_request_guidance':
            self.status_label_request.setText(f'STATUS: {mode.upper()}')
            
            if mode == 'active':
                self.status_label_request.setStyleSheet('color: green; font-weight: bold;')
            elif mode == 'error':
                self.status_label_request.setStyleSheet('color: red; font-weight: bold;')
            else:
                self.status_label_request.setStyleSheet('color: gray;')
