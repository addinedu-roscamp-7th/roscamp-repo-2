#!/usr/bin/env python3
"""
작업 테스트 위젯 (PickupBook, ReshelvingBook, CleanSeat, RearrangeBook)
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QComboBox, QLineEdit, QMessageBox
)
from PyQt6.QtCore import pyqtSlot


# 작업 타입별 Mock 좌표
MOCK_LOCATIONS = {
    'pickup_book': (5.0, 3.0, 0.0),
    'reshelving_book': (8.0, 2.0, 1.57),
    'clean_seat': (12.0, 5.0, 3.14),
    'rearrange_book': (10.0, 8.0, 0.0),
}


class WidgetTasks(QWidget):
    """작업 기능 테스트 위젯"""
    
    def __init__(self, gui_node):
        super().__init__()
        
        self.gui_node = gui_node
        
        # 메인 레이아웃
        layout = QVBoxLayout(self)
        
        # 제목
        title = QLabel('<h2>작업 기능 테스트 (RCS → DMC Action)</h2>')
        layout.addWidget(title)
        
        # 작업 선택 그룹
        task_group = QGroupBox('작업 타입 선택 및 실행')
        task_layout = QVBoxLayout(task_group)
        
        # 작업 타입 선택
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel('<b>작업 타입:</b>'))
        
        self.task_type_combo = QComboBox()
        self.task_type_combo.addItems([
            'guide_person (길안내)',
            'pickup_book (도서 픽업)',
            'reshelving_book (반납 정리)',
            'clean_seat (좌석 청소)',
            'rearrange_book (서가 정리)'
        ])
        type_layout.addWidget(self.task_type_combo)
        type_layout.addStretch()
        
        task_layout.addLayout(type_layout)
        
        # 목적지/위치 입력
        dest_layout = QHBoxLayout()
        dest_layout.addWidget(QLabel('목적지/위치 이름:'))
        self.destination_input = QLineEdit()
        self.destination_input.setPlaceholderText('예: 화장실, 서가A, 열람실1')
        dest_layout.addWidget(self.destination_input)
        
        task_layout.addLayout(dest_layout)
        
        # 실행 버튼
        btn_layout = QHBoxLayout()
        
        btn_execute = QPushButton('작업 실행 (CreateUserTask 호출)')
        btn_execute.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;')
        btn_execute.clicked.connect(self.execute_task)
        btn_layout.addWidget(btn_execute)
        
        task_layout.addLayout(btn_layout)
        
        layout.addWidget(task_group)
        
        # 빠른 테스트 그룹
        quick_group = QGroupBox('빠른 테스트 (프리셋 작업)')
        quick_layout = QVBoxLayout(quick_group)
        
        # 1행: 길안내, 도서 픽업
        row1 = QHBoxLayout()
        
        btn_guide = QPushButton('길안내\n(화장실)')
        btn_guide.clicked.connect(lambda: self.quick_test('guide_person', '화장실'))
        row1.addWidget(btn_guide)
        
        btn_pickup = QPushButton('도서 픽업\n(BOOK_12345)')
        btn_pickup.clicked.connect(lambda: self.quick_test('pickup_book', '서가A'))
        row1.addWidget(btn_pickup)
        
        btn_reshelving = QPushButton('반납 정리\n(반납데스크)')
        btn_reshelving.clicked.connect(lambda: self.quick_test('reshelving_book', '반납데스크'))
        row1.addWidget(btn_reshelving)
        
        quick_layout.addLayout(row1)
        
        # 2행: 좌석 청소, 서가 정리
        row2 = QHBoxLayout()
        
        btn_clean = QPushButton('좌석 청소\n(열람실1)')
        btn_clean.clicked.connect(lambda: self.quick_test('clean_seat', '열람실1'))
        row2.addWidget(btn_clean)
        
        btn_rearrange = QPushButton('서가 정리\n(서가B)')
        btn_rearrange.clicked.connect(lambda: self.quick_test('rearrange_book', '서가B'))
        row2.addWidget(btn_rearrange)
        
        row2.addStretch()
        
        quick_layout.addLayout(row2)
        
        layout.addWidget(quick_group)
        
        # 설명
        desc = QTextEdit()
        desc.setReadOnly(True)
        desc.setMaximumHeight(180)
        desc.setHtml('''
        <h3>RCS 작업 테스트</h3>
        <p>이 위젯은 RCS의 <b>CreateUserTask</b> 서비스를 호출하여 DMC에 작업을 요청합니다.</p>
        <p><b>작업 흐름:</b></p>
        <ol>
            <li>사용자가 작업 타입 및 목적지 선택</li>
            <li>Test GUI → RCS Mock: CreateUserTask 서비스 호출</li>
            <li>RCS Mock → DMC: 해당 작업의 Action Goal 전송</li>
            <li>DMC: Action 수락 및 실행</li>
        </ol>
        <p><b>주의:</b> Mock RCS가 'active' 모드여야 정상 동작합니다.</p>
        ''')
        layout.addWidget(desc)
        
        # 결과 표시 영역
        self.result_display = QTextEdit()
        self.result_display.setReadOnly(True)
        self.result_display.setMaximumHeight(120)
        self.result_display.setPlaceholderText('작업 실행 결과가 여기에 표시됩니다...')
        layout.addWidget(self.result_display)
        
        layout.addStretch()
        
        # Signal 연결
        self.gui_node.create_task_signal.connect(self.on_task_result)
    
    def execute_task(self):
        """선택한 작업 실행 (CreateUserTask 서비스 호출)"""
        # 작업 타입 파싱
        task_type_text = self.task_type_combo.currentText()
        task_type = task_type_text.split(' ')[0]  # 'guide_person', 'pickup_book', etc.
        
        # 목적지 가져오기
        destination = self.destination_input.text().strip()
        if not destination:
            QMessageBox.warning(self, '입력 오류', '목적지/위치 이름을 입력해주세요.')
            return
        
        # Mock 좌표 사용 (실제로는 QueryLocationInfo로 가져와야 함)
        if task_type == 'guide_person':
            x, y, theta = 10.5, -5.0, 1.57  # 화장실 기본값
        else:
            x, y, theta = MOCK_LOCATIONS.get(task_type, (0.0, 0.0, 0.0))
        
        self.gui_node.get_logger().info(
            f'Calling CreateUserTask: type={task_type}, destination={destination}'
        )
        
        self.result_display.append(
            f'<b>요청:</b> CreateUserTask(type="{task_type}", dest="{destination}")'
        )
        
        # 서비스 호출 (GUI 노드에 추가 필요)
        self.gui_node.call_create_user_task(task_type, destination, x, y, theta)
    
    def quick_test(self, task_type: str, destination: str):
        """빠른 테스트 (프리셋 작업)"""
        # ComboBox 선택 변경
        for i in range(self.task_type_combo.count()):
            if self.task_type_combo.itemText(i).startswith(task_type):
                self.task_type_combo.setCurrentIndex(i)
                break
        
        # 목적지 설정
        self.destination_input.setText(destination)
        
        # 실행
        self.execute_task()
    
    @pyqtSlot(bool, str, str)
    def on_task_result(self, success: bool, task_id: str, message: str):
        """CreateUserTask 응답 처리"""
        if success:
            result_html = f'''<b style="color:green;">✓ 작업 생성 성공</b>
            <br>Task ID: {task_id}
            <br>메시지: {message}'''
        else:
            result_html = f'<b style="color:red;">✗ 작업 생성 실패</b><br>메시지: {message}'
        
        self.result_display.append(result_html)
