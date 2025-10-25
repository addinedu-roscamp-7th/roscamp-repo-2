'''간단한 TEST GUI - Mock Bridge 및 Mock RCS 제어.'''

import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime

import rclpy
from rclpy.node import Node
from javis_dmc_test_msgs.srv import SetMockMethod, SendTask


class SimpleTestGUI(Node):
    '''간단한 TEST GUI 노드.'''

    # Mock 메서드 정의
    MOCK_METHODS = {
        'drive': ['move_to_target', 'stop', 'resume', 'start_patrol'],
        'arm': ['pick_book', 'place_book', 'change_pose'],
        'ai': ['detect_book', 'change_tracking_mode'],
    }

    # 작업 타입
    TASKS = [
        ('도서 픽업', 'pickup'),
        ('길 안내', 'guiding'),
        ('반납 정리', 'reshelving'),
        ('좌석 청소', 'clean_seat'),
    ]

    def __init__(self):
        super().__init__('test_gui_simple')

        # Service Clients
        self.mock_client = self.create_client(SetMockMethod, '/test/mock_bridge/set_method')
        self.task_client = self.create_client(SendTask, '/test/mock_rcs/send_task')

        # GUI 상태
        self.mock_state = {}  # {('drive', 'move_to_target'): {'enabled': BooleanVar, ...}}

        self.get_logger().info('Simple TEST GUI 노드 초기화 완료')

    def set_mock_method(self, interface: str, method: str, enabled: bool, success: bool, delay: float):
        '''Mock 메서드 설정.'''
        if not self.mock_client.wait_for_service(timeout_sec=0.5):
            self.log_message(f'[ERROR] Mock Bridge 서비스 미준비', 'error')
            return False

        request = SetMockMethod.Request()
        request.interface = interface
        request.method = method
        request.enabled = enabled
        request.success = success
        request.delay = delay

        future = self.mock_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_mock_response(f, interface, method))
        return True

    def send_task(self, task_type: str):
        '''작업 전송.'''
        if not self.task_client.wait_for_service(timeout_sec=0.5):
            self.log_message(f'[ERROR] Mock RCS 서비스 미준비', 'error')
            return False

        request = SendTask.Request()
        request.task_type = task_type

        future = self.task_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_task_response(f, task_type))
        return True

    def _handle_mock_response(self, future, interface: str, method: str):
        '''Mock 설정 응답 처리.'''
        try:
            response = future.result()
            if response.success:
                self.log_message(f'[Mock] {interface}.{method}: {response.message}', 'success')
            else:
                self.log_message(f'[Mock ERROR] {interface}.{method}: {response.message}', 'error')
        except Exception as e:
            self.log_message(f'[Mock ERROR] {interface}.{method}: {e}', 'error')

    def _handle_task_response(self, future, task_type: str):
        '''작업 전송 응답 처리.'''
        try:
            response = future.result()
            if response.success:
                self.log_message(f'[Task] {task_type}: {response.message}', 'success')
            else:
                self.log_message(f'[Task ERROR] {task_type}: {response.message}', 'error')
        except Exception as e:
            self.log_message(f'[Task ERROR] {task_type}: {e}', 'error')

    def log_message(self, message: str, level: str = 'info'):
        '''로그 메시지 (GUI에서 설정될 콜백).'''
        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f'[{timestamp}] {message}')


class TestGUIWindow:
    '''TEST GUI 윈도우.'''

    def __init__(self, root: tk.Tk, node: SimpleTestGUI):
        self.root = root
        self.node = node
        self.log_callback = None

        # 윈도우 설정
        self.root.title('JAVIS DMC Simple Test GUI')
        self.root.geometry('1000x700')
        self.root.configure(bg='#1e1e1e')

        self._build_ui()

        # 노드 로그 콜백 설정
        self.node.log_message = self.log_message

    def _build_ui(self):
        '''UI 구성.'''
        # 제목
        title = tk.Label(
            self.root,
            text='JAVIS DMC Simple Test GUI',
            bg='#1e1e1e',
            fg='#4CAF50',
            font=('Arial', 16, 'bold'),
        )
        title.pack(pady=10)

        # 작업 전송 패널
        self._build_task_panel()

        # Mock 제어 패널
        self._build_mock_panel()

        # 로그 패널
        self._build_log_panel()

    def _build_task_panel(self):
        '''작업 전송 패널.'''
        frame = tk.LabelFrame(
            self.root,
            text='작업 전송 (Mock RCS)',
            bg='#2e2e2e',
            fg='#ffffff',
            font=('Arial', 11, 'bold'),
        )
        frame.pack(fill='x', padx=10, pady=5)

        button_frame = tk.Frame(frame, bg='#2e2e2e')
        button_frame.pack(pady=10)

        for label, task_type in self.node.TASKS:
            btn = tk.Button(
                button_frame,
                text=label,
                bg='#4CAF50',
                fg='white',
                font=('Arial', 10, 'bold'),
                width=12,
                command=lambda t=task_type: self._send_task(t),
            )
            btn.pack(side='left', padx=5)

    def _build_mock_panel(self):
        '''Mock 제어 패널.'''
        frame = tk.LabelFrame(
            self.root,
            text='Mock 제어 (개별 메서드)',
            bg='#2e2e2e',
            fg='#ffffff',
            font=('Arial', 11, 'bold'),
        )
        frame.pack(fill='both', expand=True, padx=10, pady=5)

        # 스크롤 가능한 프레임
        canvas = tk.Canvas(frame, bg='#2e2e2e', highlightthickness=0)
        scrollbar = tk.Scrollbar(frame, orient='vertical', command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg='#2e2e2e')

        scrollable_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all')),
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        # 각 인터페이스별 패널
        for interface, methods in self.node.MOCK_METHODS.items():
            self._build_interface_section(scrollable_frame, interface, methods)

    def _build_interface_section(self, parent, interface: str, methods: list):
        '''인터페이스별 섹션.'''
        section = tk.LabelFrame(
            parent,
            text=f'{interface.upper()} Interface ({len(methods)}개)',
            bg='#3e3e3e',
            fg='#ffeb3b',
            font=('Arial', 10, 'bold'),
        )
        section.pack(fill='x', padx=5, pady=5)

        # 헤더
        header_frame = tk.Frame(section, bg='#3e3e3e')
        header_frame.pack(fill='x', padx=5, pady=2)

        headers = ['Method', 'Enabled', 'Success', 'Delay(s)', '']
        widths = [20, 8, 8, 8, 10]

        for header, width in zip(headers, widths):
            tk.Label(
                header_frame,
                text=header,
                bg='#3e3e3e',
                fg='#ffffff',
                font=('Arial', 9, 'bold'),
                width=width,
            ).pack(side='left', padx=2)

        # 각 메서드 행
        for method in methods:
            self._build_method_row(section, interface, method)

    def _build_method_row(self, parent, interface: str, method: str):
        '''메서드 제어 행.'''
        row_frame = tk.Frame(parent, bg='#3e3e3e')
        row_frame.pack(fill='x', padx=5, pady=2)

        # 변수
        enabled_var = tk.BooleanVar(value=False)
        success_var = tk.BooleanVar(value=True)
        delay_var = tk.StringVar(value='0.0')

        self.node.mock_state[(interface, method)] = {
            'enabled': enabled_var,
            'success': success_var,
            'delay': delay_var,
        }

        # Method 이름
        tk.Label(
            row_frame,
            text=method,
            bg='#3e3e3e',
            fg='#ffffff',
            font=('Arial', 9),
            width=20,
            anchor='w',
        ).pack(side='left', padx=2)

        # Enabled 체크박스
        tk.Checkbutton(
            row_frame,
            variable=enabled_var,
            bg='#3e3e3e',
            selectcolor='#2e2e2e',
            activebackground='#3e3e3e',
            width=8,
        ).pack(side='left', padx=2)

        # Success 체크박스
        tk.Checkbutton(
            row_frame,
            variable=success_var,
            bg='#3e3e3e',
            selectcolor='#2e2e2e',
            activebackground='#3e3e3e',
            width=8,
        ).pack(side='left', padx=2)

        # Delay 입력
        tk.Entry(
            row_frame,
            textvariable=delay_var,
            width=8,
            bg='#2e2e2e',
            fg='#ffffff',
            insertbackground='white',
        ).pack(side='left', padx=2)

        # 적용 버튼
        tk.Button(
            row_frame,
            text='적용',
            bg='#2196F3',
            fg='white',
            font=('Arial', 8, 'bold'),
            width=10,
            command=lambda: self._apply_mock(interface, method),
        ).pack(side='left', padx=2)

    def _build_log_panel(self):
        '''로그 패널.'''
        frame = tk.LabelFrame(
            self.root,
            text='이벤트 로그',
            bg='#2e2e2e',
            fg='#ffffff',
            font=('Arial', 11, 'bold'),
        )
        frame.pack(fill='both', expand=False, padx=10, pady=5)

        self.log_text = scrolledtext.ScrolledText(
            frame,
            height=10,
            bg='#1e1e1e',
            fg='#00ff00',
            font=('Consolas', 9),
            state='disabled',
        )
        self.log_text.pack(fill='both', expand=True, padx=5, pady=5)

        # 태그 설정
        self.log_text.tag_config('info', foreground='#00ff00')
        self.log_text.tag_config('success', foreground='#4CAF50')
        self.log_text.tag_config('error', foreground='#f44336')

    def _apply_mock(self, interface: str, method: str):
        '''Mock 설정 적용.'''
        state = self.node.mock_state.get((interface, method))
        if not state:
            return

        enabled = state['enabled'].get()
        success = state['success'].get()

        try:
            delay = float(state['delay'].get())
        except ValueError:
            delay = 0.0
            state['delay'].set('0.0')

        self.node.set_mock_method(interface, method, enabled, success, delay)
        self.log_message(f'[요청] {interface}.{method} Mock 설정 적용 요청', 'info')

    def _send_task(self, task_type: str):
        '''작업 전송.'''
        self.node.send_task(task_type)
        self.log_message(f'[요청] {task_type} 작업 전송 요청', 'info')

    def log_message(self, message: str, level: str = 'info'):
        '''로그 메시지 추가.'''
        timestamp = datetime.now().strftime('%H:%M:%S')
        log_line = f'[{timestamp}] {message}\n'

        self.log_text.configure(state='normal')
        self.log_text.insert('end', log_line, level)
        self.log_text.configure(state='disabled')
        self.log_text.see('end')


def main():
    '''TEST GUI 실행 엔트리 포인트.'''
    import threading

    rclpy.init()
    node = SimpleTestGUI()

    # ROS2 스핀 스레드
    def spin():
        rclpy.spin(node)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    # GUI 실행
    root = tk.Tk()
    gui = TestGUIWindow(root, node)

    def on_close():
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()

    root.protocol('WM_DELETE_WINDOW', on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
