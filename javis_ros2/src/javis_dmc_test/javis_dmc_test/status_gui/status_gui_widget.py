'''DMC 상태 모니터링 및 제어용 Tkinter GUI - Status GUI.'''

from datetime import datetime
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Dict, List, Optional, Tuple

from javis_dmc.states.state_enums import MainState, SubState


class StateGraphCanvas(tk.Canvas):
    '''상태 다이어그램을 그리는 Canvas 위젯.'''

    def __init__(self, parent, state_graph: Dict[str, object], width: int = 320, height: int = 520) -> None:
        super().__init__(parent, width=width, height=height, bg='#1c1f26', highlightthickness=0)
        self._default_fill = '#2f3644'
        self._highlight_fill = '#8ab4f8'
        self._text_color = '#f5f7fa'
        self._line_color = '#455a64'
        self._rectangles: Dict[str, int] = {}
        self._labels: Dict[str, int] = {}
        self._positions: Dict[str, Tuple[float, float]] = {}
        self._transition_lines: List[int] = []
        self._draw_graph(state_graph or {})

    def _draw_graph(self, state_graph: Dict[str, object]) -> None:
        main_states = state_graph.get('main_states', []) if isinstance(state_graph, dict) else []
        if not isinstance(main_states, list):
            main_states = []

        margin = 40
        box_width = 200
        box_height = 34
        spacing = 18
        total_height = len(main_states) * (box_height + spacing) + margin
        canvas_height = max(total_height, int(self['height']))
        self.config(scrollregion=(0, 0, int(self['width']), canvas_height))

        for index, entry in enumerate(main_states):
            if not isinstance(entry, dict):
                continue
            name = entry.get('name', '')
            if not name:
                continue
            y = margin + index * (box_height + spacing)
            x0 = 30
            x1 = x0 + box_width
            y1 = y + box_height
            rect = self.create_rectangle(x0, y, x1, y1, fill=self._default_fill, outline='#37474f', width=2)
            label = self.create_text((x0 + x1) / 2, (y + y1) / 2, text=name, fill=self._text_color, font=('맑은 고딕', 11, 'bold'))
            self._rectangles[name.upper()] = rect
            self._labels[name.upper()] = label
            self._positions[name.upper()] = ((x0 + x1) / 2, (y + y1) / 2)

        transitions = state_graph.get('transitions', []) if isinstance(state_graph, dict) else []
        if not isinstance(transitions, list):
            transitions = []
        for entry in transitions:
            if not isinstance(entry, dict):
                continue
            from_state = str(entry.get('from', '')).upper()
            to_state = str(entry.get('to', '')).upper()
            if from_state in {'ANY', 'ANY_TASK'}:
                continue
            if from_state not in self._positions or to_state not in self._positions:
                continue
            start = self._positions[from_state]
            end = self._positions[to_state]
            line = self.create_line(start[0] + 100, start[1], end[0] + 100, end[1], arrow=tk.LAST, fill=self._line_color, width=2)
            self._transition_lines.append(line)

    def highlight(self, main_state: str, sub_state: Optional[str] = None) -> None:
        # main_state가 dict일 수 있으므로 안전하게 처리
        if isinstance(main_state, dict):
            name = ''
        else:
            name = (main_state or '').upper()

        for state, rect_id in self._rectangles.items():
            fill = self._highlight_fill if state == name else self._default_fill
            self.itemconfig(rect_id, fill=fill)
            label_id = self._labels.get(state)
            if label_id:
                self.itemconfig(label_id, fill=self._text_color)

    def show_transition(self, from_state: str, to_state: str) -> None:
        # from_state, to_state가 dict일 수 있으므로 안전하게 처리
        if isinstance(to_state, str):
            self.highlight(to_state, None)


class StatusGuiApp:
    '''DMC 테스트 GUI 애플리케이션 - Cleaned Version.'''

    _BASE_BG = '#1c1f26'
    _PANEL_BG = '#242b36'
    _LABEL_BG = '#2f3644'
    _TEXT_FG = '#f5f7fa'
    _HEADER_FG = '#b0bbc7'
    _ACCENT_FG = '#8ab4f8'
    _SUCCESS_FG = '#6dd96d'
    _WARN_FG = '#ffb84d'
    _ERROR_FG = '#ff6b6b'
    _SYSTEM_FONT = ('맑은 고딕', 9)
    _HEADER_FONT = ('맑은 고딕', 10, 'bold')
    _DESCRIPTION_FONT = ('맑은 고딕', 8)
    _DEFAULT_LABEL = (_PANEL_BG, _HEADER_FG)

    _STATE_DESCRIPTIONS = {
        'INITIALIZING': '시스템을 초기화하고 있습니다.',
        'IDLE': '대기 상태입니다.',
        'CHARGING': '충전 중입니다.',
        'MOVING_TO_CHARGER': '충전소로 이동 중입니다.',
        'FORCE_MOVE_TO_CHARGER': '강제로 충전소로 이동 중입니다.',
        'ROAMING': '자율 순찰 중입니다.',
        'LISTENING': '음성 인식 모드입니다.',
        'EMERGENCY_STOP': '긴급 정지 상태입니다.',
        'PICKING_UP_BOOK': '책을 픽업하고 있습니다.',
        'RESHELVING_BOOK': '책을 리셀빙하고 있습니다.',
        'GUIDING': '사용자를 안내하고 있습니다.',
        'CLEANING_DESK': '책상을 청소하고 있습니다.',
        'SORTING_SHELVES': '서가를 정리하고 있습니다.',
        'MAIN_ERROR': '알 수 없는 오류 상태입니다.',
    }
    _LISTENING_COLORS = {
        '활성': ('#1b1f24', '#ffd54f'),
        '비활성': _DEFAULT_LABEL,
    }
    _FEEDBACK_COLORS = {
        'success': ( '#2e7d32', '#ffffff'),
        'resumed': ( '#2e7d32', '#ffffff'),
        'busy': ( '#ffb300', '#1b1f24'),
        'warn': ( '#ffb300', '#1b1f24'),
        'rejected': ( '#c62828', '#ffffff'),
        'error': ('#c62828', '#ffffff'),
        'emergency_stop': ( '#c62828', '#ffffff'),
    }

    _STATE_COLORS = {
        'INITIALIZING': ('#1b1f24', '#ffb300'),
        'IDLE': ('#1976d2', '#ffffff'),
        'CHARGING': ('#388e3c', '#ffffff'),
        'MOVING_TO_CHARGER': ( '#f57c00', '#ffffff'),
        'FORCE_MOVE_TO_CHARGER': ( '#d32f2f', '#ffffff'),
        'ROAMING': ('#7b1fa2', '#ffffff'),
        'LISTENING': ('#1b1f24', '#ffd54f'),
        'EMERGENCY_STOP': ( '#d32f2f', '#ffffff'),
        'PICKING_UP_BOOK': ('#00796b', '#ffffff'),
        'RESHELVING_BOOK': ('#00796b', '#ffffff'),
        'GUIDING': ('#00796b', '#ffffff'),
        'CLEANING_DESK': ('#00796b', '#ffffff'),
        'SORTING_SHELVES': ('#00796b', '#ffffff'),
        'MAIN_ERROR': ('#d32f2f', '#ffffff'),
    }

    _BATTERY_THRESHOLDS = [
        (75.0, ('#43a047', '#ffffff')),
        (40.0, ('#1e88e5', '#ffffff')),
        (20.0, ('#ffb300', '#1b1f24')),
    ]

    def __init__(self, root: tk.Tk, ros_node, event_queue) -> None:
        self.root = root
        self.node = ros_node
        self.event_queue = event_queue

        self.root.title('JAVIS DMC Status GUI')
        self.root.geometry('1400x800')
        self.root.minsize(1200, 700)
        self.root.configure(bg=self._BASE_BG)

        self._status_vars: Dict[str, tk.StringVar] = {}
        self._status_labels: Dict[str, tk.Label] = {}
        self._state_hint_var = tk.StringVar(value='상태 정보가 여기에 표시됩니다.')
        self._mode_description = ''
        self._state_description = ''

        self.state_graph_data = self.node.describe_state_machine() or {}
        runtime = self.state_graph_data.get('runtime', {}) if isinstance(self.state_graph_data, dict) else {}
        self._patrol_active = False
        if isinstance(runtime, dict):
            if 'patrol_active' in runtime:
                self._patrol_active = bool(runtime.get('patrol_active'))
        self._state_graph_canvas: Optional[StateGraphCanvas] = None

        # 3-column layout: left (status+control), center (logs), right (state graph)
        self._left_panel = tk.Frame(self.root, bg=self._BASE_BG)
        self._left_panel.pack(side='left', fill='y', padx=12, pady=12)
        self._center_panel = tk.Frame(self.root, bg=self._BASE_BG)
        self._center_panel.pack(side='left', fill='both', expand=True, padx=(0, 12), pady=12)
        self._right_panel = tk.Frame(self.root, bg=self._BASE_BG)
        self._right_panel.pack(side='right', fill='y', padx=(0, 12), pady=12)

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Dark.TLabelframe', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=self._HEADER_FONT)
        style.configure('Dark.TLabelframe.Label', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=self._HEADER_FONT)
        style.configure('Dark.TFrame', background=self._PANEL_BG)
        style.configure('Dark.TButton', background='#3a4459', foreground=self._TEXT_FG, padding=6)
        style.map('Dark.TButton', background=[('active', '#455065'), ('pressed', '#2d3440')])
        style.configure('Treeview', background='#1f2430', fieldbackground='#1f2430', foreground=self._TEXT_FG, rowheight=26, relief='flat', bordercolor='#1f2430')
        style.configure('Treeview.Heading', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=('맑은 고딕', 10, 'bold'))

        self._log_tags = {'INFO', 'WARN', 'ERROR', 'STATUS', 'SUCCESS'}

        self._build_layout()
        self._schedule_queue_poll()

    # ------------------------------------------------------------------ Layout
    def _build_layout(self) -> None:
        self._build_status_panel()
        self._build_control_panel()
        self._build_rosout_panel()
        self._build_log_panel()
        self._build_state_graph_panel()

    def _build_status_panel(self) -> None:
        frame = ttk.LabelFrame(self._left_panel, text='로봇 상태', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=0, pady=(0, 10))

        labels = [
            ('main_state', '메인 상태', 'INITIALIZING'),
            ('sub_state', '서브 상태', 'NONE'),
            ('robot_mode', '로봇 모드', 'STANDBY'),
            ('battery_status', '배터리', '상태 확인중...'),
            ('mode_feedback', '모드 피드백', '대기중...'),
            ('listening_status', 'LISTENING', '비활성'),
        ]

        for idx, (key, label_text, default_value) in enumerate(labels):
            var = tk.StringVar(value=default_value)
            self._status_vars[key] = var

            label = tk.Label(
                frame,
                text=f'{label_text}:',
                bg=self._PANEL_BG,
                fg=self._HEADER_FG,
                font=self._HEADER_FONT,
            )
            label.grid(row=idx, column=0, padx=12, pady=4, sticky='w')

            status_label = tk.Label(
                frame,
                textvariable=var,
                bg=self._LABEL_BG,
                fg=self._TEXT_FG,
                font=self._SYSTEM_FONT,
                anchor='w',
                padx=8,
                pady=4,
            )
            status_label.grid(row=idx, column=1, padx=(8, 12), pady=4, sticky='ew')
            self._status_labels[key] = status_label

        hint_label = tk.Label(
            frame,
            textvariable=self._state_hint_var,
            bg=self._PANEL_BG,
            fg='#9aa5b4',
            font=self._DESCRIPTION_FONT,
            wraplength=350,
            justify='left',
        )
        hint_label.grid(row=len(labels), column=0, columnspan=2, padx=12, pady=(8, 4), sticky='ew')
        frame.columnconfigure(1, weight=1)

    def _build_control_panel(self) -> None:
        frame = ttk.LabelFrame(self._left_panel, text='제어 패널', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=0, pady=0)

        button_frame = ttk.Frame(frame, style='Dark.TFrame')
        button_frame.pack(fill='x', padx=6, pady=6)

        buttons = [
            ('STANDBY', lambda: self.node.request_set_mode('standby')),
            ('AUTONOMY', lambda: self.node.request_set_mode('autonomy')),
            ('긴급 정지', self.node.request_emergency_stop),
            ('긴급 해제', self.node.request_resume_navigation),
            ('LISTENING 시작', lambda: self.node.request_listening(True)),
            ('LISTENING 종료', lambda: self.node.request_listening(False)),
        ]

        for idx, (text, command) in enumerate(buttons):
            row, col = divmod(idx, 2)
            ttk.Button(button_frame, text=text, style='Dark.TButton', command=command).grid(
                row=row, column=col, padx=4, pady=4, sticky='ew'
            )
        for col in range(2):
            button_frame.columnconfigure(col, weight=1)

        # Raw logging is always enabled; toggle removed per request

        # Raw logging info buttons removed (live view only)

    def _build_rosout_panel(self) -> None:
        frame = ttk.LabelFrame(self._center_panel, text='ROS 로그 (/rosout)', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=0, pady=(0, 10))

        # rosout logging display area
        columns = ('timestamp', 'level', 'node', 'message')
        self.debug_tree = ttk.Treeview(frame, columns=columns, show='headings', height=20)
        self.debug_tree.heading('timestamp', text='시간')
        self.debug_tree.heading('level', text='레벨')
        self.debug_tree.heading('node', text='노드')
        self.debug_tree.heading('message', text='메시지')
        self.debug_tree.column('timestamp', width=100, anchor='center')
        self.debug_tree.column('level', width=80, anchor='w')
        self.debug_tree.column('node', width=200, anchor='w')
        self.debug_tree.column('message', anchor='w')
        self.debug_tree.pack(fill='both', expand=True, padx=6, pady=6)

        # 레벨별 색상 태그 설정
        self.debug_tree.tag_configure('ERROR', foreground='#ef5350')   # 빨강
        self.debug_tree.tag_configure('WARN', foreground='#ffb74d')    # 주황
        self.debug_tree.tag_configure('INFO', foreground='#81c784')    # 초록
        self.debug_tree.tag_configure('STATUS', foreground='#64b5f6')  # 파랑

        # Scrollbar for debug tree
        scrollbar = ttk.Scrollbar(frame, orient='vertical', command=self.debug_tree.yview)
        self.debug_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')

    def _build_log_panel(self) -> None:
        frame = ttk.LabelFrame(self._center_panel, text='이벤트 로그', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=0, pady=0)

        self.log_text = tk.Text(
            frame,
            bg='#1f2430',
            fg=self._TEXT_FG,
            font=('Consolas', 9),
            wrap='word',
            height=15,
            state='disabled',
        )
        self.log_text.pack(fill='both', expand=True, padx=6, pady=6)

        for tag in self._log_tags:
            color = {
                'INFO': self._TEXT_FG,
                'WARN': self._WARN_FG,
                'ERROR': self._ERROR_FG,
                'STATUS': '#9aa5b4',
                'SUCCESS': self._SUCCESS_FG,
            }.get(tag, self._TEXT_FG)
            self.log_text.tag_config(tag, foreground=color)

        scrollbar_log = ttk.Scrollbar(frame, orient='vertical', command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar_log.set)
        scrollbar_log.pack(side='right', fill='y')

    def _build_state_graph_panel(self) -> None:
        frame = ttk.LabelFrame(self._right_panel, text='메인 상태 다이어그램', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True)

        # 설명 추가
        info_label = tk.Label(
            frame,
            text='현재 메인 상태가 강조표시됩니다 (로봇 모드와 무관)',
            bg=self._PANEL_BG,
            fg='#9aa5b4',
            font=self._DESCRIPTION_FONT,
            pady=4
        )
        info_label.pack(fill='x', padx=6, pady=(6, 0))

        self._state_graph_canvas = StateGraphCanvas(frame, self.state_graph_data)
        self._state_graph_canvas.pack(fill='both', expand=True, padx=6, pady=6)

    # ------------------------------------------------------------------ Event handling
    def _schedule_queue_poll(self) -> None:
        self._poll_event_queue()
        self.root.after(100, self._schedule_queue_poll)

    def _poll_event_queue(self) -> None:
        max_events = 10
        for _ in range(max_events):
            try:
                event = self.event_queue.get_nowait()
            except:
                break
            self._handle_event(event)

    def _handle_event(self, event: dict) -> None:
        event_type = event.get('type', '')
        if event_type == 'state':
            self._handle_state_event(event)
        elif event_type == 'battery':
            self._handle_battery_event(event)
        elif event_type == 'mode_feedback':
            self._handle_mode_feedback_event(event)
        elif event_type == 'listening':
            self._handle_listening_event(event)
        elif event_type == 'state_transition':
            self._handle_state_transition_event(event)
        elif event_type == 'patrol_status':
            self._handle_patrol_status_event(event)
        elif event_type == 'notify':
            level = event.get('level', 'INFO')
            message = event.get('message', '')
            self._append_log(message, level)
        elif event_type == 'rosout':
            self._handle_rosout_event(event)

    def _handle_state_event(self, event: dict) -> None:
        main_state = event.get('main', 'UNKNOWN')
        sub_state = event.get('sub', 'NONE')
        mode = event.get('mode', 'UNKNOWN')

        # 상태별로 분리해서 표시
        self._status_vars['main_state'].set(main_state)
        self._status_vars['sub_state'].set(sub_state)
        # 모드 라벨은 mode_feedback 이벤트에서만 갱신한다

        # 상태별 색상 적용
        main_colors = self._STATE_COLORS.get(main_state, self._DEFAULT_LABEL)
        sub_colors = self._STATE_COLORS.get(sub_state, self._DEFAULT_LABEL) if sub_state != 'NONE' else self._DEFAULT_LABEL

        self._status_labels['main_state'].config(bg=main_colors[0], fg=main_colors[1])
        self._status_labels['sub_state'].config(bg=sub_colors[0], fg=sub_colors[1])
        # 로봇 모드 라벨 색상도 mode_feedback 이벤트에서만 갱신한다

        self._update_state_description(main_state, sub_state, mode)
        if self._state_graph_canvas:
            self._state_graph_canvas.highlight(main_state, sub_state)

        log_message = event.get('log', f'상태: {main_state} / {sub_state}')
        self._append_log(log_message, 'STATUS')

    def _handle_battery_event(self, event: dict) -> None:
        text = event.get('text', '알 수 없음')
        self._status_vars['battery_status'].set(text)
        self._update_battery_color(text)

    def _handle_mode_feedback_event(self, event: dict) -> None:
        message = event.get('message', '알 수 없는 피드백')
        status = event.get('status', '')
        mode = event.get('mode', 'UNKNOWN')

        # 모드 텍스트와 색상 갱신
        self._status_vars['mode_feedback'].set(f'{mode}: {message}')
        self._status_vars['robot_mode'].set(mode)
        mode_colors = {
            'STANDBY': ('#1976d2', '#ffffff'),
            'AUTONOMY': ('#7b1fa2', '#ffffff')
        }.get(mode, self._DEFAULT_LABEL)
        self._status_labels['robot_mode'].config(bg=mode_colors[0], fg=mode_colors[1])
        self._update_mode_feedback_color(status)

        log_message = event.get('log', message)
        level = event.get('level', 'INFO')
        self._append_log(log_message, level)

    def _handle_listening_event(self, event: dict) -> None:
        text = event.get('text', '알 수 없음')
        self._status_vars['listening_status'].set(text)
        colors = self._LISTENING_COLORS.get(text, self._DEFAULT_LABEL)
        label = self._status_labels['listening_status']
        label.config(bg=colors[0], fg=colors[1])

    def _handle_state_transition_event(self, event: dict) -> None:
        payload = event.get('payload', {})
        if isinstance(payload, dict):
            from_state = payload.get('from', '')
            to_state = payload.get('to', '')
            if from_state and to_state and self._state_graph_canvas:
                self._state_graph_canvas.show_transition(from_state, to_state)

    def _handle_patrol_status_event(self, event: dict) -> None:
        active = event.get('active', False)
        self._patrol_active = bool(active)

    def _update_state_description(self, main_state: str, sub_state: str, mode: str) -> None:
        self._state_description = self._STATE_DESCRIPTIONS.get(main_state, '상태 설명이 없습니다.')
        parts = [self._state_description]
        if sub_state and sub_state != 'NONE':
            parts.append(f'세부 상태: {sub_state}')
        if self._patrol_active:
            parts.append('순찰 활성화됨')
        if parts:
            self._state_hint_var.set(' | '.join(parts))
        else:
            self._state_hint_var.set('상태 정보가 여기에 표시됩니다.')

    def _update_battery_color(self, text: str) -> None:
        try:
            percentage_part = text.split('%')[0]
            percentage = float(percentage_part)
        except:
            percentage = 0.0

        colors = self._DEFAULT_LABEL
        for threshold, threshold_colors in self._BATTERY_THRESHOLDS:
            if percentage >= threshold:
                colors = threshold_colors
                break

        label = self._status_labels['battery_status']
        label.config(bg=colors[0], fg=colors[1])

    def _update_mode_feedback_color(self, status: str) -> None:
        colors = self._FEEDBACK_COLORS.get(status, self._DEFAULT_LABEL)
        label = self._status_labels['mode_feedback']
        label.config(bg=colors[0], fg=colors[1])

    def _append_log(self, message: str, level: str = 'INFO') -> None:
        timestamp = datetime.now().strftime('%H:%M:%S')
        full_message = f'[{timestamp}] {message}\n'

        self.log_text.config(state='normal')
        self.log_text.insert('end', full_message, level)
        self.log_text.config(state='disabled')
        self.log_text.see('end')

    def _handle_rosout_event(self, event: dict) -> None:
        """/rosout 로그 이벤트를 디버그 트리에 반영한다."""
        if not hasattr(self, 'debug_tree'):
            return
        try:
            ts = event.get('timestamp')
            # timestamp가 없거나 형식이 이상하면 현재 시간 사용
            try:
                timestamp = datetime.fromtimestamp(float(ts)).strftime('%H:%M:%S') if ts is not None else datetime.now().strftime('%H:%M:%S')
            except Exception:
                timestamp = datetime.now().strftime('%H:%M:%S')

            level = str(event.get('level', 'INFO'))
            node = str(event.get('node', ''))
            message = str(event.get('message', ''))
            if len(message) > 200:
                message = message[:197] + '...'

            # 레벨별 태그를 지정하여 행 삽입
            self.debug_tree.insert('', 'end', values=(timestamp, level, node, message), tags=(level,))
            # 오래된 항목 정리 (최대 100개 유지)
            children = self.debug_tree.get_children()
            if len(children) > 100:
                self.debug_tree.delete(children[0])
        except Exception:
            pass
