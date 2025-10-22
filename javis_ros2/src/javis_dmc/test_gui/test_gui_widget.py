'''DMC 상태 모니터링 및 제어용 Tkinter GUI.'''

from datetime import datetime
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Optional

from javis_dmc.states.state_enums import MainState, SubState


class TestGuiApp:
    '''DMC 테스트 GUI 애플리케이션.'''

    _TASK_STATUS_LABELS = {
        'pending': '대기',
        'in_progress': '진행중',
        'parallel': '병렬',
        'completed': '완료',
        'blocked': '보류',
    }

    _BASE_BG = '#1c1f26'
    _PANEL_BG = '#242b36'
    _LABEL_BG = '#2f3644'
    _TEXT_FG = '#f5f7fa'
    _HEADER_FG = '#b0bec5'
    _ACCENT_FG = '#8ab4f8'

    _HEADER_FONT = ('맑은 고딕', 11, 'bold')
    _VALUE_FONT = ('맑은 고딕', 12, 'bold')
    _DESCRIPTION_FONT = ('맑은 고딕', 10)
    _LOG_FONT = ('Consolas', 10)

    _DEFAULT_LABEL = (_TEXT_FG, _LABEL_BG)

    _MODE_COLORS = {
        'STANDBY': ('#ffffff', '#2e7d32'),
        'AUTONOMY': ('#ffffff', '#1565c0'),
        'EMERGENCYC_STOP': ('#ffffff', '#c62828'),
    }
    _MODE_HINTS = {
        'STANDBY': '충전소 대기 모드 – 작업 대기 및 충전 상태',
        'AUTONOMY': '자율 순찰 모드 – 웨이포인트 순찰과 작업 수락 가능',
    }
    _MAIN_STATE_COLORS = {
        'IDLE': ('#ffffff', '#1b5e20'),
        'ROAMING': ('#ffffff', '#1976d2'),
        'GUIDING': ('#ffffff', '#512da8'),
        'PICKING_UP_BOOK': ('#ffffff', '#ef6c00'),
        'RESHELVING_BOOK': ('#ffffff', '#7cb342'),
        'CLEANING_DESK': ('#ffffff', '#8d6e63'),
        'SORTING_SHELVES': ('#ffffff', '#00897b'),
        'MOVING_TO_CHARGER': ('#ffffff', '#ff7043'),
        'FORCE_MOVE_TO_CHARGER': ('#ffffff', '#d84315'),
        'CHARGING': ('#1b1f24', '#ffd54f'),
        'LISTENING': ('#1b1f24', '#ffca28'),
        'INITIALIZING': ('#ffffff', '#546e7a'),
        'EMERGENCYC_STOP': ('#ffffff', '#c62828'),
        'MAIN_ERROR': ('#ffffff', '#b71c1c'),
    }
    _STATE_HINTS = {
        'IDLE': '충전소에서 대기 중입니다.',
        'ROAMING': '웨이포인트를 순찰하며 작업 요청을 기다립니다.',
        'GUIDING': '사용자를 목적지로 안내 중입니다.',
        'PICKING_UP_BOOK': '지정된 도서를 픽업 중입니다.',
        'RESHELVING_BOOK': '반납 도서를 서가에 정리하고 있습니다.',
        'CLEANING_DESK': '좌석 정리 작업을 수행 중입니다.',
        'SORTING_SHELVES': '서가 재정리 작업을 수행 중입니다.',
        'MOVING_TO_CHARGER': '충전소로 이동 중입니다.',
        'FORCE_MOVE_TO_CHARGER': '배터리 부족으로 긴급 충전 복귀 중입니다.',
        'CHARGING': '충전 중입니다.',
        'LISTENING': '음성 명령을 듣고 있습니다.',
        'INITIALIZING': '시스템 초기화 중입니다.',
        'EMERGENCYC_STOP': '긴급 정지! 관리자 조치가 필요합니다.',
        'MAIN_ERROR': '알 수 없는 오류 상태입니다.',
    }
    _LISTENING_COLORS = {
        '활성': ('#1b1f24', '#ffd54f'),
        '비활성': _DEFAULT_LABEL,
    }
    _FEEDBACK_COLORS = {
        'success': ('#ffffff', '#2e7d32'),
        'resumed': ('#ffffff', '#2e7d32'),
        'busy': ('#1b1f24', '#ffb300'),
        'warn': ('#1b1f24', '#ffb300'),
        'rejected': ('#ffffff', '#c62828'),
        'error': ('#ffffff', '#c62828'),
        'emergency_stop': ('#ffffff', '#c62828'),
    }

    _BATTERY_THRESHOLDS = [
        (75.0, ('#ffffff', '#43a047')),
        (40.0, ('#ffffff', '#1e88e5')),
        (20.0, ('#1b1f24', '#ffb300')),
    ]

    def __init__(self, root: tk.Tk, ros_node, event_queue) -> None:
        self.root = root
        self.node = ros_node
        self.event_queue = event_queue

        self.root.title('JAVIS DMC Test GUI')
        self.root.geometry('980x700')
        self.root.minsize(900, 640)
        self.root.configure(bg=self._BASE_BG)

        self._status_vars: Dict[str, tk.StringVar] = {}
        self._status_labels: Dict[str, tk.Label] = {}
        self._state_hint_var = tk.StringVar(value='상태 정보가 여기에 표시됩니다.')
        self._mode_description = ''
        self._state_description = ''

        self.tasks: Dict[str, Dict[str, str]] = {}
        self._task_seq = 1

        self._log_tags = {'INFO', 'WARN', 'ERROR', 'STATUS', 'SUCCESS'}

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Dark.TLabelframe', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=self._HEADER_FONT)
        style.configure('Dark.TLabelframe.Label', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=self._HEADER_FONT)
        style.configure('Dark.TFrame', background=self._PANEL_BG)
        style.configure('Dark.TButton', background='#3a4459', foreground=self._TEXT_FG, padding=6)
        style.map('Dark.TButton', background=[('active', '#455065'), ('pressed', '#2d3440')])
        style.configure('Treeview', background='#1f2430', fieldbackground='#1f2430', foreground=self._TEXT_FG, rowheight=26, relief='flat', bordercolor='#1f2430')
        style.configure('Treeview.Heading', background=self._PANEL_BG, foreground=self._ACCENT_FG, font=('맑은 고딕', 10, 'bold'))

        self._build_layout()
        self._schedule_queue_poll()

    # ------------------------------------------------------------------ Layout
    def _build_layout(self) -> None:
        self._build_status_panel()
        self._build_control_panel()
        self._build_task_panel()
        self._build_log_panel()

    def _build_status_panel(self) -> None:
        frame = tk.Frame(self.root, bg=self._PANEL_BG, padx=18, pady=16)
        frame.pack(fill='x', padx=12, pady=(12, 10))

        title = tk.Label(frame, text='로봇 상태', bg=self._PANEL_BG, fg=self._ACCENT_FG, font=self._HEADER_FONT)
        title.grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 10))

        status_items = [
            ('모드', 'mode'),
            ('메인 상태', 'main'),
            ('서브 상태', 'sub'),
            ('배터리', 'battery'),
            ('LISTENING', 'listening'),
            ('피드백', 'feedback'),
        ]

        for idx, (label_text, key) in enumerate(status_items):
            row = 1 + idx // 2
            col = (idx % 2) * 2

            header = tk.Label(frame, text=label_text, bg=self._PANEL_BG, fg=self._HEADER_FG, font=('맑은 고딕', 10))
            header.grid(row=row, column=col, sticky='w', padx=(0, 8), pady=2)

            var = tk.StringVar(value='-')
            value_label = tk.Label(
                frame,
                textvariable=var,
                bg=self._LABEL_BG,
                fg=self._TEXT_FG,
                font=self._VALUE_FONT,
                anchor='w',
                padx=12,
                pady=4,
                width=18,
            )
            value_label.grid(row=row, column=col + 1, sticky='we', padx=(0, 18), pady=2)
            frame.columnconfigure(col + 1, weight=1)

            self._status_vars[key] = var
            self._status_labels[key] = value_label

        hint_label = tk.Label(
            frame,
            textvariable=self._state_hint_var,
            bg=self._PANEL_BG,
            fg='#9aa5b4',
            font=self._DESCRIPTION_FONT,
            anchor='w',
            wraplength=820,
            justify='left',
        )
        hint_label.grid(row=4, column=0, columnspan=4, sticky='we', pady=(12, 0))

        # expose legacy attributes for compatibility
        self.mode_var = self._status_vars['mode']
        self.main_state_var = self._status_vars['main']
        self.sub_state_var = self._status_vars['sub']
        self.battery_var = self._status_vars['battery']
        self.listening_var = self._status_vars['listening']
        self.feedback_var = self._status_vars['feedback']

        for key in self._status_vars:
            default = '비활성' if key == 'listening' else '-'
            self._set_status(key, default, tone='status')
        self._update_status_hint()

    def _build_control_panel(self) -> None:
        frame = ttk.LabelFrame(self.root, text='제어', style='Dark.TLabelframe')
        frame.pack(fill='x', padx=12, pady=(0, 10))

        button_specs = [
            ('Autonomy 모드', lambda: self.node.request_set_mode('autonomy')),
            ('Standby 모드', lambda: self.node.request_set_mode('standby')),
            ('긴급 정지', self.node.request_emergency_stop),
            ('긴급 해제', self.node.request_resume_navigation),
            ('LISTENING 시작', lambda: self.node.request_listening(True)),
            ('LISTENING 종료', lambda: self.node.request_listening(False)),
            ('현재 작업 성공 처리', lambda: self.node.request_force_task_result(True)),
            ('현재 작업 실패 처리', lambda: self.node.request_force_task_result(False)),
        ]

        for idx, (text, callback) in enumerate(button_specs):
            row = idx // 4
            col = idx % 4
            btn = ttk.Button(frame, text=text, command=callback, style='Dark.TButton')
            btn.grid(row=row, column=col, padx=6, pady=6, sticky='ew')

        manual_frame = ttk.LabelFrame(frame, text='수동 상태 설정', style='Dark.TLabelframe')
        manual_frame.grid(row=2, column=0, columnspan=4, padx=6, pady=(10, 4), sticky='ew')

        labels = [('모드', 'manual_mode_entry'), ('메인', 'manual_main_entry'), ('서브', 'manual_sub_entry')]
        for idx, (text, attr) in enumerate(labels):
            lbl = tk.Label(manual_frame, text=text, bg=self._PANEL_BG, fg=self._HEADER_FG, font=('맑은 고딕', 10))
            lbl.grid(row=0, column=idx * 2, padx=(4, 2), pady=4, sticky='w')
            entry = ttk.Entry(manual_frame, width=8)
            entry.grid(row=0, column=idx * 2 + 1, padx=(0, 8), pady=4, sticky='w')
            setattr(self, attr, entry)

        ttk.Button(manual_frame, text='적용', style='Dark.TButton', command=self._apply_manual_state).grid(
            row=0, column=6, padx=4, pady=4
        )
        manual_frame.columnconfigure(5, weight=1)

    def _build_task_panel(self) -> None:
        frame = ttk.LabelFrame(self.root, text='작업 진행 현황 (병목 대응 메모)', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=12, pady=10)

        entry_frame = ttk.Frame(frame, style='Dark.TFrame')
        entry_frame.pack(fill='x', pady=6, padx=4)

        tk.Label(entry_frame, text='작업명', bg=self._PANEL_BG, fg=self._HEADER_FG, font=('맑은 고딕', 10)).grid(
            row=0, column=0, padx=4, pady=2, sticky='w'
        )
        self.task_entry = ttk.Entry(entry_frame)
        self.task_entry.grid(row=0, column=1, padx=4, pady=2, sticky='ew')
        entry_frame.columnconfigure(1, weight=1)

        ttk.Button(entry_frame, text='추가', style='Dark.TButton', command=self._on_add_task).grid(
            row=0, column=2, padx=4, pady=2
        )

        columns = ('name', 'status')
        self.task_tree = ttk.Treeview(frame, columns=columns, show='headings', height=8)
        self.task_tree.heading('name', text='작업')
        self.task_tree.heading('status', text='상태')
        self.task_tree.pack(fill='both', expand=True, padx=4, pady=6)
        self.task_tree.tag_configure('parallel', background='#53461f')
        self.task_tree.tag_configure('blocked', background='#4a2525')

        btn_frame = ttk.Frame(frame, style='Dark.TFrame')
        btn_frame.pack(fill='x', pady=4, padx=4)
        status_buttons = [
            ('진행중', 'in_progress'),
            ('병렬', 'parallel'),
            ('보류', 'blocked'),
            ('완료', 'completed'),
            ('삭제', 'delete'),
        ]
        for idx, (text, status) in enumerate(status_buttons):
            if status == 'delete':
                command = self._remove_selected_task
            else:
                command = lambda s=status: self._update_selected_task(s)
            ttk.Button(btn_frame, text=text, style='Dark.TButton', command=command).grid(
                row=0, column=idx, padx=4, pady=4, sticky='ew'
            )
            btn_frame.columnconfigure(idx, weight=1)

    def _build_log_panel(self) -> None:
        frame = ttk.LabelFrame(self.root, text='이벤트 로그', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=12, pady=(0, 12))

        self.log_text = tk.Text(
            frame,
            height=12,
            state='disabled',
            bg=self._PANEL_BG,
            fg=self._TEXT_FG,
            insertbackground=self._TEXT_FG,
            font=self._LOG_FONT,
            relief='flat',
        )
        self.log_text.pack(fill='both', expand=True, padx=6, pady=6)

        self.log_text.tag_configure('INFO', foreground='#90caf9')
        self.log_text.tag_configure('WARN', foreground='#ffb300')
        self.log_text.tag_configure('ERROR', foreground='#f44336')
        self.log_text.tag_configure('STATUS', foreground='#80cbc4')
        self.log_text.tag_configure('SUCCESS', foreground='#66bb6a')

    # ------------------------------------------------------------------ Event loop
    def _schedule_queue_poll(self) -> None:
        self._process_queue()
        self.root.after(150, self._schedule_queue_poll)

    def _process_queue(self) -> None:
        events: List[Dict[str, str]] = []
        while True:
            try:
                event = self.event_queue.get_nowait()
            except Exception:
                break
            events.append(event)

        for event in events:
            event_type = event.get('type')
            level = event.get('level', 'INFO').upper()

            if event_type == 'state':
                self._handle_state_event(event, level)
            elif event_type == 'mode_feedback':
                message = event.get('message', '-')
                self._set_status('feedback', message, tone=event.get('status') or level)
                if 'mode' in event:
                    self._set_status('mode', event['mode'])
                if event.get('log'):
                    self._append_log(event['log'], level)
            elif event_type == 'battery':
                self._set_status('battery', event.get('text', '-'), tone='status')
            elif event_type == 'listening':
                self._set_status('listening', event.get('text', '비활성'), tone='status')
            elif event_type == 'notify':
                self._append_log(event.get('message', ''), level)

    # ------------------------------------------------------------------ Helpers
    def _handle_state_event(self, event: Dict[str, str], level: str) -> None:
        main_state = event.get('main', '-')
        sub_state = event.get('sub', '-')
        self._set_status('main', main_state)
        self._set_status('sub', sub_state)
        if 'mode' in event:
            self._set_status('mode', event['mode'])
        if event.get('log'):
            self._append_log(event['log'], level)

    def _set_status(self, key: str, text: str, tone: Optional[str] = None) -> None:
        if key not in self._status_vars:
            return

        normalized = text.strip() if isinstance(text, str) else str(text)
        if key == 'mode':
            normalized = normalized.upper()

        self._status_vars[key].set(normalized)
        fg, bg = self._resolve_status_color(key, normalized, tone)
        self._status_labels[key].configure(bg=bg, fg=fg)

        if key == 'mode':
            self._mode_description = self._MODE_HINTS.get(normalized.upper(), '')
            self._update_status_hint()
        elif key == 'main':
            self._state_description = self._STATE_HINTS.get(normalized.upper(), '')
            self._update_status_hint()

    def _resolve_status_color(self, key: str, text: str, tone: Optional[str]) -> tuple[str, str]:
        tone_key = (tone or '').lower()

        if tone_key in {'success'}:
            return ('#ffffff', '#2e7d32')
        if tone_key in {'warn'}:
            return ('#1b1f24', '#ffb300')
        if tone_key in {'error'}:
            return ('#ffffff', '#c62828')

        if key == 'mode':
            return self._MODE_COLORS.get(text.upper(), self._DEFAULT_LABEL)
        if key == 'main':
            return self._MAIN_STATE_COLORS.get(text.upper(), self._DEFAULT_LABEL)
        if key == 'feedback':
            if tone_key in self._FEEDBACK_COLORS:
                return self._FEEDBACK_COLORS[tone_key]
            return self._DEFAULT_LABEL
        if key == 'listening':
            return self._LISTENING_COLORS.get(text, self._DEFAULT_LABEL)
        if key == 'battery':
            percent = self._parse_percentage(text)
            if percent is not None:
                for threshold, colors in self._BATTERY_THRESHOLDS:
                    if percent >= threshold:
                        return colors
                return ('#ffffff', '#c62828')
            return self._DEFAULT_LABEL

        return self._DEFAULT_LABEL

    @staticmethod
    def _parse_percentage(text: str) -> Optional[float]:
        try:
            value = text.split('%')[0]
            return float(value.strip())
        except (ValueError, AttributeError, IndexError):
            return None

    def _update_status_hint(self) -> None:
        parts = []
        if self._mode_description:
            parts.append(f'모드: {self._mode_description}')
        if self._state_description:
            parts.append(f'상태: {self._state_description}')
        if parts:
            self._state_hint_var.set(' | '.join(parts))
        else:
            self._state_hint_var.set('상태 정보가 여기에 표시됩니다.')

    def _append_log(self, message: str, level: str = 'INFO') -> None:
        if not message:
            return
        level = level.upper()
        tag = level if level in self._log_tags else 'INFO'
        timestamp = datetime.now().strftime('%H:%M:%S')
        line = f'[{timestamp}] [{level}] {message}'
        self.log_text.configure(state='normal')
        self.log_text.insert('end', line + '\n', tag)
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    # ------------------------------------------------------------------ Task editor
    def _on_add_task(self) -> None:
        name = self.task_entry.get().strip()
        if not name:
            self._append_log('⚠️ 작업명을 입력하세요.', 'WARN')
            return
        task_id = f'T{self._task_seq}'
        self._task_seq += 1
        self.tasks[task_id] = {'name': name, 'status': 'pending'}
        self.task_tree.insert('', 'end', iid=task_id, values=(name, self._TASK_STATUS_LABELS['pending']))
        self.task_entry.delete(0, 'end')
        self._append_log(f'작업 등록: {name}', 'INFO')

    def _update_selected_task(self, status: str) -> None:
        selection = self.task_tree.selection()
        if not selection:
            self._append_log('⚠️ 상태를 변경할 작업을 선택하세요.', 'WARN')
            return
        label = self._TASK_STATUS_LABELS.get(status, status)
        for item in selection:
            task = self.tasks.get(item)
            if task is None:
                continue
            task['status'] = status
            tags = ()
            if status == 'parallel':
                tags = ('parallel',)
            elif status == 'blocked':
                tags = ('blocked',)
            self.task_tree.item(item, values=(task['name'], label), tags=tags)
        self._append_log(f'작업 상태 변경: {label}', 'INFO')

    def _remove_selected_task(self) -> None:
        selection = self.task_tree.selection()
        for item in selection:
            self.task_tree.delete(item)
            self.tasks.pop(item, None)
        if selection:
            self._append_log('작업을 삭제했습니다.', 'INFO')

    # ------------------------------------------------------------------ Manual state form
    def _apply_manual_state(self) -> None:
        def _parse(entry: tk.Entry) -> Optional[int]:
            text = entry.get().strip()
            if not text:
                return None
            try:
                return int(text)
            except ValueError:
                self._append_log(f'⚠️ 숫자를 입력하세요: {text}', 'WARN')
                return None

        mode = _parse(self.manual_mode_entry)
        main = _parse(self.manual_main_entry)
        sub = _parse(self.manual_sub_entry)

        if mode is None and main is None and sub is None:
            self._append_log('⚠️ 최소 하나의 값을 입력하세요.', 'WARN')
            return

        self.node.request_manual_state(mode, main, sub)
        self._append_log('수동 상태 설정 요청을 보냈습니다.', 'INFO')

    # ------------------------------------------------------------------ Legacy API
    def update_state_from_ros(self, main_state: MainState, sub_state: SubState, mode: str, log_msg: str) -> None:
        self._set_status('main', main_state.name if isinstance(main_state, MainState) else str(main_state))
        self._set_status('sub', sub_state.name if isinstance(sub_state, SubState) else str(sub_state))
        self._set_status('mode', mode)
        if log_msg:
            self._append_log(log_msg, 'STATUS')
