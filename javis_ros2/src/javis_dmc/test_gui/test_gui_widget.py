'''DMC 상태 모니터링 및 제어용 Tkinter GUI.'''

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
        name = (main_state or '').upper()
        for state, rect_id in self._rectangles.items():
            fill = self._highlight_fill if state == name else self._default_fill
            self.itemconfig(rect_id, fill=fill)
            label_id = self._labels.get(state)
            if label_id:
                self.itemconfig(label_id, fill=self._text_color)

    def show_transition(self, from_state: str, to_state: str) -> None:
        self.highlight(to_state, None)


class TestGuiApp:
    '''DMC 테스트 GUI 애플리케이션.'''

    _TASK_STATUS_LABELS = {
        'pending': '대기',
        'in_progress': '진행중',
        'completed': '완료',
        'failed': '실패',
        'skipped': '미수행',
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
        'EMERGENCY_STOP': ('#ffffff', '#c62828'),
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
        'EMERGENCY_STOP': ('#ffffff', '#c62828'),
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
        'EMERGENCY_STOP': '긴급 정지! 관리자 조치가 필요합니다.',
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

    _MOCK_MODE_LABELS = {
        -1: '자동',
        0: '실장비',
        1: 'Mock',
    }
    _MOCK_MODE_CHOICES = ['자동', '실장비', 'Mock']
    _MOCK_MODE_PARAM = {
        '자동': 'auto',
        '실장비': 'real',
        'Mock': 'mock',
    }
    _MOCK_MODE_PARAM_TO_VALUE = {
        'auto': -1,
        'real': 0,
        'mock': 1,
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
        self._scenario_status_var = tk.StringVar(value='시나리오 미선택')
        self._scenario_summary_var = tk.StringVar(value='진행 단계: 0 완료 · 0 실패 · 0 대기')

        self._log_tags = {'INFO', 'WARN', 'ERROR', 'STATUS', 'SUCCESS'}
        self.test_mode_var = tk.BooleanVar(value=True)
        self.interface_mode_var = tk.StringVar(value='Real')
        self.scenario_var = tk.StringVar(value='')
        self.scenarios = self.node.get_scenarios()
        self.mock_methods = self.node.get_mock_methods()
        self.state_graph_data = self.node.describe_state_machine() or {}
        runtime = self.state_graph_data.get('runtime', {}) if isinstance(self.state_graph_data, dict) else {}
        self._patrol_active = False
        if isinstance(runtime, dict):
            if runtime.get('use_mock_interfaces'):
                self.interface_mode_var.set('Mock')
            if 'patrol_active' in runtime:
                self._patrol_active = bool(runtime.get('patrol_active'))
        self.mock_modes = self.node.get_mock_modes()
        if isinstance(runtime, dict) and isinstance(runtime.get('mock_modes'), dict):
            for key, value in runtime['mock_modes'].items():
                if isinstance(value, int):
                    self.mock_modes[key] = value
        self.mock_interface_var = tk.StringVar(value='drive')
        self.mock_method_var = tk.StringVar(value='')
        self.mock_error_var = tk.StringVar(value='')
        self.mock_mode_vars: Dict[str, tk.StringVar] = {}
        self.mock_mode_combos: Dict[str, ttk.Combobox] = {}
        self._state_graph_canvas: Optional[StateGraphCanvas] = None

        self._left_panel = tk.Frame(self.root, bg=self._BASE_BG)
        self._left_panel.pack(side='left', fill='both', expand=True)
        self._right_panel = tk.Frame(self.root, bg=self._BASE_BG)
        self._right_panel.pack(side='right', fill='y', padx=12, pady=12)

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
        self._build_state_graph_panel()
        self._sync_mock_mode_widgets()

    def _build_state_graph_panel(self) -> None:
        title = tk.Label(
            self._right_panel,
            text='상태 다이어그램',
            bg=self._BASE_BG,
            fg=self._ACCENT_FG,
            font=self._HEADER_FONT,
        )
        title.pack(anchor='nw', pady=(0, 8))

        self._state_graph_canvas = StateGraphCanvas(self._right_panel, self.state_graph_data)
        self._state_graph_canvas.pack(fill='both', expand=True)

    def _build_status_panel(self) -> None:
        frame = tk.Frame(self._left_panel, bg=self._PANEL_BG, padx=18, pady=16)
        frame.pack(fill='x', padx=12, pady=(12, 10))

        title = tk.Label(frame, text='로봇 상태', bg=self._PANEL_BG, fg=self._ACCENT_FG, font=self._HEADER_FONT)
        title.grid(row=0, column=0, columnspan=4, sticky='w', pady=(0, 10))

        status_items = [
            ('모드', 'mode'),
            ('메인 상태', 'main'),
            ('서브 상태', 'sub'),
            ('배터리', 'battery'),
            ('LISTENING', 'listening'),
            ('순찰', 'patrol'),
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
        self._set_status('patrol', '진행중' if self._patrol_active else '중지', tone='status')
        self._update_status_hint()

    def _build_control_panel(self) -> None:
        frame = ttk.LabelFrame(self._left_panel, text='제어', style='Dark.TLabelframe')
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

        mode_frame = ttk.Frame(frame, style='Dark.TFrame')
        mode_frame.grid(row=3, column=0, columnspan=4, sticky='ew', padx=6, pady=(10, 4))
        ttk.Checkbutton(
            mode_frame,
            text='Test 모드 (실패시 선택창 표시)',
            variable=self.test_mode_var,
            style='TCheckbutton',
        ).grid(row=0, column=0, padx=4, pady=4, sticky='w')

        ttk.Label(mode_frame, text='인터페이스', background=self._PANEL_BG, foreground=self._HEADER_FG).grid(
            row=0, column=1, padx=(12, 4), pady=4, sticky='w'
        )
        ttk.Radiobutton(
            mode_frame,
            text='Real',
            value='Real',
            variable=self.interface_mode_var,
            command=lambda: self._apply_interface_mode('Real'),
        ).grid(row=0, column=2, padx=4, pady=4, sticky='w')
        ttk.Radiobutton(
            mode_frame,
            text='Mock',
            value='Mock',
            variable=self.interface_mode_var,
            command=lambda: self._apply_interface_mode('Mock'),
        ).grid(row=0, column=3, padx=4, pady=4, sticky='w')

        mock_mode_frame = ttk.LabelFrame(frame, text='인터페이스 Mock 모드', style='Dark.TLabelframe')
        mock_mode_frame.grid(row=4, column=0, columnspan=4, padx=6, pady=(10, 4), sticky='ew')
        interfaces = [('drive', 'Drive'), ('arm', 'Arm'), ('ai', 'AI'), ('gui', 'GUI')]
        for idx, (key, label_text) in enumerate(interfaces):
            tk.Label(
                mock_mode_frame,
                text=label_text,
                bg=self._PANEL_BG,
                fg=self._HEADER_FG,
                font=('맑은 고딕', 10),
            ).grid(row=0, column=idx * 2, padx=(4, 2), pady=4, sticky='w')
            var = tk.StringVar(value=self._mock_mode_label(key))
            combo = ttk.Combobox(
                mock_mode_frame,
                textvariable=var,
                values=self._MOCK_MODE_CHOICES,
                state='readonly',
                width=10,
            )
            combo.grid(row=0, column=idx * 2 + 1, padx=(0, 8), pady=4, sticky='w')
            combo.bind('<<ComboboxSelected>>', lambda _evt, iface=key: self._on_mock_mode_change(iface))
            self.mock_mode_vars[key] = var
            self.mock_mode_combos[key] = combo
        mock_mode_frame.columnconfigure(len(interfaces) * 2 - 1, weight=1)

        scenario_frame = ttk.LabelFrame(frame, text='시나리오 실행', style='Dark.TLabelframe')
        scenario_frame.grid(row=5, column=0, columnspan=4, padx=6, pady=(10, 4), sticky='ew')

        ttk.Label(scenario_frame, text='시나리오', background=self._PANEL_BG, foreground=self._HEADER_FG).grid(
            row=0, column=0, padx=4, pady=4, sticky='w'
        )
        scenario_names = sorted(self.scenarios.keys())
        self.scenario_combo = ttk.Combobox(
            scenario_frame,
            textvariable=self.scenario_var,
            values=scenario_names,
            state='readonly',
            width=32,
        )
        if scenario_names:
            self.scenario_combo.current(0)
        self.scenario_combo.grid(row=0, column=1, padx=4, pady=4, sticky='ew')
        scenario_frame.columnconfigure(1, weight=1)

        ttk.Button(scenario_frame, text='시작', style='Dark.TButton', command=self._run_selected_scenario).grid(
            row=0, column=2, padx=4, pady=4
        )

        mock_frame = ttk.LabelFrame(frame, text='Mock 응답 제어', style='Dark.TLabelframe')
        mock_frame.grid(row=6, column=0, columnspan=4, padx=6, pady=(10, 4), sticky='ew')

        ttk.Label(mock_frame, text='인터페이스', background=self._PANEL_BG, foreground=self._HEADER_FG).grid(
            row=0, column=0, padx=4, pady=4, sticky='w'
        )
        interface_options = sorted(self.mock_methods.keys()) if self.mock_methods else ['drive', 'arm', 'ai', 'gui']
        if interface_options and self.mock_interface_var.get() not in interface_options:
            self.mock_interface_var.set(interface_options[0])
        self.mock_interface_combo = ttk.Combobox(
            mock_frame,
            textvariable=self.mock_interface_var,
            values=interface_options,
            state='readonly',
            width=12,
        )
        self.mock_interface_combo.grid(row=0, column=1, padx=4, pady=4, sticky='w')
        self.mock_interface_combo.bind('<<ComboboxSelected>>', self._refresh_mock_method_options)

        ttk.Label(mock_frame, text='메서드', background=self._PANEL_BG, foreground=self._HEADER_FG).grid(
            row=0, column=2, padx=4, pady=4, sticky='w'
        )
        self.mock_method_combo = ttk.Combobox(
            mock_frame,
            textvariable=self.mock_method_var,
            values=[],
            width=18,
        )
        self.mock_method_combo.grid(row=0, column=3, padx=4, pady=4, sticky='w')

        ttk.Label(mock_frame, text='에러 코드', background=self._PANEL_BG, foreground=self._HEADER_FG).grid(
            row=0, column=4, padx=4, pady=4, sticky='w'
        )
        ttk.Entry(mock_frame, textvariable=self.mock_error_var, width=12).grid(
            row=0, column=5, padx=4, pady=4, sticky='w'
        )

        ttk.Button(mock_frame, text='성공으로 설정', style='Dark.TButton', command=lambda: self._apply_mock_response(True)).grid(
            row=0, column=6, padx=4, pady=4
        )
        ttk.Button(mock_frame, text='실패로 설정', style='Dark.TButton', command=lambda: self._apply_mock_response(False)).grid(
            row=0, column=7, padx=4, pady=4
        )
        mock_frame.columnconfigure(3, weight=1)
        self._refresh_mock_method_options()

    def _build_task_panel(self) -> None:
        frame = ttk.LabelFrame(self._left_panel, text='시나리오 진행 현황', style='Dark.TLabelframe')
        frame.pack(fill='both', expand=True, padx=12, pady=10)

        header_frame = ttk.Frame(frame, style='Dark.TFrame')
        header_frame.pack(fill='x', padx=6, pady=(6, 2))

        tk.Label(
            header_frame,
            textvariable=self._scenario_status_var,
            bg=self._PANEL_BG,
            fg=self._HEADER_FG,
            font=('맑은 고딕', 10, 'bold'),
        ).grid(row=0, column=0, sticky='w', padx=4, pady=2)

        tk.Label(
            header_frame,
            textvariable=self._scenario_summary_var,
            bg=self._PANEL_BG,
            fg='#9aa5b4',
            font=self._DESCRIPTION_FONT,
        ).grid(row=1, column=0, sticky='w', padx=4, pady=2)
        header_frame.columnconfigure(0, weight=1)

        columns = ('order', 'description', 'status')
        self.task_tree = ttk.Treeview(frame, columns=columns, show='headings', height=8)
        self.task_tree.heading('order', text='단계')
        self.task_tree.heading('description', text='설명')
        self.task_tree.heading('status', text='상태')
        self.task_tree.column('order', width=60, anchor='center')
        self.task_tree.column('description', anchor='w')
        self.task_tree.column('status', width=120, anchor='center')
        self.task_tree.pack(fill='both', expand=True, padx=6, pady=6)
        self.task_tree.tag_configure('in_progress', background='#2d3d4f')
        self.task_tree.tag_configure('failed', background='#4a2525')
        self.task_tree.tag_configure('skipped', foreground='#9aa5b4')

        control_frame = ttk.Frame(frame, style='Dark.TFrame')
        control_frame.pack(fill='x', padx=6, pady=(0, 6))
        ttk.Button(
            control_frame,
            text='진척 초기화',
            style='Dark.TButton',
            command=self._clear_task_panel,
        ).grid(row=0, column=0, sticky='w', padx=4, pady=4)
        control_frame.columnconfigure(0, weight=1)

    def _build_log_panel(self) -> None:
        frame = ttk.LabelFrame(self._left_panel, text='이벤트 로그', style='Dark.TLabelframe')
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
            elif event_type == 'interface_mode':
                self.interface_mode_var.set(event.get('mode', 'Real'))
                self._append_log(f"인터페이스 모드 전환: {event.get('mode', 'Unknown')}", level)
            elif event_type == 'scenario':
                self._append_log(event.get('message', ''), level)
            elif event_type == 'patrol_status':
                active = bool(event.get('active'))
                self._patrol_active = active
                label = '진행중' if active else '중지'
                self._set_status('patrol', label, tone='status')
                self._append_log(f'[순찰] {label}', 'STATUS')
            elif event_type == 'state_transition':
                payload = event.get('payload', {})
                if isinstance(payload, dict):
                    frm = payload.get('from', {}).get('main', '') if isinstance(payload.get('from'), dict) else ''
                    to = payload.get('to', {}).get('main', '') if isinstance(payload.get('to'), dict) else ''
                    if self._state_graph_canvas:
                        self._state_graph_canvas.show_transition(frm.upper(), to.upper())
                    self._append_log(f"상태 전이: {frm} → {to}", 'STATUS')
        
    # ------------------------------------------------------------------ Helpers
    def _handle_state_event(self, event: Dict[str, str], level: str) -> None:
        main_state = event.get('main', '-')
        sub_state = event.get('sub', '-')
        self._set_status('main', main_state)
        self._set_status('sub', sub_state)
        if self._state_graph_canvas:
            self._state_graph_canvas.highlight(main_state)
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
        if key == 'patrol':
            return ('#ffffff', '#1976d2') if text == '진행중' else self._DEFAULT_LABEL
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
        parts.append('순찰: 진행중' if self._patrol_active else '순찰: 중지')
        if parts:
            self._state_hint_var.set(' | '.join(parts))
        else:
            self._state_hint_var.set('상태 정보가 여기에 표시됩니다.')

    def _apply_interface_mode(self, mode_label: str) -> None:
        target_label = 'Mock' if mode_label.lower() == 'mock' else 'Real'
        if self.interface_mode_var.get() == target_label:
            changed = self.node.set_interface_mode(target_label.lower() == 'mock')
        else:
            self.interface_mode_var.set(target_label)
            changed = self.node.set_interface_mode(target_label.lower() == 'mock')
        if not changed:
            # revert state
            current = 'Mock' if self.node._use_mock_interfaces else 'Real'
            self.interface_mode_var.set(current)

    def _run_selected_scenario(self) -> None:
        scenario_id = self.scenario_var.get()
        if not scenario_id:
            messagebox.showinfo('Scenario', '실행할 시나리오를 선택하세요.')
            return
        scenario = self.scenarios.get(scenario_id)
        if not scenario or not isinstance(scenario, dict):
            messagebox.showwarning('Scenario', f'시나리오를 찾을 수 없습니다: {scenario_id}')
            return

        steps = scenario.get('steps', [])
        if not isinstance(steps, list) or not steps:
            messagebox.showwarning('Scenario', '시나리오에 실행 가능한 단계가 없습니다.')
            return

        self._prepare_scenario_tracking(scenario_id, steps)
        self._append_log(f'[시나리오] {scenario_id} 시작', 'INFO')
        interactive = bool(self.test_mode_var.get())
        all_success = True
        for idx, step in enumerate(steps, start=1):
            self._set_task_status(idx, 'in_progress')
            success, detail = self._execute_scenario_step(step, interactive)
            summary = detail or f'Step {idx}: {step.get("action")}'
            if success:
                self._set_task_status(idx, 'completed')
                self._append_log(f'[시나리오] ✅ {summary}', 'SUCCESS')
                continue
            self._set_task_status(idx, 'failed')
            self._append_log(f'[시나리오] ❌ {summary}', 'ERROR')
            self._mark_remaining_steps(idx + 1)
            all_success = False
            break
        if all_success:
            self._append_log(f'[시나리오] {scenario_id} 완료', 'SUCCESS')
            counts_text = self._scenario_summary_var.get()
            self._scenario_summary_var.set(f'{scenario_id} 완료 | {counts_text}')
        else:
            counts_text = self._scenario_summary_var.get()
            self._scenario_summary_var.set(f'{scenario_id} 중단: {summary} | {counts_text}')

    def _execute_scenario_step(self, step: Dict[str, object], interactive: bool) -> Tuple[bool, str]:
        action = str(step.get('action', '')).lower()
        if not action:
            return False, '액션이 지정되지 않았습니다.'

        if action == 'call_service':
            target = str(step.get('target', ''))
            args = step.get('args', {})
            args = args if isinstance(args, dict) else {}
            success = self.node.call_service_step(target, **args)
            if success:
                return True, f'서비스 호출: {target}'
            if interactive:
                result = messagebox.askyesnocancel('시나리오 실패', f'[{target}] 호출 실패\n성공으로 간주하시겠습니까?')
                if result is True:
                    return True, f'서비스 호출: {target} (성공 간주)'
                if result is False:
                    return False, f'서비스 호출 실패: {target}'
                return False, '사용자가 시나리오를 취소했습니다.'
            return False, f'서비스 호출 실패: {target}'

        if action == 'wait_for_state':
            args = step.get('args', {})
            args = args if isinstance(args, dict) else {}
            main = args.get('main')
            sub = args.get('sub')
            timeout = float(args.get('timeout', 10.0))
            success = self.node.wait_for_state(main, sub, timeout)
            if success:
                return True, f'상태 대기 완료: {main or "*"}/{sub or "*"}'
            if interactive:
                result = messagebox.askyesnocancel('시나리오 실패', f'상태 대기 실패: {main}/{sub}\n성공으로 간주하시겠습니까?')
                if result is True:
                    return True, f'상태 대기: {main or "*"}/{sub or "*"} (성공 간주)'
                if result is False:
                    return False, f'상태 대기 실패: {main}/{sub}'
                return False, '사용자가 시나리오를 취소했습니다.'
            return False, f'상태 대기 실패: {main}/{sub}'

        if action == 'sleep':
            duration = float(step.get('seconds', step.get('duration', 1.0)))
            time.sleep(max(0.0, duration))
            return True, f'{duration:.1f}초 대기'

        if action == 'publish_mock':
            target = str(step.get('target', ''))
            self._append_log(f'[시나리오] Mock 호출 준비: {target} (미구현)', 'WARN')
            return True, f'Mock 호출(미구현): {target}'

        if action == 'log':
            message = str(step.get('message', ''))
            self._append_log(message, 'INFO')
            return True, message

        return False, f'지원하지 않는 액션: {action}'

    def _refresh_mock_method_options(self, *_args) -> None:
        interface = self.mock_interface_var.get().strip()
        methods = self.mock_methods.get(interface, [])
        self.mock_method_combo['values'] = methods
        if methods:
            if self.mock_method_var.get() not in methods:
                self.mock_method_var.set(methods[0])
        else:
            self.mock_method_var.set('')

    def _mock_mode_label(self, interface: str) -> str:
        value = self.mock_modes.get(interface, -1)
        return self._MOCK_MODE_LABELS.get(value, '자동')

    def _on_mock_mode_change(self, interface: str) -> None:
        var = self.mock_mode_vars.get(interface)
        if var is None:
            return
        label = var.get()
        mode_key = self._MOCK_MODE_PARAM.get(label, 'auto')
        success = self.node.set_mock_mode(interface, mode_key)
        if success:
            self.mock_modes[interface] = self._MOCK_MODE_PARAM_TO_VALUE.get(mode_key, -1)
            self._append_log(f'[Mock 모드] {interface} → {label}', 'INFO')
        else:
            # 롤백
            var.set(self._mock_mode_label(interface))

    def _sync_mock_mode_widgets(self) -> None:
        for key, var in self.mock_mode_vars.items():
            var.set(self._mock_mode_label(key))

    def _apply_mock_response(self, success_flag: bool) -> None:
        interface = self.mock_interface_var.get().strip()
        method = self.mock_method_var.get().strip()
        error_code = self.mock_error_var.get().strip()
        if not interface or not method:
            messagebox.showwarning('Mock 응답', '인터페이스와 메서드를 모두 입력하세요.')
            return
        scenario = method if not error_code else f'{method}:{error_code}'
        result = self.node.set_mock_response(interface, scenario, success_flag)
        if result:
            status = '성공' if success_flag else '실패'
            self._append_log(f'[Mock] {interface}.{method} → {status}', 'INFO')
        else:
            self._append_log(f'[Mock] 설정 실패: {interface}.{method}', 'ERROR')

    def _clear_task_panel(self) -> None:
        self.tasks.clear()
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)
        self._scenario_status_var.set('시나리오 미선택')
        self._scenario_summary_var.set('진행 단계: 0 완료 · 0 실패 · 0 대기')

    def _prepare_scenario_tracking(self, scenario_id: str, steps: List[Dict[str, object]]) -> None:
        self.tasks.clear()
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)

        for idx, step in enumerate(steps, start=1):
            step_id = f'S{idx}'
            description = self._describe_scenario_step(step)
            self.tasks[step_id] = {'name': description, 'status': 'pending'}
            self.task_tree.insert(
                '',
                'end',
                iid=step_id,
                values=(idx, description, self._TASK_STATUS_LABELS['pending']),
            )

        self._scenario_status_var.set(f'{scenario_id} (총 {len(steps)}단계)')
        self._update_scenario_summary()

    def _set_task_status(self, step_index: int, status: str) -> None:
        step_id = f'S{step_index}'
        task = self.tasks.get(step_id)
        if task is None:
            return

        task['status'] = status
        label = self._TASK_STATUS_LABELS.get(status, status)
        tags = (status,) if status in {'in_progress', 'failed', 'skipped'} else ()
        self.task_tree.item(step_id, values=(step_index, task['name'], label), tags=tags)
        if status == 'in_progress':
            self.task_tree.selection_set(step_id)
            self.task_tree.see(step_id)
        self._update_scenario_summary()

    def _mark_remaining_steps(self, start_index: int) -> None:
        for idx in range(start_index, len(self.tasks) + 1):
            step_id = f'S{idx}'
            task = self.tasks.get(step_id)
            if task is None:
                continue
            if task['status'] == 'pending':
                self._set_task_status(idx, 'skipped')

    def _update_scenario_summary(self) -> None:
        counts = {'completed': 0, 'failed': 0, 'in_progress': 0, 'pending': 0, 'skipped': 0}
        for task in self.tasks.values():
            status = task.get('status', 'pending')
            counts[status] = counts.get(status, 0) + 1
        waiting = counts['pending'] + counts['skipped']
        summary = f"완료 {counts['completed']} · 실패 {counts['failed']} · 진행중 {counts['in_progress']} · 대기 {waiting}"
        self._scenario_summary_var.set(f'진행 단계: {summary}')

    def _describe_scenario_step(self, step: Dict[str, object]) -> str:
        action = str(step.get('action', '')).lower()
        if not action:
            return '미정의 단계'

        if action == 'call_service':
            args = step.get('args', {})
            args = args if isinstance(args, dict) else {}
            target = step.get('target') or args.get('target')
            return f'service:{target}'

        if action == 'wait_for_state':
            args = step.get('args', {})
            args = args if isinstance(args, dict) else {}
            main = args.get('main', '*')
            sub = args.get('sub', '*')
            return f'wait:{main}/{sub}'

        if action == 'sleep':
            duration = step.get('seconds', step.get('duration', 0))
            try:
                duration = float(duration)
            except (TypeError, ValueError):
                duration = 0.0
            return f'sleep:{duration:.1f}s'

        if action == 'publish_mock':
            return f'mock:{step.get("target", "")}'

        if action == 'log':
            message = str(step.get('message', ''))
            return f'log:{message[:24]}'

        return action

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
