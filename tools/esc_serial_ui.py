import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except Exception as exc:
    raise SystemExit(
        "pyserial is required.\n"
        "Install with: py -m pip install --user pyserial\n"
        f"Import error: {exc}"
    )


RE_THR_ACK = re.compile(r"^\[CMD\]\s+thr\s+(\d+)\s+(-?\d+)\s+->\s+thr\s+->\s+(\d+)$")
RE_THR_ALL_ACK = re.compile(r"^\[CMD\]\s+thr_all\s+(-?\d+)\s+->\s+thr_all\s+->\s+(\d+)$")


class EscSerialUi:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ESC Motor Control (COM) v4")
        self.root.geometry("1280x820")
        self.root.minsize(1120, 720)

        self.serial_port = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self.stream_job = None
        self.status_poll_job = None
        self.ramp_job = None
        self.keepalive_job = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.safety_var = tk.BooleanVar(value=False)
        self.streaming_var = tk.BooleanVar(value=False)
        self.compact_log_var = tk.BooleanVar(value=True)
        self.show_rx_log_var = tk.BooleanVar(value=False)
        self.show_thr_ack_log_var = tk.BooleanVar(value=False)
        self.keepalive_var = tk.BooleanVar(value=True)
        self.stream_ms_var = tk.IntVar(value=120)

        self.master_raw_var = tk.IntVar(value=300)
        self.motor_count = 1
        self.thr_ack_count = 0
        self.max_log_lines = 1500
        self._log_line_counter = 0
        self._log_trim_interval = 40

        self.motor_enabled_vars = [tk.BooleanVar(value=(i == 0)) for i in range(4)]
        self.motor_raw_vars = [tk.IntVar(value=0) for _ in range(4)]
        self.motor_scale_widgets: list[tk.Scale] = []
        self.motor_send_buttons: list[ttk.Button] = []
        self.motor_avail_labels: list[ttk.Label] = []

        self.pt_motor_var = tk.IntVar(value=0)
        self.pt_addr_var = tk.IntVar(value=0)
        self.pt_len_var = tk.IntVar(value=16)
        self.pt_write_var = tk.StringVar(value="1 2 3 4")
        self.pt_motor_spin = None

        self.custom_cmd_var = tk.StringVar(value="status")

        self.graph_canvas = None
        self.graph_window_s = 20.0
        self.graph_colors = ["#ff5d5d", "#45c1ff", "#7bdc65", "#ffd166"]
        self.graph_samples = deque(maxlen=2500)
        self.graph_latest = [0, 0, 0, 0]
        self._graph_redraw_job = None
        self._graph_dirty = False
        self._graph_min_redraw_interval_ms = 33
        self._graph_last_draw_ms = 0.0

        self.ramp_running = False
        self.ramp_state_var = tk.StringVar(value="Idle")
        self.ramp_min_var = tk.IntVar(value=150)
        self.ramp_max_var = tk.IntVar(value=900)
        self.ramp_step_var = tk.IntVar(value=30)
        self.ramp_interval_var = tk.IntVar(value=90)
        self.ramp_hold_var = tk.IntVar(value=300)
        self._ramp_enabled: list[int] = []
        self._ramp_index = 0
        self._ramp_value = 0
        self._ramp_dir = 1
        self._ramp_min_raw = 150
        self._ramp_max_raw = 900
        self._ramp_step_raw = 30
        self._ramp_interval_ms = 90
        self._ramp_hold_ms = 300
        self._ramp_hold_until = 0.0
        self._ramp_has_peaked = False

        self.status_values = {
            "conn": tk.StringVar(value="Disconnected"),
            "armed": tk.StringVar(value="-"),
            "autoarm": tk.StringVar(value="-"),
            "passthrough": tk.StringVar(value="-"),
            "motors": tk.StringVar(value="-"),
            "timeout": tk.StringVar(value="-"),
            "refresh": tk.StringVar(value="-"),
            "thr_acks": tk.StringVar(value="0"),
            "curr_a": tk.StringVar(value="-"),
            "curr_mv": tk.StringVar(value="-"),
            "curr_raw": tk.StringVar(value="-"),
            "rmt_err_code": tk.StringVar(value="-"),
            "rmt_err_motor": tk.StringVar(value="-"),
            "rmt_err_pin": tk.StringVar(value="-"),
        }

        self.motor_tx_counts = [0, 0, 0, 0]
        self.motor_ack_counts = [0, 0, 0, 0]
        self.motor_last_ack_code = ["-", "-", "-", "-"]
        self.motor_last_ack_ts = [0.0, 0.0, 0.0, 0.0]
        self.motor_tx_vars = [tk.StringVar(value="0") for _ in range(4)]
        self.motor_ack_vars = [tk.StringVar(value="0") for _ in range(4)]
        self.motor_code_vars = [tk.StringVar(value="-") for _ in range(4)]
        self.motor_age_vars = [tk.StringVar(value="-") for _ in range(4)]
        self.motor_link_vars = [tk.IntVar(value=0) for _ in range(4)]
        self.link_job = None

        self._build_ui()
        self._reset_motor_comm()
        self._refresh_ports()
        self._schedule_ui_pump()
        self._schedule_status_poll()
        self._schedule_keepalive()
        self._schedule_link_health()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.colors = {
            "bg": "#0b1220",
            "panel": "#121b2d",
            "panel2": "#17243a",
            "border": "#243852",
            "text": "#e6edf7",
            "muted": "#93a6c2",
            "accent": "#35b6ff",
            "accent2": "#1c8cd8",
            "ok": "#2ecc71",
            "warn": "#f5b942",
        }

        self.root.configure(bg=self.colors["bg"])
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure(".", background=self.colors["bg"], foreground=self.colors["text"])
        style.configure("TFrame", background=self.colors["bg"])
        style.configure("TLabelframe", background=self.colors["panel"], bordercolor=self.colors["border"], relief="solid", borderwidth=1)
        style.configure("TLabelframe.Label", background=self.colors["panel"], foreground=self.colors["text"])
        style.configure("TLabel", background=self.colors["bg"], foreground=self.colors["text"])
        style.configure("TButton", background=self.colors["panel2"], foreground=self.colors["text"], bordercolor=self.colors["border"])
        style.map("TButton", background=[("active", self.colors["accent2"])], foreground=[("active", "#ffffff")])
        style.configure("Accent.TButton", background=self.colors["accent"], foreground="#06101f", bordercolor=self.colors["accent2"])
        style.map("Accent.TButton", background=[("active", "#63c9ff")], foreground=[("active", "#03111f")])
        style.configure("TCheckbutton", background=self.colors["panel"], foreground=self.colors["text"])
        style.configure("TEntry", fieldbackground=self.colors["panel2"], foreground=self.colors["text"])
        style.configure("TCombobox", fieldbackground=self.colors["panel2"], foreground=self.colors["text"])
        style.configure("TSpinbox", fieldbackground=self.colors["panel2"], foreground=self.colors["text"])
        style.configure("Horizontal.TProgressbar", troughcolor="#1a2740", background=self.colors["accent"], bordercolor=self.colors["border"])
        style.configure("TNotebook", background=self.colors["bg"], borderwidth=0)
        style.configure("TNotebook.Tab", background=self.colors["panel2"], foreground=self.colors["muted"], padding=(12, 6))
        style.map("TNotebook.Tab", background=[("selected", self.colors["accent2"])], foreground=[("selected", "#ffffff")])

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, width=14, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Label(top, text="Baud").grid(row=0, column=2, sticky="w")
        ttk.Entry(top, width=9, textvariable=self.baud_var).grid(row=0, column=3, padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).grid(row=0, column=4, padx=4)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, padx=4)
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        status = ttk.LabelFrame(self.root, text="Status", padding=8)
        status.pack(fill=tk.X, padx=8, pady=4)
        fields = [
            ("Connection", "conn"),
            ("Armed", "armed"),
            ("AutoArm", "autoarm"),
            ("Passthrough", "passthrough"),
            ("Motors", "motors"),
            ("Timeout(ms)", "timeout"),
            ("Refresh(ms)", "refresh"),
            ("Thr ACKs", "thr_acks"),
            ("Curr(A)", "curr_a"),
            ("Curr(mV)", "curr_mv"),
            ("CurrRaw", "curr_raw"),
            ("RMT Err", "rmt_err_code"),
            ("RMT Motor", "rmt_err_motor"),
            ("RMT Pin", "rmt_err_pin"),
        ]
        for idx, (label, key) in enumerate(fields):
            row = idx // 6
            c = (idx % 6) * 2
            ttk.Label(status, text=label + ":").grid(row=row, column=c, sticky="e", padx=(0, 4))
            ttk.Label(status, textvariable=self.status_values[key], width=10).grid(row=row, column=c + 1, sticky="w", padx=(0, 12))

        comm = ttk.LabelFrame(self.root, text="Motor Communication", padding=8)
        comm.pack(fill=tk.X, padx=8, pady=4)
        for i in range(4):
            row = ttk.Frame(comm)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f"M{i + 1}", width=3).pack(side=tk.LEFT)
            ttk.Progressbar(row, maximum=100, variable=self.motor_link_vars[i], length=160).pack(side=tk.LEFT, padx=(0, 8))
            ttk.Label(row, text="TX").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.motor_tx_vars[i], width=7).pack(side=tk.LEFT)
            ttk.Label(row, text="ACK").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.motor_ack_vars[i], width=7).pack(side=tk.LEFT)
            ttk.Label(row, text="Code").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.motor_code_vars[i], width=6).pack(side=tk.LEFT)
            ttk.Label(row, text="Age").pack(side=tk.LEFT)
            ttk.Label(row, textvariable=self.motor_age_vars[i], width=10).pack(side=tk.LEFT)

        safety = ttk.LabelFrame(self.root, text="Safety", padding=8)
        safety.pack(fill=tk.X, padx=8, pady=4)
        ttk.Checkbutton(
            safety,
            text="I confirm propellers are removed and bench test area is clear",
            variable=self.safety_var,
        ).pack(anchor="w")

        actions = ttk.LabelFrame(self.root, text="Quick Actions", padding=8)
        actions.pack(fill=tk.X, padx=8, pady=4)
        ttk.Button(actions, text="Arm", style="Accent.TButton", command=self._arm).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Disarm", command=self._disarm).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Stop All", command=self._stop_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Status", command=lambda: self._send_cmd("status")).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Read Current", command=lambda: self._send_cmd("curr")).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="AutoArm ON", command=lambda: self._send_cmd("autoarm on")).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="AutoArm OFF", command=lambda: self._send_cmd("autoarm off")).pack(side=tk.LEFT, padx=4)

        motors = ttk.LabelFrame(self.root, text="Motors (Betaflight-style test panel)", padding=8)
        motors.pack(fill=tk.X, padx=8, pady=4)
        motors_body = ttk.Frame(motors)
        motors_body.pack(fill=tk.X)
        motors_body.columnconfigure(0, weight=3)
        motors_body.columnconfigure(1, weight=2)

        deck = ttk.Frame(motors_body)
        deck.grid(row=0, column=0, sticky="nw")
        for i in range(4):
            col = ttk.LabelFrame(deck, text=f"M{i + 1}", padding=6)
            col.grid(row=0, column=i, padx=6, sticky="n")
            ttk.Checkbutton(col, text="Enable", variable=self.motor_enabled_vars[i]).pack(anchor="w")
            scale = tk.Scale(
                col,
                from_=2047,
                to=0,
                orient=tk.VERTICAL,
                length=210,
                resolution=1,
                variable=self.motor_raw_vars[i],
                showvalue=False,
                bg=self.colors["panel2"],
                fg=self.colors["text"],
                troughcolor="#23395a",
                highlightthickness=0,
                bd=0,
                relief=tk.FLAT,
                activebackground=self.colors["accent"],
            )
            scale.pack(pady=4)
            self.motor_scale_widgets.append(scale)
            ttk.Label(col, textvariable=self.motor_raw_vars[i], width=6, anchor="center").pack()
            send_btn = ttk.Button(col, text="Send", command=lambda idx=i: self._send_motor_once(idx))
            send_btn.pack(pady=4)
            self.motor_send_buttons.append(send_btn)
            avail = ttk.Label(col, text="", foreground=self.colors["muted"])
            avail.pack()
            self.motor_avail_labels.append(avail)

        motor_right = ttk.LabelFrame(motors_body, text="Test + Graph", padding=8)
        motor_right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self.graph_canvas = tk.Canvas(
            motor_right,
            width=430,
            height=260,
            background="#07111f",
            highlightthickness=1,
            highlightbackground="#24405f",
        )
        self.graph_canvas.pack(fill=tk.X)
        self.graph_canvas.bind("<Configure>", lambda _evt: self._request_graph_redraw(force=True))

        legend = ttk.Frame(motor_right)
        legend.pack(fill=tk.X, pady=(6, 2))
        for i in range(4):
            ttk.Label(legend, text=f"M{i + 1}", foreground=self.graph_colors[i]).pack(side=tk.LEFT, padx=6)

        ramp_cfg = ttk.Frame(motor_right)
        ramp_cfg.pack(fill=tk.X, pady=(6, 2))
        ttk.Label(ramp_cfg, text="Min").grid(row=0, column=0, sticky="w")
        ttk.Entry(ramp_cfg, width=6, textvariable=self.ramp_min_var).grid(row=0, column=1, padx=4)
        ttk.Label(ramp_cfg, text="Max").grid(row=0, column=2, sticky="w")
        ttk.Entry(ramp_cfg, width=6, textvariable=self.ramp_max_var).grid(row=0, column=3, padx=4)
        ttk.Label(ramp_cfg, text="Step").grid(row=1, column=0, sticky="w")
        ttk.Entry(ramp_cfg, width=6, textvariable=self.ramp_step_var).grid(row=1, column=1, padx=4)
        ttk.Label(ramp_cfg, text="Int ms").grid(row=1, column=2, sticky="w")
        ttk.Entry(ramp_cfg, width=6, textvariable=self.ramp_interval_var).grid(row=1, column=3, padx=4)
        ttk.Label(ramp_cfg, text="Hold").grid(row=2, column=0, sticky="w")
        ttk.Entry(ramp_cfg, width=6, textvariable=self.ramp_hold_var).grid(row=2, column=1, padx=4)

        ramp_btns = ttk.Frame(motor_right)
        ramp_btns.pack(fill=tk.X, pady=(6, 2))
        ttk.Button(ramp_btns, text="Start Test", style="Accent.TButton", command=self._start_ramp_test).pack(side=tk.LEFT, padx=2)
        ttk.Button(ramp_btns, text="Stop Test", command=self._stop_ramp_test).pack(side=tk.LEFT, padx=2)
        ttk.Button(ramp_btns, text="Clear Graph", command=self._clear_graph).pack(side=tk.LEFT, padx=2)
        ramp_state_row = ttk.Frame(motor_right)
        ramp_state_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(ramp_state_row, text="State:").pack(side=tk.LEFT)
        ttk.Label(ramp_state_row, textvariable=self.ramp_state_var).pack(side=tk.LEFT, padx=6)

        tabs = ttk.Notebook(self.root)
        tabs.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        throttle_tab = ttk.Frame(tabs, padding=8)
        passthrough_tab = ttk.Frame(tabs, padding=8)
        console_tab = ttk.Frame(tabs, padding=8)
        tabs.add(throttle_tab, text="Throttle")
        tabs.add(passthrough_tab, text="Passthrough")
        tabs.add(console_tab, text="Console")

        stream = ttk.LabelFrame(throttle_tab, text="Master + Streaming", padding=8)
        stream.pack(fill=tk.X)
        master_row = ttk.Frame(stream)
        master_row.pack(fill=tk.X, pady=2)
        ttk.Label(master_row, text="Master Raw").pack(side=tk.LEFT)
        ttk.Scale(master_row, from_=0, to=2047, orient=tk.HORIZONTAL, variable=self.master_raw_var, length=360).pack(side=tk.LEFT, padx=8)
        ttk.Label(master_row, textvariable=self.master_raw_var, width=6).pack(side=tk.LEFT)
        ttk.Button(master_row, text="Apply to Enabled Motors", command=self._apply_master_to_enabled).pack(side=tk.LEFT, padx=8)
        ttk.Button(master_row, text="Send All Now (thr_all)", command=self._send_all_once).pack(side=tk.LEFT, padx=8)

        preset_row = ttk.Frame(stream)
        preset_row.pack(fill=tk.X, pady=2)
        ttk.Label(preset_row, text="Presets:").pack(side=tk.LEFT)
        for val in (150, 300, 600, 1000, 1500, 2000):
            ttk.Button(preset_row, text=str(val), command=lambda v=val: self.master_raw_var.set(v)).pack(side=tk.LEFT, padx=3)

        stream_row = ttk.Frame(stream)
        stream_row.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(stream_row, text="Live Stream", variable=self.streaming_var, command=self._toggle_stream).pack(side=tk.LEFT)
        ttk.Label(stream_row, text="Interval ms").pack(side=tk.LEFT, padx=(16, 4))
        ttk.Spinbox(stream_row, from_=20, to=1000, width=8, textvariable=self.stream_ms_var).pack(side=tk.LEFT)
        ttk.Checkbutton(stream_row, text="Compact throttle log", variable=self.compact_log_var).pack(side=tk.LEFT, padx=(16, 4))
        ttk.Checkbutton(stream_row, text="Show RX log", variable=self.show_rx_log_var).pack(side=tk.LEFT, padx=(16, 4))
        ttk.Checkbutton(stream_row, text="Show thr ACK lines", variable=self.show_thr_ack_log_var).pack(side=tk.LEFT, padx=(8, 4))
        ttk.Checkbutton(stream_row, text="Arm keepalive (0-thr)", variable=self.keepalive_var).pack(side=tk.LEFT, padx=(8, 4))

        pt = ttk.LabelFrame(passthrough_tab, text="Passthrough (advanced)", padding=8)
        pt.pack(fill=tk.X)
        row1 = ttk.Frame(pt)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="Motor").pack(side=tk.LEFT)
        self.pt_motor_spin = ttk.Spinbox(row1, from_=0, to=3, width=5, textvariable=self.pt_motor_var)
        self.pt_motor_spin.pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text="Enter", command=self._pt_enter).pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text="Exit", command=lambda: self._send_cmd("pt_exit")).pack(side=tk.LEFT, padx=4)

        row2 = ttk.Frame(pt)
        row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="Addr").pack(side=tk.LEFT)
        ttk.Entry(row2, width=8, textvariable=self.pt_addr_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(row2, text="Len").pack(side=tk.LEFT)
        ttk.Entry(row2, width=6, textvariable=self.pt_len_var).pack(side=tk.LEFT, padx=4)
        ttk.Button(row2, text="Read", command=self._pt_read).pack(side=tk.LEFT, padx=4)

        row3 = ttk.Frame(pt)
        row3.pack(fill=tk.X, pady=2)
        ttk.Label(row3, text="Write bytes").pack(side=tk.LEFT)
        ttk.Entry(row3, width=36, textvariable=self.pt_write_var).pack(side=tk.LEFT, padx=4)
        ttk.Button(row3, text="Write", command=self._pt_write).pack(side=tk.LEFT, padx=4)
        ttk.Button(row3, text="Erase", command=self._pt_erase).pack(side=tk.LEFT, padx=4)

        console_top = ttk.Frame(console_tab)
        console_top.pack(fill=tk.X, pady=(0, 6))
        ttk.Entry(console_top, textvariable=self.custom_cmd_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ttk.Button(console_top, text="Send", command=self._send_custom).pack(side=tk.LEFT, padx=4)
        ttk.Button(console_top, text="Clear Log", command=self._clear_log).pack(side=tk.LEFT, padx=4)

        log_frame = ttk.LabelFrame(console_tab, text="Serial Log", padding=6)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log = tk.Text(
            log_frame,
            wrap=tk.WORD,
            height=12,
            bg="#07111f",
            fg="#d5e9ff",
            insertbackground="#ffffff",
            relief=tk.FLAT,
            borderwidth=0,
        )
        self.log.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        yscroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log.yview)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.configure(yscrollcommand=yscroll.set)

        self._apply_motor_availability()
        self._request_graph_redraw(force=True)

    def _log(self, msg: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{stamp}] {msg}\n")
        self.log.see(tk.END)
        self._log_line_counter += 1
        if self._log_line_counter >= self._log_trim_interval:
            self._log_line_counter = 0
            line_count = int(self.log.index("end-1c").split(".")[0])
            if line_count > self.max_log_lines:
                delete_to = line_count - self.max_log_lines
                self.log.delete("1.0", f"{delete_to}.0")

    def _clear_log(self) -> None:
        self.log.delete("1.0", tk.END)
        self._log_line_counter = 0

    def _reset_motor_comm(self) -> None:
        self.motor_tx_counts = [0, 0, 0, 0]
        self.motor_ack_counts = [0, 0, 0, 0]
        self.motor_last_ack_code = ["-", "-", "-", "-"]
        self.motor_last_ack_ts = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            self.motor_tx_vars[i].set("0")
            self.motor_ack_vars[i].set("0")
            self.motor_code_vars[i].set("-")
            self.motor_age_vars[i].set("-")
            self.motor_link_vars[i].set(0)

    def _mark_motor_tx(self, motor: int) -> None:
        if motor < 0 or motor >= 4:
            return
        self.motor_tx_counts[motor] += 1
        self.motor_tx_vars[motor].set(str(self.motor_tx_counts[motor]))

    def _mark_motor_ack(self, motor: int, code: int) -> None:
        if motor < 0 or motor >= 4:
            return
        self.motor_ack_counts[motor] += 1
        self.motor_ack_vars[motor].set(str(self.motor_ack_counts[motor]))
        self.motor_last_ack_code[motor] = str(code)
        self.motor_code_vars[motor].set(str(code))
        self.motor_last_ack_ts[motor] = time.time()
        self.motor_link_vars[motor].set(100)
        self.motor_age_vars[motor].set("0 ms")

    def _schedule_link_health(self) -> None:
        now = time.time()
        for i in range(4):
            if i >= self.motor_count:
                self.motor_link_vars[i].set(0)
                self.motor_age_vars[i].set("N/A")
                continue

            last_ts = self.motor_last_ack_ts[i]
            if last_ts <= 0:
                self.motor_link_vars[i].set(0)
                self.motor_age_vars[i].set("-")
                continue

            age = now - last_ts
            self.motor_age_vars[i].set(f"{int(age * 1000)} ms")
            if age < 0.2:
                value = 100
            elif age < 0.5:
                value = 80
            elif age < 1.0:
                value = 60
            elif age < 2.0:
                value = 40
            elif age < 4.0:
                value = 20
            else:
                value = 0
            self.motor_link_vars[i].set(value)

        self.link_job = self.root.after(200, self._schedule_link_health)

    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self._log(f"Ports: {ports if ports else 'none'}")

    def _set_connected(self, connected: bool) -> None:
        self.status_values["conn"].set("Connected" if connected else "Disconnected")
        self.connect_btn.configure(text="Disconnect" if connected else "Connect")

    def _toggle_connect(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Missing port", "Select a COM port first.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud must be an integer.")
            return
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0.2, rtscts=False, dsrdtr=False)
            self.serial_port.setDTR(False)
            self.serial_port.setRTS(False)
            self.serial_port.reset_input_buffer()
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            return

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self._set_connected(True)
        self._reset_motor_comm()
        self.thr_ack_count = 0
        self.status_values["thr_acks"].set("0")
        self._log(f"Connected: {port} @ {baud}")
        self.root.after(900, lambda: self._send_cmd("status", echo=True))

    def _disconnect(self) -> None:
        self._stop_ramp_test(force=True)
        self.streaming_var.set(False)
        self._toggle_stream()

        self.reader_stop.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        self.reader_thread = None

        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self._reset_motor_comm()
        self._set_connected(False)
        self._log("Disconnected")

    def _reader_loop(self) -> None:
        while not self.reader_stop.is_set():
            try:
                if self.serial_port and self.serial_port.is_open:
                    line = self.serial_port.readline()
                    if line:
                        text = line.decode(errors="replace").strip()
                        if self.rx_queue.qsize() <= 5000:
                            self.rx_queue.put(text)
                else:
                    time.sleep(0.1)
            except Exception as exc:
                self.rx_queue.put(f"[reader error] {exc}")
                time.sleep(0.2)

    def _schedule_ui_pump(self) -> None:
        processed = 0
        max_per_tick = 320
        while processed < max_per_tick:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_rx_line(line)
            processed += 1
        next_ms = 8 if processed >= max_per_tick else 50
        self.root.after(next_ms, self._schedule_ui_pump)

    def _schedule_status_poll(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            self._send_cmd("status", echo=False)
        self.status_poll_job = self.root.after(500, self._schedule_status_poll)

    def _schedule_keepalive(self) -> None:
        if (
            self.keepalive_var.get()
            and self.serial_port
            and self.serial_port.is_open
            and not self.streaming_var.get()
            and not self.ramp_running
            and self.status_values["armed"].get() == "1"
            and self.status_values["passthrough"].get() == "0"
        ):
            # Keep armed state alive safely by streaming explicit zero throttle.
            self._send_cmd("thr_all 0", echo=False, track=False)
            self._record_graph([0, 0, 0, 0])
        self.keepalive_job = self.root.after(250, self._schedule_keepalive)

    def _handle_rx_line(self, line: str) -> None:
        is_thr_line = line.startswith("[CMD] thr ") or line.startswith("[CMD] thr_all ")
        if is_thr_line:
            self.thr_ack_count += 1
            self.status_values["thr_acks"].set(str(self.thr_ack_count))

            m = RE_THR_ACK.match(line)
            if m:
                motor = int(m.group(1))
                code = int(m.group(3))
                self._mark_motor_ack(motor, code)
            else:
                m = RE_THR_ALL_ACK.match(line)
                if m:
                    code = int(m.group(2))
                    for i in range(min(self.motor_count, 4)):
                        self._mark_motor_ack(i, code)

            if self.compact_log_var.get() and not self.show_thr_ack_log_var.get():
                return

        should_log_rx = (
            self.show_rx_log_var.get()
            or line.startswith("[RMT]")
            or line.startswith("[BOOT]")
            or line.startswith("[reader error]")
            or "ERR" in line
        )
        if should_log_rx:
            self._log(f"RX: {line}")

        payload = None
        marker = "[CMD] status -> "
        if marker in line:
            payload = line.split(marker, 1)[1].strip()
        elif "motors=" in line and "armed=" in line:
            payload = line.strip()
        if payload:
            self._apply_status_payload(payload)

    def _apply_status_payload(self, payload: str) -> None:
        values = {}
        for token in payload.replace(",", " ").split():
            if "=" in token:
                key, val = token.split("=", 1)
                values[key.strip()] = val.strip()

        if "motors" in values:
            try:
                self.motor_count = max(1, min(4, int(values["motors"])))
            except ValueError:
                pass
            self.status_values["motors"].set(str(self.motor_count))
            self._apply_motor_availability()
        if "armed" in values:
            self.status_values["armed"].set(values["armed"])
        if "autoarm" in values:
            self.status_values["autoarm"].set(values["autoarm"])
        if "passthrough" in values:
            self.status_values["passthrough"].set(values["passthrough"])
        if "timeout_ms" in values:
            self.status_values["timeout"].set(values["timeout_ms"])
        if "refresh_ms" in values:
            self.status_values["refresh"].set(values["refresh_ms"])
        if "curr_a" in values:
            self.status_values["curr_a"].set(values["curr_a"])
        if "curr_mv" in values:
            self.status_values["curr_mv"].set(values["curr_mv"])
        if "curr_raw" in values:
            self.status_values["curr_raw"].set(values["curr_raw"])
        if "rmt_err_code" in values:
            self.status_values["rmt_err_code"].set(values["rmt_err_code"])
        if "rmt_err_motor" in values:
            self.status_values["rmt_err_motor"].set(values["rmt_err_motor"])
        if "rmt_err_pin" in values:
            self.status_values["rmt_err_pin"].set(values["rmt_err_pin"])

    def _apply_motor_availability(self) -> None:
        for i in range(4):
            available = i < self.motor_count
            tk_state = "normal" if available else "disabled"
            btn_state = tk.NORMAL if available else tk.DISABLED
            self.motor_scale_widgets[i].configure(state=tk_state)
            self.motor_send_buttons[i].configure(state=btn_state)
            if available:
                self.motor_avail_labels[i].configure(text="")
            else:
                self.motor_enabled_vars[i].set(False)
                self.motor_raw_vars[i].set(0)
                self.motor_avail_labels[i].configure(text="N/A")

        if self.pt_motor_spin is not None:
            self.pt_motor_spin.configure(to=max(0, self.motor_count - 1))
            if self.pt_motor_var.get() >= self.motor_count:
                self.pt_motor_var.set(max(0, self.motor_count - 1))
        self._request_graph_redraw()

    def _ensure_safety(self, action: str) -> bool:
        if self.safety_var.get():
            return True
        messagebox.showwarning("Safety Check", f"Cannot {action} until you confirm props are removed.")
        self._log(f"Blocked {action}: safety checkbox is not enabled")
        return False

    def _track_tx_command(self, cmd: str) -> None:
        parts = cmd.strip().split()
        if not parts:
            return
        if parts[0] == "thr" and len(parts) >= 3:
            try:
                motor = int(parts[1])
                raw = max(0, min(2047, int(parts[2])))
            except ValueError:
                return
            if 0 <= motor < 4:
                self._mark_motor_tx(motor)
                values = self.graph_latest[:]
                values[motor] = raw
                self._record_graph(values)
            return
        if parts[0] == "thr_all" and len(parts) >= 2:
            try:
                raw = max(0, min(2047, int(parts[1])))
            except ValueError:
                return
            for i in range(min(self.motor_count, 4)):
                self._mark_motor_tx(i)
            self._record_graph([raw if i < self.motor_count else 0 for i in range(4)])
            return
        if parts[0] in {"stop", "disarm"}:
            for i in range(min(self.motor_count, 4)):
                self._mark_motor_tx(i)
            self._record_graph([0, 0, 0, 0])

    def _send_cmd(self, cmd: str, echo: bool = True, track: bool = True) -> None:
        if not (self.serial_port and self.serial_port.is_open):
            if echo:
                self._log("ERR not connected")
            return
        clean = cmd.strip()
        try:
            self.serial_port.write((clean + "\n").encode())
            if track:
                self._track_tx_command(clean)
            if echo:
                self._log(f"TX: {clean}")
        except Exception as exc:
            self._log(f"TX error: {exc}")

    def _send_motor_values(self, values: list[int], echo: bool = False) -> None:
        if not (self.serial_port and self.serial_port.is_open):
            return
        active_count = min(self.motor_count, 4)
        if active_count <= 0:
            return
        clamped = [max(0, min(2047, int(values[i]))) for i in range(active_count)]
        if all(v == clamped[0] for v in clamped):
            lines = [f"thr_all {clamped[0]}"]
            for i in range(active_count):
                self._mark_motor_tx(i)
        else:
            lines = [f"thr {i} {clamped[i]}" for i in range(active_count)]
            for i in range(active_count):
                self._mark_motor_tx(i)
        payload = ("\n".join(lines) + "\n").encode()
        try:
            self.serial_port.write(payload)
            if echo:
                self._log(f"TX(batch): {' | '.join(lines)}")
        except Exception as exc:
            self._log(f"TX error: {exc}")

    def _arm(self) -> None:
        if not self._ensure_safety("arm"):
            return
        self._send_cmd("arm")

    def _disarm(self) -> None:
        self._stop_ramp_test(force=True)
        self._send_cmd("disarm")

    def _stop_all(self) -> None:
        self._stop_ramp_test(force=True)
        self._send_cmd("stop")

    def _send_motor_once(self, motor: int) -> None:
        if not self._ensure_safety("send throttle"):
            return
        if motor >= self.motor_count:
            self._log(f"Motor {motor} unavailable (firmware motors={self.motor_count})")
            return
        raw = int(self.motor_raw_vars[motor].get())
        self._send_cmd(f"thr {motor} {raw}")

    def _send_all_once(self) -> None:
        if not self._ensure_safety("send all motors"):
            return
        raw = int(self.master_raw_var.get())
        self._send_cmd(f"thr_all {raw}")

    def _apply_master_to_enabled(self) -> None:
        raw = int(self.master_raw_var.get())
        for i in range(min(self.motor_count, 4)):
            if self.motor_enabled_vars[i].get():
                self.motor_raw_vars[i].set(raw)
        self._log(f"Master raw {raw} applied to enabled motors")

    def _toggle_stream(self) -> None:
        if self.stream_job is not None:
            self.root.after_cancel(self.stream_job)
            self.stream_job = None
        if not self.streaming_var.get():
            self._log("Streaming: OFF")
            return
        if self.ramp_running:
            self._stop_ramp_test(force=True)
        if not self._ensure_safety("start streaming"):
            self.streaming_var.set(False)
            return
        self._log("Streaming: ON")
        self._stream_tick()

    def _stream_tick(self) -> None:
        if not self.streaming_var.get():
            return
        values = [0, 0, 0, 0]
        for i in range(min(self.motor_count, 4)):
            raw = int(self.motor_raw_vars[i].get()) if self.motor_enabled_vars[i].get() else 0
            values[i] = raw
        self._send_motor_values(values, echo=False)
        self._record_graph(values)
        interval = max(20, int(self.stream_ms_var.get()))
        self.stream_job = self.root.after(interval, self._stream_tick)

    def _start_ramp_test(self) -> None:
        if self.ramp_running:
            self._log("Ramp test already running")
            return
        if not self._ensure_safety("start ramp test"):
            return
        if not (self.serial_port and self.serial_port.is_open):
            self._log("ERR not connected")
            return
        enabled = [i for i in range(min(self.motor_count, 4)) if self.motor_enabled_vars[i].get()]
        if not enabled:
            self._log("ERR enable at least one motor before ramp test")
            return

        self._ramp_min_raw = max(0, min(2047, int(self.ramp_min_var.get())))
        self._ramp_max_raw = max(0, min(2047, int(self.ramp_max_var.get())))
        if self._ramp_min_raw > self._ramp_max_raw:
            self._ramp_min_raw, self._ramp_max_raw = self._ramp_max_raw, self._ramp_min_raw
        self._ramp_step_raw = max(1, int(self.ramp_step_var.get()))
        self._ramp_interval_ms = max(20, int(self.ramp_interval_var.get()))
        self._ramp_hold_ms = max(0, int(self.ramp_hold_var.get()))

        if self.streaming_var.get():
            self.streaming_var.set(False)
            self._toggle_stream()

        self._send_cmd("arm", echo=False)
        self._send_cmd("thr_all 0", echo=False, track=False)
        for i in range(4):
            self.motor_raw_vars[i].set(0)

        self._ramp_enabled = enabled
        self._ramp_value = self._ramp_min_raw
        self._ramp_dir = 1
        self._ramp_hold_until = 0.0
        self._ramp_has_peaked = False
        self.ramp_running = True
        self.ramp_state_var.set("Running")
        self._log(
            f"Ramp test (simultaneous): motors={','.join(str(i + 1) for i in enabled)} "
            f"min={self._ramp_min_raw} max={self._ramp_max_raw} step={self._ramp_step_raw} "
            f"interval={self._ramp_interval_ms}ms"
        )
        self._ramp_tick()

    def _stop_ramp_test(self, force: bool = False) -> None:
        was_running = self.ramp_running
        self.ramp_running = False
        self._ramp_has_peaked = False
        self.ramp_state_var.set("Idle")
        if self.ramp_job is not None:
            self.root.after_cancel(self.ramp_job)
            self.ramp_job = None
        if was_running and self.serial_port and self.serial_port.is_open:
            self._send_cmd("thr_all 0", echo=False, track=False)
            for i in range(4):
                self.motor_raw_vars[i].set(0)
            self._record_graph([0, 0, 0, 0])
            if not force:
                self._log("Ramp test: stopped")

    def _ramp_tick(self) -> None:
        if not self.ramp_running:
            return
        now = time.time()
        values = [0, 0, 0, 0]
        for motor in self._ramp_enabled:
            values[motor] = self._ramp_value
        for i in range(min(self.motor_count, 4)):
            self.motor_raw_vars[i].set(values[i])
        self._send_motor_values(values, echo=False)
        self._record_graph(values)

        if self._ramp_hold_until > now:
            self.ramp_job = self.root.after(self._ramp_interval_ms, self._ramp_tick)
            return

        next_value = self._ramp_value + (self._ramp_step_raw * self._ramp_dir)
        if self._ramp_dir > 0 and next_value >= self._ramp_max_raw:
            self._ramp_value = self._ramp_max_raw
            self._ramp_dir = -1
            self._ramp_has_peaked = True
            self._ramp_hold_until = now + (self._ramp_hold_ms / 1000.0)
        elif self._ramp_dir < 0 and next_value <= self._ramp_min_raw:
            self._ramp_value = self._ramp_min_raw
            if self._ramp_has_peaked:
                self.ramp_running = False
                self.ramp_state_var.set("Done")
                self._send_cmd("thr_all 0", echo=False, track=False)
                for i in range(4):
                    self.motor_raw_vars[i].set(0)
                self._record_graph([0, 0, 0, 0])
                self._log("Ramp test: complete")
                return
            self._ramp_dir = 1
            self._ramp_hold_until = now + (self._ramp_hold_ms / 1000.0)
        else:
            self._ramp_value = next_value
        self.ramp_job = self.root.after(self._ramp_interval_ms, self._ramp_tick)

    def _record_graph(self, values: list[int]) -> None:
        clamped = [max(0, min(2047, int(v))) for v in values[:4]]
        while len(clamped) < 4:
            clamped.append(0)
        now = time.time()
        self.graph_latest = clamped
        self.graph_samples.append((now, clamped[:]))
        cutoff = now - self.graph_window_s
        while self.graph_samples and self.graph_samples[0][0] < cutoff:
            self.graph_samples.popleft()
        self._request_graph_redraw()

    def _clear_graph(self) -> None:
        self.graph_samples.clear()
        self.graph_latest = [0, 0, 0, 0]
        self._request_graph_redraw(force=True)

    def _request_graph_redraw(self, force: bool = False) -> None:
        now_ms = time.perf_counter() * 1000.0
        self._graph_dirty = True
        if force or (now_ms - self._graph_last_draw_ms) >= self._graph_min_redraw_interval_ms:
            if self._graph_redraw_job is not None:
                self.root.after_cancel(self._graph_redraw_job)
                self._graph_redraw_job = None
            self._graph_redraw_tick()
            return
        if self._graph_redraw_job is None:
            delay_ms = max(1, int(self._graph_min_redraw_interval_ms - (now_ms - self._graph_last_draw_ms)))
            self._graph_redraw_job = self.root.after(delay_ms, self._graph_redraw_tick)

    def _graph_redraw_tick(self) -> None:
        self._graph_redraw_job = None
        if not self._graph_dirty:
            return
        self._graph_dirty = False
        self._redraw_graph()
        self._graph_last_draw_ms = time.perf_counter() * 1000.0

    def _redraw_graph(self) -> None:
        if self.graph_canvas is None:
            return
        c = self.graph_canvas
        w = max(40, c.winfo_width())
        h = max(40, c.winfo_height())
        c.delete("all")

        left, right, top, bottom = 42, 8, 10, 24
        pw = max(10, w - left - right)
        ph = max(10, h - top - bottom)
        c.create_rectangle(left, top, left + pw, top + ph, outline="#3a4652")

        for frac, label in ((0.0, "2047"), (0.25, "1535"), (0.5, "1023"), (0.75, "511"), (1.0, "0")):
            y = top + (frac * ph)
            c.create_line(left, y, left + pw, y, fill="#25303a")
            c.create_text(left - 6, y, text=label, fill="#94a0ab", anchor="e", font=("TkDefaultFont", 8))

        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = left + (frac * pw)
            c.create_line(x, top, x, top + ph, fill="#1d2730")
        c.create_text(left, h - 10, text=f"-{int(self.graph_window_s)}s", fill="#94a0ab", anchor="w", font=("TkDefaultFont", 8))
        c.create_text(left + pw, h - 10, text="now", fill="#94a0ab", anchor="e", font=("TkDefaultFont", 8))

        if not self.graph_samples:
            c.create_text(left + (pw / 2), top + (ph / 2), text="No throttle data yet", fill="#70808f")
            return

        t_end = time.time()
        t_start = t_end - self.graph_window_s
        active_motors = min(self.motor_count, 4)
        for motor in range(active_motors):
            points: list[float] = []
            for ts, values in self.graph_samples:
                if ts < t_start:
                    continue
                x = left + ((ts - t_start) / self.graph_window_s) * pw
                y = top + (1.0 - (values[motor] / 2047.0)) * ph
                points.extend((x, y))
            if len(points) >= 4:
                c.create_line(*points, fill=self.graph_colors[motor], width=2)

        for motor in range(active_motors):
            txt = f"M{motor + 1}: {self.graph_latest[motor]}"
            y = top + 12 + (motor * 14)
            c.create_text(left + pw - 6, y, text=txt, fill=self.graph_colors[motor], anchor="e", font=("TkDefaultFont", 8))

    def _pt_enter(self) -> None:
        self._send_cmd(f"pt_enter {int(self.pt_motor_var.get())}")

    def _pt_read(self) -> None:
        self._send_cmd(f"pt_read {int(self.pt_addr_var.get())} {int(self.pt_len_var.get())}")

    def _pt_write(self) -> None:
        addr = int(self.pt_addr_var.get())
        data = self.pt_write_var.get().strip()
        if not data:
            self._log("ERR write bytes are empty")
            return
        self._send_cmd(f"pt_write {addr} {data}")

    def _pt_erase(self) -> None:
        self._send_cmd(f"pt_erase {int(self.pt_addr_var.get())}")

    def _send_custom(self) -> None:
        cmd = self.custom_cmd_var.get().strip()
        if cmd:
            self._send_cmd(cmd)

    def _on_close(self) -> None:
        if self.status_poll_job is not None:
            self.root.after_cancel(self.status_poll_job)
            self.status_poll_job = None
        if self.keepalive_job is not None:
            self.root.after_cancel(self.keepalive_job)
            self.keepalive_job = None
        if self.link_job is not None:
            self.root.after_cancel(self.link_job)
            self.link_job = None
        if self._graph_redraw_job is not None:
            self.root.after_cancel(self._graph_redraw_job)
            self._graph_redraw_job = None
        self._stop_ramp_test(force=True)
        self._disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    EscSerialUi(root)
    root.mainloop()


if __name__ == "__main__":
    main()
