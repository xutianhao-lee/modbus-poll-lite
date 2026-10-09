# -*- coding: utf-8 -*-
"""
Modbus Poll Lite —— Modbus 主站调试工具（Modbus Poll 风格，零依赖）

照着 Modbus Poll 的操作逻辑做的练习版，术语/快捷键/流程一致：
    F3  连接设置      Connection -> Connect...
    F8  读写定义      Setup -> Read/Write Definition...
    双击单元格       写入数值
    显示 -> 报文      查看十六进制收发

只依赖 Python 标准库：tkinter + socket + struct + threading

运行：PYTHONIOENCODING=utf-8 python src/modbus_poll_lite.py
"""

import json
import queue
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

APP = "Modbus Poll Lite"

# ----------------------------------------------------------------- 协议常量

FUNCTIONS = [
    (1, "01 读线圈 (Read Coils)"),
    (2, "02 读离散输入 (Read Discrete Inputs)"),
    (3, "03 读保持寄存器 (Read Holding Registers)"),
    (4, "04 读输入寄存器 (Read Input Registers)"),
    (5, "05 写单线圈 (Write Single Coil)"),
    (6, "06 写单寄存器 (Write Single Register)"),
    (15, "15 写多线圈 (Write Multiple Coils)"),
    (16, "16 写多寄存器 (Write Multiple Registers)"),
]
FC_LABEL = {code: text for code, text in FUNCTIONS}
BIT_FUNCS = (1, 2)
WRITE_FUNCS = (5, 6, 15, 16)      # 写功能码：窗口进入写入模式，不轮询
# 读功能码 -> 双击时用的写功能码
WRITE_FOR_READ = {1: 5, 3: 6, 15: 15, 16: 16}

EXC_TEXT = {
    1: "非法功能码 Illegal Function",
    2: "非法数据地址 Illegal Data Address",
    3: "非法数据值 Illegal Data Value",
    4: "从站设备故障 Slave Device Failure",
    5: "确认 Acknowledge",
    6: "从站忙 Slave Device Busy",
    8: "存储奇偶校验错 Memory Parity Error",
    10: "网关路径不可用 Gateway Path Unavailable",
    11: "网关目标设备无响应 Gateway Target Failed",
}

# 机器人控制器 Modbus 从站地址表（见《工业机器人通信手册》2.4.1）
# (短名, 地址范围, 功能码, 起始地址)
ROBOT_RANGES = [
    ("离散量输入", "0~4095", 2, 0),
    ("线圈", "4096~8191", 1, 4096),
    ("输入寄存器", "0~32767", 4, 0),
    ("保持寄存器", "32768~65535", 3, 32768),
]

FORMATS = ["Signed", "Unsigned", "Hex", "Binary", "Float", "Long"]
WORD_ORDERS = ["高位在前 (Big)", "低位在前 (Little)"]


def exc_text(code):
    return EXC_TEXT.get(code, f"未知异常码 {code}")


# ----------------------------------------------------------------- 主站实现

class ModbusError(Exception):
    def __init__(self, code):
        super().__init__(exc_text(code))
        self.code = code


class ModbusMaster:
    """极简 Modbus TCP 主站"""

    def __init__(self):
        self.sock = None
        self.lock = threading.Lock()
        self.tid = 0
        self.unit = 1
        self.timeout = 1.0
        self.tx = 0
        self.err = 0
        self.connected = False
        self.peer = ""
        self.last_error = ""
        self.traffic = []          # [(方向, 十六进制串, 说明)]
        self.on_traffic = None

    # -- 连接管理 ----------------------------------------------------------
    def connect(self, ip, port, unit, timeout):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        self.unit = unit
        self.timeout = timeout
        self.connected = True
        self.peer = f"{ip}:{port}"
        self.tx = self.err = 0
        self.last_error = ""

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected = False

    # -- 收发 --------------------------------------------------------------
    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("连接被从站关闭")
            buf += chunk
        return buf

    def _log(self, direction, data, note=""):
        line = (direction, data.hex(" ").upper(), note)
        self.traffic.append(line)
        if len(self.traffic) > 2000:
            del self.traffic[:500]
        if self.on_traffic:
            try:
                self.on_traffic(line)
            except Exception:
                pass

    def request(self, pdu):
        """发一帧，返回响应 PDU。异常响应抛 ModbusError，通讯故障抛其它异常。"""
        with self.lock:
            if not self.sock:
                raise ConnectionError("未连接")
            if not 0 <= self.unit <= 255:
                raise ValueError(f"从站号 {self.unit} 超出范围（0 ~ 255）")
            self.tid = (self.tid + 1) & 0xFFFF
            frame = struct.pack(">HHHB", self.tid, 0, len(pdu) + 1, self.unit) + pdu
            self.tx += 1
            self._log("Tx", frame)
            self.sock.sendall(frame)
            if self.unit == 0:
                # 广播：Modbus 规范里从站不应答，发完即算成功
                self.last_error = ""
                return b""
            hdr = self._recv_exact(7)
            tid, _pid, length, _uid = struct.unpack(">HHHB", hdr)
            body = self._recv_exact(length - 1) if length > 1 else b""
            self._log("Rx", hdr + body)
            if body and (body[0] & 0x80):
                code = body[1] if len(body) > 1 else 0
                self.err += 1
                self.last_error = exc_text(code)
                raise ModbusError(code)
            self.last_error = ""
            return body

    # -- 功能码 ------------------------------------------------------------
    def read(self, fc, addr, qty):
        pdu = struct.pack(">BHH", fc, addr, qty)
        resp = self.request(pdu)
        if fc in BIT_FUNCS:
            nbytes = resp[1]
            raw = resp[2:2 + nbytes]
            return [bool(raw[i // 8] >> (i % 8) & 1) for i in range(qty)]
        nbytes = resp[1]
        return list(struct.unpack(">" + "H" * (nbytes // 2), resp[2:2 + nbytes]))

    def write_single_register(self, addr, value):
        self.request(struct.pack(">BHH", 6, addr, value & 0xFFFF))

    def write_single_coil(self, addr, on):
        self.request(struct.pack(">BHH", 5, addr, 0xFF00 if on else 0x0000))

    def write_multiple_registers(self, addr, values):
        body = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
        self.request(struct.pack(">BHHB", 16, addr, len(values), len(body)) + body)

    def write_multiple_coils(self, addr, values):
        nbytes = (len(values) + 7) // 8
        buf = bytearray(nbytes)
        for i, v in enumerate(values):
            if v:
                buf[i // 8] |= 1 << (i % 8)
        self.request(struct.pack(">BHHB", 15, addr, len(values), nbytes) + bytes(buf))


# ----------------------------------------------------------------- 界面

class PollLite(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(f"{APP}")
        self.geometry("980x620")
        self.minsize(820, 480)

        # --- 主站 ---
        self.mb = ModbusMaster()
        self.mb.on_traffic = self._traffic_cb
        self.poll_thread = None
        self.stop_flag = threading.Event()
        self.data_lock = threading.Lock()

        # --- 当前读写定义（默认跟 Modbus Poll 出厂一致）---
        self.slave_id = 1
        self.fc = 3
        self.addr = 0
        self.qty = 10
        self.scan_rate = 1000
        self.enabled = False
        self.fmt = "Signed"
        self.word_order = WORD_ORDERS[0]
        self.base1 = False
        self.max_rows = 10

        # 缩放：工程量 = 原始值 × scale + offset
        self.scale = 1.0
        self.offset = 0.0
        self.unit = ""
        # 条件着色阈值（None = 不启用）
        self.alarm_low = None
        self.alarm_high = None
        # 数据记录
        self.log_fh = None
        self.log_lock = threading.Lock()
        self.log_path = ""
        # 上次连接参数（保存配置用）
        self.last_ip = "127.0.0.1"
        self.last_port = 502
        self.last_timeout = 1000

        self.values = []
        self.err_state = None
        self.alias = {}
        self.traffic_win = None
        self.traffic_queue = queue.Queue()

        self._build_menu()
        self._build_toolbar()
        self._build_status()
        self._build_grid()
        self._bind_keys()
        self._tick()

    # ---------------------------------------------------------------- 菜单
    def _build_menu(self):
        m = tk.Menu(self)

        f = tk.Menu(m, tearoff=0)
        f.add_command(label="新建", command=self._new)
        f.add_separator()
        f.add_command(label="打开配置…", command=self._load_config)
        f.add_command(label="保存配置…", command=self._save_config)
        f.add_separator()
        f.add_command(label="退出", command=self.destroy)
        m.add_cascade(label="文件", menu=f)

        c = tk.Menu(m, tearoff=0)
        c.add_command(label="连接…", accelerator="F3", command=self.dlg_connect)
        c.add_command(label="断开", command=self._disconnect)
        m.add_cascade(label="连接", menu=c)

        s = tk.Menu(m, tearoff=0)
        s.add_command(label="读写定义…", accelerator="F8", command=self.dlg_definition)
        m.add_cascade(label="设置", menu=s)

        fu = tk.Menu(m, tearoff=0)
        fu.add_command(label="写单线圈 (05)…", command=lambda: self._write_dialog(5))
        fu.add_command(label="写单寄存器 (06)…", command=lambda: self._write_dialog(6))
        fu.add_separator()
        fu.add_command(label="写多线圈 (15)…", command=lambda: self._write_dialog(15))
        fu.add_command(label="写多寄存器 (16)…", command=lambda: self._write_dialog(16))
        m.add_cascade(label="功能", menu=fu)

        d = tk.Menu(m, tearoff=0)
        self.var_base = tk.IntVar(value=0)
        d.add_radiobutton(label="地址基准 Base 0（协议地址，从 0 起）",
                          variable=self.var_base, value=0, command=self._refresh_grid)
        d.add_radiobutton(label="地址基准 Base 1（PLC 地址，40001 风格）",
                          variable=self.var_base, value=1, command=self._refresh_grid)
        d.add_separator()
        d.add_command(label="显示格式…", command=self._cycle_format)
        d.add_command(label="报文 (Communication)", command=self.dlg_traffic)
        d.add_separator()
        d.add_command(label="开始记录到 CSV…", command=lambda: self._toggle_log(True))
        d.add_command(label="停止记录 CSV", command=lambda: self._toggle_log(False))
        m.add_cascade(label="显示", menu=d)

        v = tk.Menu(m, tearoff=0)
        v.add_command(label="机器人地址表", command=self.dlg_robot_ranges)
        v.add_command(label="地址扫描…", command=self.dlg_scan)
        m.add_cascade(label="视图", menu=v)

        h = tk.Menu(m, tearoff=0)
        h.add_command(label="使用说明", command=self.dlg_help)
        m.add_cascade(label="帮助", menu=h)

        self.config(menu=m)

    # --------------------------------------------------------------- 工具栏
    def _build_toolbar(self):
        bar = ttk.Frame(self, padding=(6, 4))
        bar.pack(fill="x")

        ttk.Button(bar, text="连接 (F3)", command=self.dlg_connect).pack(side="left")
        ttk.Button(bar, text="断开", command=self._disconnect).pack(side="left", padx=(4, 10))
        ttk.Button(bar, text="读写定义 (F8)", command=self.dlg_definition).pack(side="left", padx=(0, 10))

        ttk.Label(bar, text="快捷功能码:").pack(side="left")
        for code in (1, 2, 3, 4, 5, 6, 15, 16):
            ttk.Button(bar, text=f"{code:02d}", width=3,
                       command=lambda c=code: self._quick_fc(c)).pack(side="left", padx=1)

        ttk.Button(bar, text="报文", command=self.dlg_traffic).pack(side="left", padx=(10, 0))

        # 第二行：当前定义摘要 + 机器人地址表快捷
        bar2 = ttk.Frame(self, padding=(6, 0))
        bar2.pack(fill="x")
        ttk.Label(bar2, text="机器人地址表:").pack(side="left")
        for name, rng, fc, addr in ROBOT_RANGES:
            ttk.Button(bar2, text=f"{name} {rng}", width=17,
                       command=lambda f=fc, a=addr: self._preset(f, a)).pack(side="left", padx=2)
        ttk.Label(bar2, text="  ← 一键填入功能码和起始地址",
                  foreground="#666").pack(side="left")

    def _build_status(self):
        f = ttk.Frame(self, padding=(6, 4))
        f.pack(fill="x")
        self.lbl_def = ttk.Label(f, text="", font=("Consolas", 10))
        self.lbl_def.pack(side="left")
        self.lbl_conn = ttk.Label(f, text="  No connection", font=("Consolas", 10, "bold"),
                                  foreground="#c00")
        self.lbl_conn.pack(side="left", padx=(12, 0))

    def _build_grid(self):
        f = ttk.Frame(self, padding=(6, 2))
        f.pack(fill="both", expand=True)

        cols = ("addr", "alias", "value")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=20)
        self.tree.heading("addr", text="地址")
        self.tree.heading("alias", text="别名")
        self.tree.heading("value", text="数值")
        self.tree.column("addr", width=130, anchor="w", stretch=False)
        self.tree.column("alias", width=180, anchor="w", stretch=False)
        self.tree.column("value", width=220, anchor="w")
        self.tree.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(f, orient="vertical", command=self.tree.yview)
        sb.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=sb.set)

        self.tree.tag_configure("err", foreground="#c00")
        self.tree.tag_configure("ok", foreground="#000")
        self.tree.tag_configure("alarm", foreground="#900", background="#ffd9d9")
        self.tree.tag_configure("warn", foreground="#960", background="#fff2cc")
        self.tree.bind("<Double-1>", self._on_double_click)
        self.tree.bind("<Button-3>", self._on_right_click)

        self.lbl_hint = ttk.Label(self, text="", foreground="#666", padding=(8, 2))
        self.lbl_hint.pack(fill="x")

    def _bind_keys(self):
        self.bind("<F3>", lambda e: self.dlg_connect())
        self.bind("<F8>", lambda e: self.dlg_definition())
        self.bind("<F1>", lambda e: self.dlg_help())

    # ------------------------------------------------------------ 连接对话框
    def dlg_connect(self):
        top = tk.Toplevel(self)
        top.title("Connection Setup")
        top.transient(self)
        top.grab_set()
        top.resizable(False, False)

        mode = tk.StringVar(value="tcp")
        frm = ttk.LabelFrame(top, text="Connection", padding=10)
        frm.pack(fill="x", padx=12, pady=(12, 6))
        ttk.Radiobutton(frm, text="Serial Port（串口，本工具暂不支持）",
                        variable=mode, value="serial").pack(anchor="w")
        ttk.Radiobutton(frm, text="TCP/IP（Modbus TCP）",
                        variable=mode, value="tcp").pack(anchor="w")

        rf = ttk.LabelFrame(top, text="Remote Server", padding=10)
        rf.pack(fill="x", padx=12, pady=6)

        v_ip = tk.StringVar(value=self.last_ip)
        v_port = tk.StringVar(value=str(self.last_port))
        v_unit = tk.StringVar(value=str(self.slave_id))
        v_to = tk.StringVar(value=str(self.last_timeout))

        for row, (lab, var, width) in enumerate([
            ("IP 地址", v_ip, 18), ("端口 Port", v_port, 8),
            ("从站号 Unit ID", v_unit, 8), ("响应超时 (ms)", v_to, 8)]):
            ttk.Label(rf, text=lab).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(rf, textvariable=var, width=width).grid(row=row, column=1, sticky="w", padx=8)

        ttk.Label(top, text="提示：机器人控制器网口 2 固定为 192.168.23.25，端口 502",
                  foreground="#666").pack(anchor="w", padx=14)

        def do_connect():
            if mode.get() != "tcp":
                messagebox.showinfo("提示", "本练习版只实现了 Modbus TCP。\n认证用的 Modbus Poll 支持串口。", parent=top)
                return
            try:
                unit = int(v_unit.get(), 0)
                if not 0 <= unit <= 255:
                    raise ValueError(f"从站号 Unit ID 超出范围：0 ~ 255，你填的是 {unit}")
                self.last_ip = v_ip.get().strip()
                self.last_port = int(v_port.get())
                self.last_timeout = int(v_to.get())
                self.mb.connect(self.last_ip, self.last_port,
                                unit, self.last_timeout / 1000.0)
            except Exception as e:
                messagebox.showerror("连接失败", f"{type(e).__name__}: {e}", parent=top)
                self._update_status()
                return
            top.destroy()
            self._update_status()
            self._start_poll()

        ttk.Button(rf, text="OK", command=do_connect).grid(row=4, column=1, sticky="e", pady=(8, 0))
        ttk.Button(rf, text="Cancel", command=top.destroy).grid(row=4, column=0, sticky="w", pady=(8, 0))

    # -------------------------------------------------------- 读写定义对话框
    def dlg_definition(self):
        top = tk.Toplevel(self)
        top.title("Read/Write Definition")
        top.transient(self)
        top.grab_set()
        top.resizable(False, False)

        v_slave = tk.StringVar(value=str(self.slave_id))
        v_func = tk.StringVar(value=FC_LABEL[self.fc])
        v_addr = tk.StringVar(value=str(self.addr))
        v_qty = tk.StringVar(value=str(self.qty))
        v_rate = tk.StringVar(value=str(self.scan_rate))
        v_on = tk.BooleanVar(value=self.enabled)
        v_rows = tk.IntVar(value=self.max_rows)
        v_fmt = tk.StringVar(value=self.fmt)
        v_ord = tk.StringVar(value=self.word_order)
        v_scale = tk.StringVar(value=("%g" % self.scale))
        v_offset = tk.StringVar(value=("%g" % self.offset))
        v_unit = tk.StringVar(value=self.unit)
        v_low = tk.StringVar(value="" if self.alarm_low is None else "%g" % self.alarm_low)
        v_high = tk.StringVar(value="" if self.alarm_high is None else "%g" % self.alarm_high)

        f = ttk.Frame(top, padding=12)
        f.pack(fill="both", expand=True)

        def row(r, label, widget):
            ttk.Label(f, text=label).grid(row=r, column=0, sticky="w", pady=4)
            widget.grid(row=r, column=1, sticky="w", padx=10)

        row(0, "Slave ID（从站号）", ttk.Entry(f, textvariable=v_slave, width=12))
        cb = ttk.Combobox(f, textvariable=v_func, values=[t for _, t in FUNCTIONS],
                          state="readonly", width=38)
        row(1, "Function（功能码）", cb)
        row(2, "Address（起始地址）", ttk.Entry(f, textvariable=v_addr, width=12))
        row(3, "Quantity（数量）", ttk.Entry(f, textvariable=v_qty, width=12))
        row(4, "Scan Rate（扫描周期 ms）", ttk.Entry(f, textvariable=v_rate, width=12))
        row(5, "显示格式", ttk.Combobox(f, textvariable=v_fmt, values=FORMATS,
                                    state="readonly", width=14))
        row(6, "32 位字序", ttk.Combobox(f, textvariable=v_ord, values=WORD_ORDERS,
                                    state="readonly", width=18))

        ttk.Label(f, text="缩放").grid(row=7, column=0, sticky="w", pady=4)
        sc = ttk.Frame(f)
        sc.grid(row=7, column=1, sticky="w", padx=10)
        ttk.Label(sc, text="系数").pack(side="left")
        ttk.Entry(sc, textvariable=v_scale, width=8).pack(side="left", padx=(2, 10))
        ttk.Label(sc, text="偏移").pack(side="left")
        ttk.Entry(sc, textvariable=v_offset, width=8).pack(side="left", padx=(2, 10))
        ttk.Label(sc, text="单位").pack(side="left")
        ttk.Entry(sc, textvariable=v_unit, width=7).pack(side="left", padx=2)

        ttk.Label(f, text="条件着色").grid(row=8, column=0, sticky="w", pady=4)
        al = ttk.Frame(f)
        al.grid(row=8, column=1, sticky="w", padx=10)
        ttk.Label(al, text="下限").pack(side="left")
        ttk.Entry(al, textvariable=v_low, width=8).pack(side="left", padx=(2, 10))
        ttk.Label(al, text="上限").pack(side="left")
        ttk.Entry(al, textvariable=v_high, width=8).pack(side="left", padx=(2, 10))
        ttk.Label(al, text="越界标红，留空即不启用", foreground="#666").pack(side="left")

        ttk.Checkbutton(f, text="Read/Write Enabled（启用轮询）", variable=v_on).grid(
            row=9, column=1, sticky="w", pady=6)
        rr = ttk.Frame(f)
        rr.grid(row=10, column=1, sticky="w", pady=2)
        ttk.Label(rr, text="显示行数:").pack(side="left")
        for n in (10, 20, 50):
            ttk.Radiobutton(rr, text=str(n), variable=v_rows, value=n).pack(side="left", padx=6)

        ttk.Label(f, text="※ 地址一律填协议地址（从 0 起）。手册里的 40001 在这里就是 0。\n"
                          "※ 03 保持寄存器上限 125 个，01/02 线圈上限 2000 个。\n"
                          "※ 缩放与条件着色只对 Signed / Unsigned / Float 生效，Hex 和 Binary 显示原始值。",
                  foreground="#666", justify="left").grid(row=11, column=0, columnspan=2,
                                                          sticky="w", pady=(10, 0))

        def apply():
            try:
                slave = int(v_slave.get(), 0)
                if not 0 <= slave <= 255:
                    raise ValueError(
                        f"Slave ID（从站号）超出范围：0 ~ 255，你填的是 {slave}。\n\n"
                        "从站号最大只有 255。\n"
                        "如果你要填的是寄存器地址，请填到下面的 Address 栏。")
                fc = next(c for c, t in FUNCTIONS if t == v_func.get())
                addr = int(v_addr.get(), 0)
                if not 0 <= addr <= 65535:
                    raise ValueError(f"Address（起始地址）超出范围：0 ~ 65535，你填的是 {addr}。")
                qty = int(v_qty.get(), 0)
                limit = 2000 if fc in BIT_FUNCS else 125
                if not 1 <= qty <= limit:
                    raise ValueError(
                        f"Quantity（数量）超出范围：功能码 {fc:02d} 最多 {limit} 个，你填的是 {qty}。")
                rate = int(v_rate.get(), 0)
                if rate < 50:
                    raise ValueError("Scan Rate（扫描周期）不能小于 50 ms。")
                scale = float(v_scale.get() or 1)
                offs = float(v_offset.get() or 0)
                low_s, high_s = v_low.get().strip(), v_high.get().strip()
                low = float(low_s) if low_s else None
                high = float(high_s) if high_s else None
                if low is not None and high is not None and low > high:
                    raise ValueError("条件着色：下限不能大于上限。")
            except (ValueError, StopIteration) as e:
                messagebox.showerror("参数错误", str(e) or "功能码选择无效", parent=top)
                return

            self.scale = scale
            self.offset = offs
            self.unit = v_unit.get().strip()
            self.alarm_low = low
            self.alarm_high = high
            self.slave_id = slave
            self.fc = fc
            self.addr = addr
            self.qty = qty
            self.scan_rate = rate
            self.enabled = v_on.get()
            self.max_rows = v_rows.get()
            self.fmt = v_fmt.get()
            self.word_order = v_ord.get()
            self.mb.unit = self.slave_id
            top.destroy()
            self._update_status()
            self._refresh_grid()
            if self.enabled and not self.mb.connected:
                if messagebox.askyesno(
                        "还没连接",
                        "读写定义已保存，但当前没有连接到从站。\n\n现在打开连接设置吗？"):
                    self.dlg_connect()

        bf = ttk.Frame(f)
        bf.grid(row=12, column=0, columnspan=2, sticky="e", pady=(12, 0))
        ttk.Button(bf, text="OK", command=apply).pack(side="left", padx=4)
        ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")

    # ------------------------------------------------------------ 写入对话框
    def _row_step(self):
        """Float/Long 格式下，一行吃两个寄存器"""
        return 2 if (self.fc not in BIT_FUNCS and self.fmt in ("Float", "Long")) else 1

    def _write_dialog(self, fc):
        """fc ∈ {5,6} 写单个，{15,16} 写多个；起始地址取当前选中行"""
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("提示", "先在表格里选中一行")
            return
        addr = self.addr + self.tree.index(sel[0]) * self._row_step()

        try:
            if fc == 5:
                v = simpledialog_onoff(self, f"写单线圈 (05)   地址 {addr}")
                if v is None:
                    return
                self.mb.write_single_coil(addr, v)
            elif fc == 6:
                v = simpledialog_int(self, f"写单寄存器 (06)   地址 {addr}", "数值 (0~65535):")
                if v is None:
                    return
                self.mb.write_single_register(addr, v)
            elif fc == 15:
                vals = simpledialog_list(self, f"写多线圈 (15)   起始地址 {addr}",
                                         "每行或逗号分隔，0 / 1：", self.qty)
                if vals is None:
                    return
                self.mb.write_multiple_coils(addr, [int(x) != 0 for x in vals])
            elif fc == 16:
                vals = simpledialog_list(self, f"写多寄存器 (16)   起始地址 {addr}",
                                         "每行或逗号分隔，0 ~ 65535：", self.qty)
                if vals is None:
                    return
                self.mb.write_multiple_registers(addr, vals)
            else:
                return
        except ModbusError as e:
            messagebox.showerror("从站返回异常", exc_text(e.code))
            return
        except Exception as e:
            messagebox.showerror("写入失败", str(e))
            return

        self._read_once()          # 写完立刻刷新，不等下一个轮询周期

    def _on_double_click(self, event):
        row = self.tree.identify_row(event.y)
        if not row:
            return
        self.tree.selection_set(row)
        # 双击「别名」列 → 编辑别名
        if self.tree.identify_column(event.x) == "#2":
            self._edit_alias(self.tree.index(row))
            return
        wf = WRITE_FOR_READ.get(self.fc)
        if wf is None:
            messagebox.showinfo("提示",
                                f"功能码 {self.fc:02d} 是只读的。\n"
                                "只能写线圈(01/15) 或 保持寄存器(03/16)。")
            return
        self._write_dialog(wf)

    def _edit_alias(self, row):
        addr = self.addr + row * self._row_step()
        new = simpledialog_text(self, f"设置别名   地址 {addr}",
                                "别名（留空即清除）：", self.alias.get(addr, ""))
        if new is None:
            return
        if new.strip():
            self.alias[addr] = new.strip()
        else:
            self.alias.pop(addr, None)
        self._refresh_grid()

    def _on_right_click(self, event):
        row = self.tree.identify_row(event.y)
        if row:
            self.tree.selection_set(row)
        menu = tk.Menu(self, tearoff=0)
        sel = self.tree.selection()
        if sel:
            idx = self.tree.index(sel[0])
            menu.add_command(label="设置别名…", command=lambda: self._edit_alias(idx))
            wf = WRITE_FOR_READ.get(self.fc)
            if wf:
                menu.add_command(label=f"写入 (FC{wf:02d})…",
                                 command=lambda: self._write_dialog(wf))
            menu.add_separator()
            menu.add_command(label="复制本行", command=lambda: self._copy_row(idx))
        menu.tk_popup(event.x_root, event.y_root)

    def _copy_row(self, row):
        rows = self.tree.get_children()
        if row < len(rows):
            vals = self.tree.item(rows[row], "values")
            self.clipboard_clear()
            self.clipboard_append("\t".join(str(v) for v in vals))

    # ---------------------------------------------------------------- 轮询
    def _start_poll(self):
        self._stop_poll()
        self.stop_flag.clear()
        self.poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self.poll_thread.start()

    def _stop_poll(self):
        self.stop_flag.set()
        if self.poll_thread and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=2)
        self.poll_thread = None

    def _poll_loop(self):
        while not self.stop_flag.is_set():
            if self.enabled and self.mb.connected and self.fc not in WRITE_FUNCS:
                try:
                    vals = self.mb.read(self.fc, self.addr, self.qty)
                    with self.data_lock:
                        self.values = vals
                        self.err_state = None
                    self._log_values(vals)
                except ModbusError as e:
                    with self.data_lock:
                        self.values = []
                        self.err_state = exc_text(e.code)
                except Exception as e:
                    with self.data_lock:
                        self.values = []
                        self.err_state = f"{type(e).__name__}: {e}"
                    self.stop_flag.wait(1.0)
                    continue
            self.stop_flag.wait(max(0.05, self.scan_rate / 1000.0))

    def _read_once(self):
        """立即读写一次并刷新界面（写值后用，避免干等一个轮询周期）"""
        if not self.mb.connected or self.fc in WRITE_FUNCS:
            return
        try:
            vals = self.mb.read(self.fc, self.addr, self.qty)
            with self.data_lock:
                self.values = vals
                self.err_state = None
            self._log_values(vals)
        except ModbusError as e:
            with self.data_lock:
                self.values = []
                self.err_state = exc_text(e.code)
        except Exception as e:
            with self.data_lock:
                self.values = []
                self.err_state = f"{type(e).__name__}: {e}"
        self._refresh_grid()
        self._update_status()

    def _tick(self):
        while True:
            try:
                line = self.traffic_queue.get_nowait()
            except queue.Empty:
                break
            self._append_traffic(line)
        self._refresh_grid()
        self._update_status()
        self.after(200, self._tick)

    # ---------------------------------------------------------------- 刷新
    def _disp_addr(self, addr):
        if not self.var_base.get():
            return str(addr)
        if self.fc == 1:
            return f"{addr + 1:05d}"
        if self.fc == 2:
            return f"{addr + 10001:05d}"
        if self.fc == 4:
            return f"{addr + 30001:05d}"
        return f"{addr + 40001:05d}"

    def _raw_number(self, vals, i):
        """取第 i 行对应的原始数值（未缩放）；非数值行返回 None"""
        if self.fc in BIT_FUNCS:
            return 1.0 if vals[i] else 0.0
        if self.fmt in ("Float", "Long"):
            if i + 1 >= len(vals):
                return None
            hi, lo = vals[i], vals[i + 1]
            if self.word_order.startswith("低位"):
                hi, lo = lo, hi
            raw = struct.pack(">HH", hi, lo)
            if self.fmt == "Float":
                return float(struct.unpack(">f", raw)[0])
            return float(struct.unpack(">i", raw)[0])
        v = vals[i]
        if self.fmt == "Signed":
            return float(v - 65536 if v >= 32768 else v)
        return float(v)

    def _scaled(self, n):
        """工程量 = 原始值 × 系数 + 偏移"""
        return n * self.scale + self.offset

    def _format_value(self, vals, i):
        """返回第 i 行显示的字符串；Float/Long 一次吃 2 个寄存器"""
        if self.fc in BIT_FUNCS:
            return "1  ON" if vals[i] else "0  OFF"

        if self.fmt in ("Hex", "Binary"):
            v = vals[i]
            return f"0x{v:04X}" if self.fmt == "Hex" else f"{v:016b}"

        n = self._raw_number(vals, i)
        if n is None:
            return "-"
        if self.fmt == "Long":
            return str(int(n))                       # 计数器类不做缩放，避免精度损失
        if self.fmt == "Float":
            out = f"{self._scaled(n):.4f}"
        else:                                        # Signed / Unsigned
            out = "%g" % self._scaled(n)
        return out + (" " + self.unit if self.unit else "")

    def _alarm_state(self, vals, i):
        """按阈值判断是否越界，返回标签名或 None"""
        if self.alarm_low is None and self.alarm_high is None:
            return None
        n = self._raw_number(vals, i)
        if n is None:
            return None
        val = n if self.fmt in ("Hex", "Binary", "Long") else self._scaled(n)
        if ((self.alarm_low is not None and val < self.alarm_low) or
                (self.alarm_high is not None and val > self.alarm_high)):
            return "alarm"
        return None

    def _refresh_grid(self):
        with self.data_lock:
            vals = list(self.values)
            err = getattr(self, "err_state", None)

        step = 2 if (self.fc not in BIT_FUNCS and self.fmt in ("Float", "Long")) else 1
        n_show = min(self.max_rows, max(1, self.qty // step)) if vals else self.max_rows

        if self.tree.get_children() and len(self.tree.get_children()) == n_show:
            pass
        else:
            self.tree.delete(*self.tree.get_children())
            for i in range(n_show):
                tag = "ok"
                self.tree.insert("", "end", iid=f"r{i}",
                                 values=(self._disp_addr(self.addr + i * step),
                                         self.alias.get(self.addr + i * step, ""), "0"),
                                 tags=(tag,))

        for i in range(n_show):
            iid = f"r{i}"
            if not self.tree.exists(iid):
                continue
            a = self.addr + i * step
            if err:
                txt, tag = "—", "err"
            elif vals and i < len(vals):
                txt = self._format_value(vals, i * step)
                tag = "ok"
                if self.fc not in BIT_FUNCS:
                    tag = self._alarm_state(vals, i * step) or "ok"
            else:
                txt, tag = "—", "ok"
            self.tree.item(iid, values=(self._disp_addr(a), self.alias.get(a, ""), txt), tags=(tag,))

        hint = f"{FC_LABEL[self.fc]}   起始地址 {self.addr}   数量 {self.qty}   " \
               f"格式 {self.fmt}   周期 {self.scan_rate}ms   行数 {n_show}"
        if self.fc in WRITE_FUNCS:
            self.lbl_hint.configure(
                foreground="#06c",
                text=f"✎ 写模式（{FC_LABEL[self.fc]}）—— 本窗口不轮询，"
                     f"用「功能」菜单或双击行发起写入。      " + hint)
        elif err:
            hint += f"    ← 从站返回：{err}"
            self.lbl_hint.configure(foreground="#c00", text=hint)
        elif not self.mb.connected:
            self.lbl_hint.configure(
                foreground="#c00",
                text="⚠ 未连接 —— 按 F3 连接从站后才会开始轮询。      " + hint)
        elif not self.enabled:
            self.lbl_hint.configure(
                foreground="#c60",
                text="⚠ 已连接，但未启用轮询 —— 按 F8 勾选「Read/Write Enabled」。      " + hint)
        else:
            self.lbl_hint.configure(foreground="#666", text=hint)

    def _update_status(self):
        self.lbl_def.configure(
            text=f"Tx = {self.mb.tx}  Err = {self.mb.err}  ID = {self.slave_id}  "
                 f"F = {self.fc:02d}: SR = {self.scan_rate}ms"
                 + ("" if self.enabled else "  (DISABLED)"))
        if self.mb.connected:
            self.lbl_conn.configure(text=f"  ● {self.mb.peer}", foreground="#0a0")
        else:
            self.lbl_conn.configure(text="  No connection", foreground="#c00")

    # ---------------------------------------------------------------- 动作
    def _quick_fc(self, code):
        self.fc = code
        self._update_status()
        self._refresh_grid()

    def _preset(self, fc, addr):
        self.fc = fc
        self.addr = addr
        self.qty = 10
        self.enabled = True
        self.fmt = "Signed"
        self._update_status()
        self._refresh_grid()

    def _cycle_format(self):
        self.fmt = FORMATS[(FORMATS.index(self.fmt) + 1) % len(FORMATS)]
        self._refresh_grid()

    # ------------------------------------------------------------ 数据记录
    def _log_values(self, vals):
        if not self.log_fh:
            return
        try:
            cells = [str(int(v)) if isinstance(v, bool) else str(v) for v in vals]
            line = time.strftime("%Y-%m-%d %H:%M:%S") + "," + ",".join(cells) + "\n"
            with self.log_lock:
                self.log_fh.write(line)
                self.log_fh.flush()
        except Exception:
            pass

    def _toggle_log(self, start):
        if not start:
            with self.log_lock:
                if self.log_fh:
                    try:
                        self.log_fh.close()
                    except Exception:
                        pass
                    self.log_fh = None
                    messagebox.showinfo("已停止记录", f"文件：\n{self.log_path}")
            return
        if self.log_fh:
            messagebox.showinfo("提示", f"已经在记录了：\n{self.log_path}")
            return
        path = filedialog.asksaveasfilename(
            title="数据记录保存为", defaultextension=".csv",
            initialfile=time.strftime("modbus_log_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")])
        if not path:
            return
        step = self._row_step()
        header = ["时间"] + [self._disp_addr(self.addr + i * step)
                            for i in range(max(1, self.qty // step))]
        try:
            fh = open(path, "w", encoding="utf-8-sig", newline="")
            fh.write(",".join(str(h) for h in header) + "\n")
            fh.flush()
        except Exception as e:
            messagebox.showerror("打开文件失败", str(e))
            return
        with self.log_lock:
            self.log_fh = fh
            self.log_path = path
        messagebox.showinfo("开始记录",
                            f"写入：\n{path}\n\n停止：显示 → 停止记录 CSV")

    # ------------------------------------------------------------ 配置读写
    def _config_dict(self):
        return {
            "ip": self.last_ip, "port": self.last_port, "timeout": self.last_timeout,
            "unit": self.slave_id, "fc": self.fc, "addr": self.addr, "qty": self.qty,
            "scan_rate": self.scan_rate, "fmt": self.fmt, "word_order": self.word_order,
            "base": self.var_base.get(), "scale": self.scale, "offset": self.offset,
            "unit_text": self.unit, "alarm_low": self.alarm_low, "alarm_high": self.alarm_high,
            "alias": {str(k): v for k, v in self.alias.items()},
        }

    def _save_config(self):
        path = filedialog.asksaveasfilename(
            title="保存配置", defaultextension=".mplite",
            initialfile="modbus_config.mplite",
            filetypes=[("Modbus Poll Lite 配置", "*.mplite"),
                       ("JSON", "*.json"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self._config_dict(), fh, ensure_ascii=False, indent=2)
        except Exception as e:
            messagebox.showerror("保存失败", str(e))
            return
        messagebox.showinfo("已保存", path)

    def _load_config(self):
        path = filedialog.askopenfilename(
            title="打开配置",
            filetypes=[("Modbus Poll Lite 配置", "*.mplite"),
                       ("JSON", "*.json"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                cfg = json.load(fh)
        except Exception as e:
            messagebox.showerror("读取失败", str(e))
            return
        try:
            self._disconnect()
            self.last_ip = cfg.get("ip", "127.0.0.1")
            self.last_port = int(cfg.get("port", 502))
            self.last_timeout = int(cfg.get("timeout", 1000))
            self.slave_id = int(cfg.get("unit", 1))
            self.fc = int(cfg.get("fc", 3))
            self.addr = int(cfg.get("addr", 0))
            self.qty = int(cfg.get("qty", 10))
            self.scan_rate = int(cfg.get("scan_rate", 1000))
            self.fmt = cfg.get("fmt", "Signed")
            self.word_order = cfg.get("word_order", WORD_ORDERS[0])
            self.var_base.set(int(cfg.get("base", 0)))
            self.scale = float(cfg.get("scale", 1.0))
            self.offset = float(cfg.get("offset", 0.0))
            self.unit = cfg.get("unit_text", "")
            self.alarm_low = cfg.get("alarm_low")
            self.alarm_high = cfg.get("alarm_high")
            self.alias = {int(k): v for k, v in (cfg.get("alias") or {}).items()}
        except Exception as e:
            messagebox.showerror("配置内容有误", str(e))
            return
        self.mb.unit = self.slave_id
        self._update_status()
        self._refresh_grid()
        messagebox.showinfo("已载入", f"{path}\n\n按 F3 连接。")

    def _new(self):
        self._disconnect()
        self.addr = 0
        self.qty = 10
        self.fc = 3
        self.fmt = "Signed"
        self.alias.clear()
        with self.data_lock:
            self.values = []
        self._refresh_grid()

    def _disconnect(self):
        self._stop_poll()
        self.mb.close()
        with self.data_lock:
            self.values = []
        self._update_status()
        self._refresh_grid()

    # ---------------------------------------------------------------- 窗口
    def _traffic_cb(self, line):
        # 从轮询线程调用：只入队，由主线程的 _tick 取出显示（避免跨线程操作 tk）
        self.traffic_queue.put(line)

    def _append_traffic(self, line):
        if not (self.traffic_win and self.traffic_win.winfo_exists()):
            return
        txt = self.traffic_win.text
        d, hexs, note = line
        txt.configure(state="normal")
        txt.insert("end", f"{d}  {hexs}\n")
        if note:
            txt.insert("end", f"    {note}\n")
        txt.see("end")
        txt.configure(state="disabled")

    def dlg_traffic(self):
        if self.traffic_win and self.traffic_win.winfo_exists():
            self.traffic_win.lift()
            return
        top = tk.Toplevel(self)
        top.title("Communication —— 报文 (只显示本工具收发的帧)")
        top.geometry("640x420")
        txt = tk.Text(top, font=("Consolas", 9), wrap="none")
        txt.pack(fill="both", expand=True)
        txt.configure(state="disabled")
        top.text = txt
        self.traffic_win = top
        for line in self.mb.traffic:
            self._append_traffic(line)

    def dlg_scan(self):
        """地址扫描：逐地址读一个，列出哪些地址从站响应"""
        top = tk.Toplevel(self)
        top.title("地址扫描 Address Scan")
        top.geometry("600x480")
        top.transient(self)

        f = ttk.Frame(top, padding=10)
        f.pack(fill="x")

        default_fc = self.fc if self.fc in (1, 2, 3, 4) else 3
        v_func = tk.StringVar(value=FC_LABEL[default_fc])
        v_from = tk.StringVar(value=str(self.addr))
        v_to = tk.StringVar(value=str(self.addr + 15))

        ttk.Label(f, text="功能码").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Combobox(f, textvariable=v_func, state="readonly", width=34,
                     values=[t for c, t in FUNCTIONS if c in (1, 2, 3, 4)]).grid(
            row=0, column=1, sticky="w", padx=8)
        ttk.Label(f, text="起始地址").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(f, textvariable=v_from, width=12).grid(row=1, column=1, sticky="w", padx=8)
        ttk.Label(f, text="结束地址").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(f, textvariable=v_to, width=12).grid(row=2, column=1, sticky="w", padx=8)

        res = ttk.Treeview(top, columns=("a", "s"), show="headings", height=14)
        res.heading("a", text="地址")
        res.heading("s", text="结果")
        res.column("a", width=110, anchor="w")
        res.column("s", width=430, anchor="w")
        res.pack(fill="both", expand=True, padx=10)
        res.tag_configure("ok", foreground="#080")
        res.tag_configure("bad", foreground="#c00")

        ctl = ttk.Frame(top, padding=10)
        ctl.pack(fill="x")
        btn = ttk.Button(ctl, text="开始扫描")
        btn.pack(side="left")
        lbl = ttk.Label(ctl, text="选好范围后点开始")
        lbl.pack(side="left", padx=10)

        ttk.Label(top, text="※ 用 03 / 04 扫描可摸清从站寄存器分布；回异常码 02 = 该地址不在从站允许范围内。\n"
                            "※ 扫描期间会占用连接，请先停掉轮询（取消勾选 Read/Write Enabled）。",
                  foreground="#666", justify="left",
                  padding=(10, 0, 10, 8)).pack(anchor="w")

        q = queue.Queue()
        state = {"running": False, "stop": False}

        def drain():
            while True:
                try:
                    kind, payload, *rest = q.get_nowait()
                except queue.Empty:
                    break
                if kind == "row":
                    res.insert("", "end", values=payload, tags=(rest[0],))
                    res.see(res.get_children()[-1])
                else:
                    lbl.configure(text=payload)
                    btn.configure(text="开始扫描")
            if top.winfo_exists():
                top.after(150, drain)

        def worker(fc, a0, a1):
            ok = bad = 0
            for a in range(a0, a1 + 1):
                if state["stop"]:
                    break
                try:
                    self.mb.read(fc, a, 1)
                    q.put(("row", (a, "✔ 正常响应"), "ok"))
                    ok += 1
                except ModbusError as e:
                    q.put(("row", (a, f"异常码 {e.code:02d} — {e}"), "bad"))
                    bad += 1
                except Exception as e:
                    q.put(("row", (a, f"通讯失败 — {type(e).__name__}: {e}"), "bad"))
                    bad += 1
            q.put(("done", f"扫描结束：正常 {ok} 个，异常/失败 {bad} 个", ""))

        def start():
            if state["running"]:
                state["stop"] = True
                lbl.configure(text="正在停止…")
                return
            if not self.mb.connected:
                messagebox.showwarning("未连接", "请先按 F3 连接从站", parent=top)
                return
            try:
                fc = next(c for c, t in FUNCTIONS if t == v_func.get())
                a0, a1 = int(v_from.get(), 0), int(v_to.get(), 0)
                if a1 < a0:
                    a0, a1 = a1, a0
                if not 0 <= a0 <= 65535 or not 0 <= a1 <= 65535:
                    raise ValueError("地址必须在 0 ~ 65535 之间")
                if a1 - a0 > 999 and not messagebox.askyesno(
                        "范围较大", f"要扫 {a1 - a0 + 1} 个地址，可能比较慢。继续吗？", parent=top):
                    return
            except Exception as e:
                messagebox.showerror("参数错误", str(e), parent=top)
                return
            res.delete(*res.get_children())
            state["running"] = True
            state["stop"] = False
            btn.configure(text="停止")
            lbl.configure(text="扫描中…")
            threading.Thread(target=worker, args=(fc, a0, a1), daemon=True).start()

        btn.configure(command=start)
        top.after(150, drain)

    def dlg_robot_ranges(self):
        top = tk.Toplevel(self)
        top.title("机器人 Modbus 从站地址表")
        top.transient(self)
        ttk.Label(top, text="机器人控制器 Modbus 从站地址表",
                  font=("", 11, "bold"), padding=(12, 10, 12, 4)).pack(anchor="w")
        cols = ("obj", "range", "fc", "rw")
        tv = ttk.Treeview(top, columns=cols, show="headings", height=4)
        for c, t, w in (("obj", "对象", 130), ("range", "地址范围", 150),
                        ("fc", "功能码", 110), ("rw", "访问", 80)):
            tv.heading(c, text=t)
            tv.column(c, width=w, anchor="w")
        tv.pack(fill="x", padx=12)
        for obj, rng, fc, rw in [
            ("离散量输入", "0 ~ 4095", "02", "只读"),
            ("线圈", "4096 ~ 8191", "01 / 05 / 15", "读写"),
            ("输入寄存器", "0 ~ 32767", "04", "只读"),
            ("保持寄存器", "32768 ~ 65535", "03 / 06 / 16", "读写"),
        ]:
            tv.insert("", "end", values=(obj, rng, fc, rw))
        ttk.Label(top, text="※ 地址为协议地址（从 0 起）。读错段从站会回异常码 02 —— \n"
                            "   这正是官方课件里 Illegal Data Address 的成因。",
                  foreground="#666", justify="left", padding=(12, 10)).pack(anchor="w")

    def dlg_help(self):
        top = tk.Toplevel(self)
        top.title("使用说明")
        top.geometry("620x480")
        txt = tk.Text(top, wrap="word", font=("", 10), padx=12, pady=10)
        txt.pack(fill="both", expand=True)
        txt.insert("end", HELP_TEXT)
        txt.configure(state="disabled")


# ----------------------------------------------------------------- 小对话框

def simpledialog_int(parent, title, prompt):
    top = tk.Toplevel(parent)
    top.title(title)
    top.transient(parent)
    top.grab_set()
    ttk.Label(top, text=prompt, padding=(12, 12, 12, 4)).pack(anchor="w")
    var = tk.StringVar()
    e = ttk.Entry(top, textvariable=var, width=16)
    e.pack(padx=12)
    e.focus_set()
    result = {"v": None}

    def ok():
        try:
            result["v"] = int(var.get(), 0)
        except ValueError:
            messagebox.showerror("格式错误", "请输入整数（可用 0x 前缀写十六进制）", parent=top)
            return
        top.destroy()

    bf = ttk.Frame(top, padding=12)
    bf.pack()
    ttk.Button(bf, text="OK", command=ok).pack(side="left", padx=4)
    ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")
    e.bind("<Return>", lambda e_: ok())
    parent.wait_window(top)
    return result["v"]


def simpledialog_text(parent, title, prompt, initial=""):
    top = tk.Toplevel(parent)
    top.title(title)
    top.transient(parent)
    top.grab_set()
    ttk.Label(top, text=prompt, padding=(12, 12, 12, 4)).pack(anchor="w")
    var = tk.StringVar(value=initial)
    e = ttk.Entry(top, textvariable=var, width=32)
    e.pack(padx=12)
    e.focus_set()
    e.select_range(0, "end")
    result = {"v": None}

    def ok():
        result["v"] = var.get()
        top.destroy()

    bf = ttk.Frame(top, padding=12)
    bf.pack()
    ttk.Button(bf, text="OK", command=ok).pack(side="left", padx=4)
    ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")
    e.bind("<Return>", lambda _e: ok())
    parent.wait_window(top)
    return result["v"]


def simpledialog_list(parent, title, prompt, count):
    """多值输入：换行、逗号或空格分隔"""
    top = tk.Toplevel(parent)
    top.title(title)
    top.transient(parent)
    top.grab_set()
    ttk.Label(top, text=f"{prompt}\n（当前读写定义数量 {count}，也可以少于这个数）",
              padding=(12, 12, 12, 4), justify="left").pack(anchor="w")
    txt = tk.Text(top, width=30, height=8, font=("Consolas", 10))
    txt.pack(padx=12)
    txt.focus_set()
    result = {"v": None}

    def ok():
        raw = txt.get("1.0", "end").replace(",", " ").split()
        if not raw:
            messagebox.showerror("输入为空", "请至少填一个数值", parent=top)
            return
        try:
            result["v"] = [int(x, 0) for x in raw]
        except ValueError:
            messagebox.showerror("格式错误", "只能填整数，可用 0x 前缀写十六进制", parent=top)
            return
        top.destroy()

    bf = ttk.Frame(top, padding=12)
    bf.pack()
    ttk.Button(bf, text="OK", command=ok).pack(side="left", padx=4)
    ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")
    parent.wait_window(top)
    return result["v"]


def simpledialog_onoff(parent, title):
    top = tk.Toplevel(parent)
    top.title(title)
    top.transient(parent)
    top.grab_set()
    ttk.Label(top, text="线圈值:", padding=(12, 12, 12, 4)).pack(anchor="w")
    var = tk.StringVar(value="ON")
    f = ttk.Frame(top, padding=(12, 0, 12, 12))
    f.pack()
    ttk.Radiobutton(f, text="ON (0xFF00)", variable=var, value="ON").pack(side="left")
    ttk.Radiobutton(f, text="OFF (0x0000)", variable=var, value="OFF").pack(side="left", padx=10)
    result = {"v": None}

    def ok():
        result["v"] = (var.get() == "ON")
        top.destroy()

    bf = ttk.Frame(top, padding=(12, 0, 12, 12))
    bf.pack()
    ttk.Button(bf, text="OK", command=ok).pack(side="left", padx=4)
    ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")
    parent.wait_window(top)
    return result["v"]


HELP_TEXT = """Modbus Poll Lite —— 操作说明

一、连接（F3）
    连接 → 连接…
    选 TCP/IP，填 IP 和端口（标准 502），填从站号 Unit ID。
    连上后左下角显示 ● IP:端口，右上角 No connection 消失。

二、读写定义（F8）
    设置 → 读写定义…
      Slave ID      从站号，必须与从站一致
      Function      功能码，读用 01/02/03/04，写用 05/06/15/16
      Address       起始地址（协议地址，从 0 起）
      Quantity      读几个，03/04 上限 125，01/02 上限 2000
      Scan Rate     轮询周期，单位毫秒
      显示格式      Signed / Unsigned / Hex / Binary / Float / Long
      32 位字序     读 Float / Long 时，两个字的高低顺序
    勾上「Read/Write Enabled」才开始轮询。

三、写入
    双击表格里的一行 → 弹框输入数值。
      当前功能码 01（线圈）    → 发 FC05 写单线圈
      当前功能码 03（保持寄存器）→ 发 FC06 写单寄存器
    菜单 功能 → 写单线圈(05) / 写单寄存器(06) / 写多线圈(15) / 写多寄存器(16)
      选 15 / 16 可一次写多个值，用换行、逗号或空格分隔。
    02 离散输入 / 04 输入寄存器 是只读的，写不了。
    写完会立刻读一次刷新，不用干等下一个轮询周期。
    从站号填 0 是广播，按规范从站不应答，发完即算成功。

三之二、别名
    双击「别名」列，或右键某行 → 设置别名，可给地址写中文注释（留空即清除）。

四、看报文
    显示 → 报文
    十六进制显示本工具收发的每一帧，异常响应也会标出来。

四之二、缩放与条件着色（读写定义对话框里设）
    缩放：工程量 = 原始值 × 系数 + 偏移，可带单位（如 rpm / ℃ / mm）。
          只对 Signed / Unsigned / Float 生效；Hex、Binary、Long 显示原始值。
    条件着色：填了下限/上限后，数值越界的行整行标红。留空即不启用。

四之三、数据记录
    显示 → 开始记录到 CSV…  选文件名，之后每个采样周期落一行（带时间戳）。
    显示 → 停止记录 CSV     关闭文件。
    文件是 UTF-8 带 BOM 的 CSV，Excel 直接双击打开不乱码。

四之四、配置保存
    文件 → 保存配置…   把连接参数、读写定义、缩放、别名一起存成 .mplite
    文件 → 打开配置…   下次直接载入，不用重填

四之五、地址扫描
    视图 → 地址扫描…
    选功能码（01/02/03/04）和地址范围，逐地址试读一遍。
    用途：摸不清从站地址表时，扫一遍就知道哪些地址有效。
    注意：扫描会占用连接，先把 Read/Write Enabled 取消掉。

五、地址基准（重要）
    显示 → 地址基准
      Base 0   协议地址，从 0 起            ← 手册口径，推荐
      Base 1   PLC 风格，40001 / 30001 …
    只影响表格里的地址显示，不影响实际读写的地址。

六、机器人地址表
    视图 → 机器人地址表，或工具栏上的四个快捷按钮。
    读错地址段，从站回异常码 02（Illegal Data Address）。

七、常见异常码
    01 非法功能码      从站不支持该功能码
    02 非法数据地址    地址超出从站允许范围
    03 非法数据值      数量超限（如 03 读超过 125 个）
    04 从站设备故障    从站内部出错，查从站自身报警
    06 从站忙          从站被其他主站占用

八、排查顺序
    1) 包/Err 都不动        → 没连上（检查 IP、端口、从站号）
    2) Err 一直涨、有异常码 → 连上了，但地址/功能码不对
    3) 数值不对             → 显示格式或字序设错
"""


def main():
    try:
        app = PollLite()
    except Exception as e:
        print("启动失败:", e)
        return 1
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
