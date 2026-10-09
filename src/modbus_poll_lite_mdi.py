# -*- coding: utf-8 -*-
"""
Modbus Poll Lite —— 多窗口版（MDI）

与单窗口版 modbus_poll_lite.py 的关系：
    复用它的核心（ModbusMaster / 功能码常量 / 异常码表 / 简单对话框），
    把"数据区"抽成独立的 DataArea 类，每个数据区是一个独立窗口，
    可以同时盯多个数据区（比如机器人 IO 一个、位置寄存器一个）。

    ⚠ 依赖同目录的 modbus_poll_lite.py，别删。

运行：PYTHONIOENCODING=utf-8 python src/modbus_poll_lite_mdi.py

差异（相对单窗口版）：
    · 多窗口：文件 → 新建数据区 / 关闭数据区；窗口菜单列出全部数据区
    · Base 0/1 基准是每个数据区各自持有的
    · 菜单挂在主窗口上，作用于"当前活动数据区"（点一下窗口标题栏即可切换）
    其余操作（F3 连接 / F8 读写定义 / 双击写值 / 报文 / 缩放 / 着色 /
    CSV 记录 / 配置保存 / 地址扫描）与单窗口版完全一致。
"""

import json
import os
import queue
import struct
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import modbus_poll_lite as core          # noqa: E402  复用核心
import modbus_serial as ms               # noqa: E402  串口 RTU / ASCII
from modbus_chart import ChartWindow     # noqa: E402  实时曲线
from modbus_testcenter import TestCenter  # noqa: E402  Test Center 手搓报文

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

APP = "Modbus Poll Lite MDI"
DEFAULT_GEOM = "980x520"


# ==================================================================== 数据区

class DataArea(tk.Toplevel):
    """一个数据区窗口：一套读写定义 + 一份数据 + 一个连接 + 一张表"""

    def __init__(self, master, index):
        super().__init__(master)
        self.root = master
        self.index = index
        self.title(f"数据区 {index} —— 未连接")
        self.geometry(DEFAULT_GEOM)
        self.minsize(780, 420)

        # ---- 主站 ----
        self.mb = core.ModbusMaster()
        self.poll_thread = None
        self.stop_flag = threading.Event()
        self.data_lock = threading.Lock()

        # ---- 读写定义（默认与 Modbus Poll 出厂一致）----
        self.slave_id = 1
        self.fc = 3
        self.addr = 0
        self.qty = 10
        self.scan_rate = 1000
        self.enabled = False
        self.fmt = "Signed"
        self.word_order = core.WORD_ORDERS[0]
        self.max_rows = 10
        self.var_base = tk.IntVar(value=0)

        # ---- 缩放 / 条件着色 ----
        self.scale = 1.0
        self.offset = 0.0
        self.unit = ""
        self.alarm_low = None
        self.alarm_high = None

        # ---- 数据 ----
        self.values = []
        self.err_state = None
        self.alias = {}

        # ---- 记录 ----
        self.log_fh = None
        self.log_lock = threading.Lock()
        self.log_path = ""
        self.last_ip = "127.0.0.1"
        self.last_port = 502
        self.last_timeout = 1000
        self.last_type = "TCP"                     # TCP / RTU / ASCII
        self.last_serial = {"port": "", "baud": "9600", "bytesize": 8,
                            "parity": "N", "stopbits": 1, "rts": False, "timeout": 1.0}

        # ---- 报文 / 曲线 ----
        self.traffic_win = None
        self.traffic_queue = queue.Queue()
        self.mb.on_traffic = self._traffic_cb
        self.chart_win = None
        self.test_win = None
        self.sample_no = 0            # 每次成功读取 +1，供曲线窗口判断"有没有新数据"

        self._build_toolbar()
        self._build_status()
        self._build_grid()

        self.bind("<F3>", lambda e: self.dlg_connect())
        self.bind("<F8>", lambda e: self.dlg_definition())
        self.bind("<FocusIn>", lambda e: master.set_active(self))
        self.protocol("WM_DELETE_WINDOW", self.close)

        self._update_status()
        self._refresh_grid()
        self._tick()

    # ---------------------------------------------------------------- 界面
    def _build_toolbar(self):
        bar = ttk.Frame(self, padding=(6, 4))
        bar.pack(fill="x")
        ttk.Button(bar, text="连接 (F3)", command=self.dlg_connect).pack(side="left")
        ttk.Button(bar, text="断开", command=self._disconnect).pack(side="left", padx=(4, 10))
        ttk.Button(bar, text="读写定义 (F8)", command=self.dlg_definition).pack(side="left", padx=(0, 10))
        ttk.Label(bar, text="功能码:").pack(side="left")
        for code in (1, 2, 3, 4, 5, 6, 15, 16):
            ttk.Button(bar, text=f"{code:02d}", width=3,
                       command=lambda c=code: self._quick_fc(c)).pack(side="left", padx=1)
        ttk.Button(bar, text="报文", command=self.dlg_traffic).pack(side="left", padx=(10, 0))

        bar2 = ttk.Frame(self, padding=(6, 0))
        bar2.pack(fill="x")
        ttk.Label(bar2, text="机器人地址表:").pack(side="left")
        for name, rng, fc, addr in core.ROBOT_RANGES:
            ttk.Button(bar2, text=f"{name} {rng}", width=17,
                       command=lambda f=fc, a=addr: self._preset(f, a)).pack(side="left", padx=2)

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
        self.tree = ttk.Treeview(f, columns=("addr", "alias", "value"),
                                 show="headings", height=20)
        self.tree.heading("addr", text="地址")
        self.tree.heading("alias", text="别名")
        self.tree.heading("value", text="数值")
        self.tree.column("addr", width=130, anchor="w", stretch=False)
        self.tree.column("alias", width=180, anchor="w", stretch=False)
        self.tree.column("value", width=240, anchor="w")
        self.tree.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(f, orient="vertical", command=self.tree.yview)
        sb.pack(side="right", fill="y")
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.tag_configure("err", foreground="#c00")
        self.tree.tag_configure("ok", foreground="#000")
        self.tree.tag_configure("alarm", foreground="#900", background="#ffd9d9")
        self.tree.bind("<Double-1>", self._on_double_click)
        self.tree.bind("<Button-3>", self._on_right_click)

        self.lbl_hint = ttk.Label(self, text="", foreground="#666", padding=(8, 2))
        self.lbl_hint.pack(fill="x")

    # ------------------------------------------------------------ 连接对话框
    def dlg_connect(self):
        top = tk.Toplevel(self)
        top.title(f"Connection Setup —— 数据区 {self.index}")
        top.transient(self)
        top.grab_set()
        top.resizable(False, False)

        v_type = tk.StringVar(value=self.last_type)
        tf = ttk.LabelFrame(top, text="Connection（连接方式）", padding=10)
        tf.pack(fill="x", padx=12, pady=(12, 6))
        for txt, val in (("TCP/IP（Modbus TCP）", "TCP"),
                         ("串口 RTU", "RTU"),
                         ("串口 ASCII", "ASCII")):
            ttk.Radiobutton(tf, text=txt, variable=v_type,
                            value=val).pack(side="left", padx=(0, 16))

        vf = ttk.Frame(top)
        vf.pack(fill="x", padx=14)
        ttk.Label(vf, text="从站号 Unit ID").pack(side="left")
        v_unit = tk.StringVar(value=str(self.slave_id))
        ttk.Entry(vf, textvariable=v_unit, width=8).pack(side="left", padx=8)
        ttk.Label(vf, text="（0 = 广播，从站不应答）", foreground="#666").pack(side="left")

        # ------------------------------------------------ TCP 参数
        tcpf = ttk.LabelFrame(top, text="TCP 参数", padding=10)
        tcpf.pack(fill="x", padx=12, pady=6)
        v_ip = tk.StringVar(value=self.last_ip)
        v_port = tk.StringVar(value=str(self.last_port))
        v_to = tk.StringVar(value=str(self.last_timeout))
        for r, (lab, var, w) in enumerate([("IP 地址", v_ip, 18),
                                           ("端口 Port", v_port, 8),
                                           ("响应超时 (ms)", v_to, 8)]):
            ttk.Label(tcpf, text=lab).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Entry(tcpf, textvariable=var, width=w).grid(row=r, column=1, sticky="w", padx=8)
        ttk.Label(tcpf, text="提示：机器人控制器网口 2 固定为 192.168.23.25，端口 502",
                  foreground="#666").grid(row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))

        # ------------------------------------------------ 串口参数
        serf = ttk.LabelFrame(top, text="串口参数", padding=10)
        serf.pack(fill="x", padx=12, pady=6)
        s = self.last_serial
        v_sport = tk.StringVar(value=s["port"])
        v_baud = tk.StringVar(value=s["baud"])
        v_bytes = tk.StringVar(value=str(s["bytesize"]))
        v_par = tk.StringVar(value=next(t for t, c in ms.PARITY_CHOICES if c == s["parity"]))
        v_stop = tk.StringVar(value=next(t for t, v in ms.STOP_CHOICES if v == s["stopbits"]))
        v_rts = tk.BooleanVar(value=s["rts"])
        v_sto = tk.StringVar(value=str(s["timeout"]))

        ttk.Label(serf, text="串口").grid(row=0, column=0, sticky="w", pady=3)
        cb_port = ttk.Combobox(serf, textvariable=v_sport, width=10)
        cb_port.grid(row=0, column=1, sticky="w", padx=8)
        lbl_ports = ttk.Label(serf, text="", foreground="#666")
        lbl_ports.grid(row=0, column=2, sticky="w", padx=4)

        def refresh_ports():
            ps = ms.available_ports()
            cb_port.configure(values=[d for d, _ in ps])
            lbl_ports.configure(
                text=("检测到 " + "，".join(f"{d}" for d, _ in ps)) if ps
                else "★ 没检测到串口 —— 检查 USB 转串口线插好没有、驱动正常没有")

        ttk.Button(serf, text="刷新", command=refresh_ports).grid(row=0, column=3, padx=4)
        refresh_ports()

        ttk.Label(serf, text="波特率").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Combobox(serf, textvariable=v_baud, width=10,
                     values=ms.BAUD_CHOICES).grid(row=1, column=1, sticky="w", padx=8)
        ttk.Label(serf, text="数据位").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Combobox(serf, textvariable=v_bytes, width=10, state="readonly",
                     values=[str(v) for _, v in ms.BYTE_CHOICES]).grid(
            row=2, column=1, sticky="w", padx=8)
        ttk.Label(serf, text="校验位").grid(row=3, column=0, sticky="w", pady=3)
        ttk.Combobox(serf, textvariable=v_par, width=18, state="readonly",
                     values=[t for t, _ in ms.PARITY_CHOICES]).grid(
            row=3, column=1, sticky="w", padx=8)
        ttk.Label(serf, text="停止位").grid(row=4, column=0, sticky="w", pady=3)
        ttk.Combobox(serf, textvariable=v_stop, width=18, state="readonly",
                     values=[t for t, _ in ms.STOP_CHOICES]).grid(
            row=4, column=1, sticky="w", padx=8)
        ttk.Label(serf, text="超时 (s)").grid(row=5, column=0, sticky="w", pady=3)
        ttk.Entry(serf, textvariable=v_sto, width=10).grid(row=5, column=1, sticky="w", padx=8)
        ttk.Checkbutton(serf, text="RTS toggle（部分 USB-RS485 转换器需要勾）",
                        variable=v_rts).grid(row=6, column=0, columnspan=3, sticky="w", pady=(6, 0))

        # 按所选方式灰掉另一组
        def set_state(w, on):
            try:
                w.configure(state="normal" if on else "disabled")
            except tk.TclError:
                pass
            for c in w.winfo_children():
                set_state(c, on)

        def sync(*_):
            t = v_type.get()
            set_state(tcpf, t == "TCP")
            set_state(serf, t != "TCP")

        v_type.trace_add("write", sync)
        sync()

        def do_connect():
            try:
                unit = int(v_unit.get(), 0)
                if not 0 <= unit <= 255:
                    raise ValueError(f"从站号 Unit ID 超出范围：0 ~ 255，你填的是 {unit}")
                t = v_type.get()
                if t == "TCP":
                    self.last_ip = v_ip.get().strip()
                    self.last_port = int(v_port.get())
                    self.last_timeout = int(v_to.get())
                    newmb = core.ModbusMaster()
                    newmb.unit = unit
                    newmb.connect(self.last_ip, self.last_port, unit,
                                  self.last_timeout / 1000.0)
                else:
                    port = v_sport.get().strip()
                    if not port:
                        raise ValueError("请选择串口（点「刷新」重新枚举）")
                    par = {lb: c for lb, c in ms.PARITY_CHOICES}.get(v_par.get(), "N")
                    stop = {lb: v for lb, v in ms.STOP_CHOICES}.get(v_stop.get(), 1)
                    timeout = float(v_sto.get() or 1.0)
                    self.last_serial = {"port": port, "baud": v_baud.get(),
                                        "bytesize": int(v_bytes.get()), "parity": par,
                                        "stopbits": int(stop), "rts": v_rts.get(),
                                        "timeout": timeout}
                    newmb = ms.ModbusSerialMaster(mode=t)
                    newmb.unit = unit
                    newmb.connect(port, baud=v_baud.get(), bytesize=int(v_bytes.get()),
                                  parity=par, stopbits=int(stop), timeout=timeout,
                                  rts_toggle=v_rts.get())
                newmb.on_traffic = self._traffic_cb
                newmb.traffic = self.mb.traffic          # 沿用报文历史
                old = self.mb
                self.mb = newmb
                self.last_type = t
                try:
                    old.close()
                except Exception:
                    pass
            except Exception as e:
                messagebox.showerror("连接失败", f"{type(e).__name__}: {e}", parent=top)
                self._update_status()
                return
            top.destroy()
            self._update_status()
            self._start_poll()

        bf = ttk.Frame(top, padding=(12, 4, 12, 12))
        bf.pack(fill="x")
        ttk.Button(bf, text="OK", command=do_connect).pack(side="right", padx=4)
        ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="right")

    # -------------------------------------------------------- 读写定义对话框
    def dlg_definition(self):
        top = tk.Toplevel(self)
        top.title(f"Read/Write Definition —— 数据区 {self.index}")
        top.transient(self)
        top.grab_set()
        top.resizable(False, False)

        v_slave = tk.StringVar(value=str(self.slave_id))
        v_func = tk.StringVar(value=core.FC_LABEL[self.fc])
        v_addr = tk.StringVar(value=str(self.addr))
        v_qty = tk.StringVar(value=str(self.qty))
        v_rate = tk.StringVar(value=str(self.scan_rate))
        v_on = tk.BooleanVar(value=self.enabled)
        v_rows = tk.IntVar(value=self.max_rows)
        v_fmt = tk.StringVar(value=self.fmt)
        v_ord = tk.StringVar(value=self.word_order)
        v_scale = tk.StringVar(value="%g" % self.scale)
        v_offset = tk.StringVar(value="%g" % self.offset)
        v_unit = tk.StringVar(value=self.unit)
        v_low = tk.StringVar(value="" if self.alarm_low is None else "%g" % self.alarm_low)
        v_high = tk.StringVar(value="" if self.alarm_high is None else "%g" % self.alarm_high)

        f = ttk.Frame(top, padding=12)
        f.pack(fill="both", expand=True)

        def row(r, label, widget):
            ttk.Label(f, text=label).grid(row=r, column=0, sticky="w", pady=4)
            widget.grid(row=r, column=1, sticky="w", padx=10)

        row(0, "Slave ID（从站号）", ttk.Entry(f, textvariable=v_slave, width=12))
        row(1, "Function（功能码）",
            ttk.Combobox(f, textvariable=v_func, state="readonly", width=38,
                         values=[t for _, t in core.FUNCTIONS]))
        row(2, "Address（起始地址）", ttk.Entry(f, textvariable=v_addr, width=12))
        row(3, "Quantity（数量）", ttk.Entry(f, textvariable=v_qty, width=12))
        row(4, "Scan Rate（扫描周期 ms）", ttk.Entry(f, textvariable=v_rate, width=12))
        row(5, "显示格式", ttk.Combobox(f, textvariable=v_fmt, state="readonly",
                                     width=14, values=core.FORMATS))
        row(6, "32 位字序", ttk.Combobox(f, textvariable=v_ord, state="readonly",
                                      width=18, values=core.WORD_ORDERS))

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
                          "※ 缩放与条件着色只对 Signed / Unsigned / Float 生效。",
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
                fc = next(c for c, t in core.FUNCTIONS if t == v_func.get())
                addr = int(v_addr.get(), 0)
                if not 0 <= addr <= 65535:
                    raise ValueError(f"Address（起始地址）超出范围：0 ~ 65535，你填的是 {addr}。")
                qty = int(v_qty.get(), 0)
                limit = 2000 if fc in core.BIT_FUNCS else 125
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

            self.slave_id, self.fc, self.addr, self.qty = slave, fc, addr, qty
            self.scan_rate, self.enabled, self.max_rows = rate, v_on.get(), v_rows.get()
            self.fmt, self.word_order = v_fmt.get(), v_ord.get()
            self.scale, self.offset, self.unit = scale, offs, v_unit.get().strip()
            self.alarm_low, self.alarm_high = low, high
            self.mb.unit = self.slave_id
            top.destroy()
            self._update_status()
            self._refresh_grid()
            if self.enabled and not self.mb.connected:
                if messagebox.askyesno("还没连接",
                                       "读写定义已保存，但当前没有连接到从站。\n\n现在打开连接设置吗？"):
                    self.dlg_connect()

        bf = ttk.Frame(f)
        bf.grid(row=12, column=0, columnspan=2, sticky="e", pady=(12, 0))
        ttk.Button(bf, text="OK", command=apply).pack(side="left", padx=4)
        ttk.Button(bf, text="Cancel", command=top.destroy).pack(side="left")

    # ------------------------------------------------------------ 写入
    def _row_step(self):
        return 2 if (self.fc not in core.BIT_FUNCS and self.fmt in ("Float", "Long")) else 1

    def _write_dialog(self, fc):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("提示", "先在表格里选中一行")
            return
        addr = self.addr + self.tree.index(sel[0]) * self._row_step()
        try:
            if fc == 5:
                v = core.simpledialog_onoff(self, f"写单线圈 (05)   地址 {addr}")
                if v is None:
                    return
                self.mb.write_single_coil(addr, v)
            elif fc == 6:
                v = core.simpledialog_int(self, f"写单寄存器 (06)   地址 {addr}", "数值 (0~65535):")
                if v is None:
                    return
                self.mb.write_single_register(addr, v)
            elif fc == 15:
                vals = core.simpledialog_list(self, f"写多线圈 (15)   起始地址 {addr}",
                                              "每行或逗号分隔，0 / 1：", self.qty)
                if vals is None:
                    return
                self.mb.write_multiple_coils(addr, [int(x) != 0 for x in vals])
            elif fc == 16:
                vals = core.simpledialog_list(self, f"写多寄存器 (16)   起始地址 {addr}",
                                              "每行或逗号分隔，0 ~ 65535：", self.qty)
                if vals is None:
                    return
                self.mb.write_multiple_registers(addr, vals)
            else:
                return
        except core.ModbusError as e:
            messagebox.showerror("从站返回异常", core.exc_text(e.code))
            return
        except Exception as e:
            messagebox.showerror("写入失败", str(e))
            return
        self._read_once()

    def _on_double_click(self, event):
        row = self.tree.identify_row(event.y)
        if not row:
            return
        self.tree.selection_set(row)
        if self.tree.identify_column(event.x) == "#2":
            self._edit_alias(self.tree.index(row))
            return
        wf = core.WRITE_FOR_READ.get(self.fc)
        if wf is None:
            messagebox.showinfo("提示",
                                f"功能码 {self.fc:02d} 是只读的。\n"
                                "只能写线圈(01/15) 或 保持寄存器(03/16)。")
            return
        self._write_dialog(wf)

    def _edit_alias(self, row):
        addr = self.addr + row * self._row_step()
        new = core.simpledialog_text(self, f"设置别名   地址 {addr}",
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
            wf = core.WRITE_FOR_READ.get(self.fc)
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

    # ------------------------------------------------------------ 数值转换
    def _raw_number(self, vals, i):
        if self.fc in core.BIT_FUNCS:
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
        return n * self.scale + self.offset

    def _format_value(self, vals, i):
        if self.fc in core.BIT_FUNCS:
            return "1  ON" if vals[i] else "0  OFF"
        if self.fmt in ("Hex", "Binary"):
            v = vals[i]
            return f"0x{v:04X}" if self.fmt == "Hex" else f"{v:016b}"
        n = self._raw_number(vals, i)
        if n is None:
            return "-"
        if self.fmt == "Long":
            return str(int(n))
        if self.fmt == "Float":
            out = f"{self._scaled(n):.4f}"
        else:
            out = "%g" % self._scaled(n)
        return out + (" " + self.unit if self.unit else "")

    def _alarm_state(self, vals, i):
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

    def _plot_value(self, vals, i):
        """曲线用的数值：与表格显示口径一致（该缩放的缩放）"""
        n = self._raw_number(vals, i)
        if n is None:
            return None
        if self.fmt in ("Hex", "Binary", "Long"):
            return n
        return self._scaled(n)

    def _disp_addr(self, addr):
        if not self.var_base.get():
            return str(addr)
        base = {1: 1, 2: 10001, 4: 30001}.get(self.fc, 40001)
        return f"{addr + base:05d}"

    # ------------------------------------------------------------ 轮询
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
            if self.enabled and self.mb.connected and self.fc not in core.WRITE_FUNCS:
                try:
                    vals = self.mb.read(self.fc, self.addr, self.qty)
                    with self.data_lock:
                        self.values = vals
                        self.err_state = None
                        self.sample_no += 1
                    self._log_values(vals)
                except core.ModbusError as e:
                    with self.data_lock:
                        self.values = []
                        self.err_state = core.exc_text(e.code)
                except Exception as e:
                    with self.data_lock:
                        self.values = []
                        self.err_state = f"{type(e).__name__}: {e}"
                    self.stop_flag.wait(1.0)
                    continue
            self.stop_flag.wait(max(0.05, self.scan_rate / 1000.0))

    def _read_once(self):
        if not self.mb.connected or self.fc in core.WRITE_FUNCS:
            return
        try:
            vals = self.mb.read(self.fc, self.addr, self.qty)
            with self.data_lock:
                self.values = vals
                self.err_state = None
                self.sample_no += 1
            self._log_values(vals)
        except core.ModbusError as e:
            with self.data_lock:
                self.values = []
                self.err_state = core.exc_text(e.code)
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
        try:
            self._refresh_grid()
            self._update_status()
        except tk.TclError:
            return
        if self.winfo_exists():
            self.after(200, self._tick)

    # ------------------------------------------------------------ 刷新
    def _refresh_grid(self):
        if not self.winfo_exists():
            return
        with self.data_lock:
            vals = list(self.values)
            err = self.err_state

        step = self._row_step()
        n_show = min(self.max_rows, max(1, self.qty // step)) if vals else self.max_rows

        if len(self.tree.get_children()) != n_show:
            self.tree.delete(*self.tree.get_children())
            for i in range(n_show):
                self.tree.insert("", "end", iid=f"r{i}",
                                 values=(self._disp_addr(self.addr + i * step),
                                         self.alias.get(self.addr + i * step, ""), "—"),
                                 tags=("ok",))

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
                if self.fc not in core.BIT_FUNCS:
                    tag = self._alarm_state(vals, i * step) or "ok"
            else:
                txt, tag = "—", "ok"
            self.tree.item(iid, values=(self._disp_addr(a), self.alias.get(a, ""), txt),
                           tags=(tag,))

        hint = (f"{core.FC_LABEL[self.fc]}   起始地址 {self.addr}   数量 {self.qty}   "
                f"格式 {self.fmt}   周期 {self.scan_rate}ms   行数 {n_show}")
        if self.fc in core.WRITE_FUNCS:
            self.lbl_hint.configure(
                foreground="#06c",
                text=f"✎ 写模式（{core.FC_LABEL[self.fc]}）—— 本窗口不轮询，"
                     f"用右键菜单或双击行发起写入。      " + hint)
        elif err:
            self.lbl_hint.configure(foreground="#c00",
                                    text=hint + f"    ← 从站返回：{err}")
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
        if not self.winfo_exists():
            return
        self.lbl_def.configure(
            text=f"Tx = {self.mb.tx}  Err = {self.mb.err}  ID = {self.slave_id}  "
                 f"F = {self.fc:02d}: SR = {self.scan_rate}ms"
                 + ("" if self.enabled else "  (DISABLED)"))
        if self.mb.connected:
            self.lbl_conn.configure(text=f"  ● {self.mb.peer}", foreground="#0a0")
            self.title(f"数据区 {self.index} —— {self.mb.peer}")
        else:
            self.lbl_conn.configure(text="  No connection", foreground="#c00")
            self.title(f"数据区 {self.index} —— 未连接")

    # ------------------------------------------------------------ 动作
    def _quick_fc(self, code):
        self.fc = code
        self._update_status()
        self._refresh_grid()

    def _preset(self, fc, addr):
        self.fc, self.addr, self.qty = fc, addr, 10
        self.enabled, self.fmt = True, "Signed"
        self._update_status()
        self._refresh_grid()

    def _cycle_format(self):
        self.fmt = core.FORMATS[(core.FORMATS.index(self.fmt) + 1) % len(core.FORMATS)]
        self._refresh_grid()

    def _disconnect(self):
        self._stop_poll()
        self.mb.close()
        with self.data_lock:
            self.values = []
        self._update_status()
        self._refresh_grid()

    def reset(self):
        self._disconnect()
        self.fc, self.addr, self.qty, self.fmt = 3, 0, 10, "Signed"
        self.alias.clear()
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
            title=f"数据区 {self.index} 数据记录保存为", defaultextension=".csv",
            initialfile=time.strftime(f"modbus_log_a{self.index}_%Y%m%d_%H%M%S.csv"),
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
        messagebox.showinfo("开始记录", f"写入：\n{path}")

    # ------------------------------------------------------------ 配置
    def config(self):
        return {
            "ip": self.last_ip, "port": self.last_port, "timeout": self.last_timeout,
            "unit": self.slave_id, "fc": self.fc, "addr": self.addr, "qty": self.qty,
            "scan_rate": self.scan_rate, "fmt": self.fmt, "word_order": self.word_order,
            "base": self.var_base.get(), "scale": self.scale, "offset": self.offset,
            "unit_text": self.unit, "alarm_low": self.alarm_low,
            "alarm_high": self.alarm_high,
            "alias": {str(k): v for k, v in self.alias.items()},
        }

    def apply_config(self, cfg):
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
        self.word_order = cfg.get("word_order", core.WORD_ORDERS[0])
        self.var_base.set(int(cfg.get("base", 0)))
        self.scale = float(cfg.get("scale", 1.0))
        self.offset = float(cfg.get("offset", 0.0))
        self.unit = cfg.get("unit_text", "")
        self.alarm_low = cfg.get("alarm_low")
        self.alarm_high = cfg.get("alarm_high")
        self.alias = {int(k): v for k, v in (cfg.get("alias") or {}).items()}
        self.mb.unit = self.slave_id
        self._update_status()
        self._refresh_grid()

    def _save_config(self):
        path = filedialog.asksaveasfilename(
            title=f"数据区 {self.index} 保存配置", defaultextension=".mplite",
            initialfile=f"modbus_area{self.index}.mplite",
            filetypes=[("Modbus Poll Lite 配置", "*.mplite"),
                       ("JSON", "*.json"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self.config(), fh, ensure_ascii=False, indent=2)
        except Exception as e:
            messagebox.showerror("保存失败", str(e))
            return
        messagebox.showinfo("已保存", path)

    def _load_config(self):
        path = filedialog.askopenfilename(
            title=f"数据区 {self.index} 打开配置",
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
            self.apply_config(cfg)
        except Exception as e:
            messagebox.showerror("配置内容有误", str(e))
            return
        messagebox.showinfo("已载入", f"{path}\n\n按 F3 连接。")

    # ------------------------------------------------------------ 报文
    def _traffic_cb(self, line):
        self.traffic_queue.put(line)

    def _append_traffic(self, line):
        if not (self.traffic_win and self.traffic_win.winfo_exists()):
            return
        d, hexs, note = line
        txt = self.traffic_win.text
        txt.configure(state="normal")
        txt.insert("end", f"{d}  {hexs}\n")
        if note:
            txt.insert("end", f"    {note}\n")
        txt.see("end")
        txt.configure(state="disabled")

    def dlg_chart(self):
        """实时曲线窗口（每个数据区一个）"""
        if self.chart_win and self.chart_win.winfo_exists():
            self.chart_win.lift()
            self.chart_win.focus_force()
            return
        self.chart_win = ChartWindow(self)

    def dlg_testcenter(self):
        """Test Center：手搓报文，看从站原始应答"""
        if self.test_win and self.test_win.winfo_exists():
            self.test_win.lift()
            self.test_win.focus_force()
            return
        if not self.mb.connected:
            messagebox.showwarning("未连接", "Test Center 需要先连接从站（F3）")
            return
        self.test_win = TestCenter(self)

    def dlg_traffic(self):
        if self.traffic_win and self.traffic_win.winfo_exists():
            self.traffic_win.lift()
            return
        top = tk.Toplevel(self)
        top.title(f"Communication —— 数据区 {self.index}")
        top.geometry("640x400")
        txt = tk.Text(top, font=("Consolas", 9), wrap="none")
        txt.pack(fill="both", expand=True)
        txt.configure(state="disabled")
        top.text = txt
        self.traffic_win = top
        for line in self.mb.traffic:
            self._append_traffic(line)

    # ------------------------------------------------------------ 地址扫描
    def dlg_scan(self):
        top = tk.Toplevel(self)
        top.title(f"地址扫描 —— 数据区 {self.index}")
        top.geometry("600x480")
        top.transient(self)

        f = ttk.Frame(top, padding=10)
        f.pack(fill="x")
        default_fc = self.fc if self.fc in (1, 2, 3, 4) else 3
        v_func = tk.StringVar(value=core.FC_LABEL[default_fc])
        v_from = tk.StringVar(value=str(self.addr))
        v_to = tk.StringVar(value=str(self.addr + 15))

        ttk.Label(f, text="功能码").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Combobox(f, textvariable=v_func, state="readonly", width=34,
                     values=[t for c, t in core.FUNCTIONS if c in (1, 2, 3, 4)]).grid(
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
        ttk.Label(top, text="※ 用 03 / 04 扫描可摸清从站寄存器分布；回异常码 02 = 不在从站允许范围内。",
                  foreground="#666", padding=(10, 0, 10, 8)).pack(anchor="w")

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
                except core.ModbusError as e:
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
                fc = next(c for c, t in core.FUNCTIONS if t == v_func.get())
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

    # ------------------------------------------------------------ 关闭
    def close(self):
        try:
            self._stop_poll()
            self.mb.close()
            with self.log_lock:
                if self.log_fh:
                    self.log_fh.close()
                    self.log_fh = None
        except Exception:
            pass
        self.root.forget_area(self)
        self.destroy()


# ==================================================================== 主窗口

class PollLiteMDI(tk.Tk):
    """主窗口：只管菜单、数据区管理和调度"""

    def __init__(self):
        super().__init__()
        self.title(APP)
        self.geometry("560x120")
        self.areas = []
        self.active = None
        self._seq = 0
        self._build_menu()
        self.lbl = ttk.Label(self, text="没有打开的数据区。\n文件 → 新建数据区（Ctrl+N）",
                             padding=24, justify="center", foreground="#666")
        self.lbl.pack(expand=True)
        self.bind("<Control-n>", lambda e: self.new_area())
        self.new_area()

    def _build_menu(self):
        m = tk.Menu(self)

        f = tk.Menu(m, tearoff=0)
        f.add_command(label="新建数据区", accelerator="Ctrl+N", command=self.new_area)
        f.add_command(label="关闭当前数据区", command=self.close_active)
        f.add_separator()
        f.add_command(label="打开当前数据区配置…", command=self._c("_load_config"))
        f.add_command(label="保存当前数据区配置…", command=self._c("_save_config"))
        f.add_separator()
        f.add_command(label="退出", command=self.destroy)
        m.add_cascade(label="文件", menu=f)

        a = tk.Menu(m, tearoff=0)
        a.add_command(label="连接…", accelerator="F3", command=self._c("dlg_connect"))
        a.add_command(label="断开", command=self._c("_disconnect"))
        a.add_command(label="读写定义…", accelerator="F8", command=self._c("dlg_definition"))
        a.add_separator()
        a.add_command(label="写单线圈 (05)…", command=self._c("_write_dialog", 5))
        a.add_command(label="写单寄存器 (06)…", command=self._c("_write_dialog", 6))
        a.add_separator()
        a.add_command(label="写多线圈 (15)…", command=self._c("_write_dialog", 15))
        a.add_command(label="写多寄存器 (16)…", command=self._c("_write_dialog", 16))
        a.add_separator()
        a.add_command(label="Test Center（手搓报文）…", command=self._c("dlg_testcenter"))
        m.add_cascade(label="数据区", menu=a)

        d = tk.Menu(m, tearoff=0)
        d.add_command(label="地址基准 Base 0（协议地址）",
                      command=lambda: self._set_base(0))
        d.add_command(label="地址基准 Base 1（PLC 地址）",
                      command=lambda: self._set_base(1))
        d.add_separator()
        d.add_command(label="循环切换显示格式", command=self._c("_cycle_format"))
        d.add_command(label="实时曲线", command=self._c("dlg_chart"))
        d.add_command(label="报文 (Communication)", command=self._c("dlg_traffic"))
        d.add_separator()
        d.add_command(label="开始记录到 CSV…", command=self._c("_toggle_log", True))
        d.add_command(label="停止记录 CSV", command=self._c("_toggle_log", False))
        m.add_cascade(label="显示", menu=d)

        v = tk.Menu(m, tearoff=0)
        v.add_command(label="机器人地址表", command=lambda: dlg_robot_ranges(self))
        v.add_command(label="地址扫描…", command=self._c("dlg_scan"))
        m.add_cascade(label="视图", menu=v)

        self.win_menu = tk.Menu(m, tearoff=0)
        m.add_cascade(label="窗口", menu=self.win_menu)

        h = tk.Menu(m, tearoff=0)
        h.add_command(label="使用说明", command=lambda: dlg_help(self))
        m.add_cascade(label="帮助", menu=h)

        self.config(menu=m)

    # -------------------------------------------------------------- 调度
    def _c(self, method, *args):
        """生成一个作用于当前活动数据区的菜单命令"""
        def run():
            area = self.active
            if area is None or not area.winfo_exists():
                messagebox.showinfo("提示", "没有打开的数据区。\n文件 → 新建数据区")
                return
            getattr(area, method)(*args)
        return run

    def _set_base(self, v):
        area = self.active
        if area is not None and area.winfo_exists():
            area.var_base.set(v)
            area._refresh_grid()

    def set_active(self, area):
        if area is self.active:
            return
        self.active = area
        self._refresh_window_menu()

    def new_area(self):
        self._seq += 1
        area = DataArea(self, self._seq)
        self.areas.append(area)
        self.active = area
        area.lift()
        area.focus_force()
        self._refresh_window_menu()
        return area

    def close_active(self):
        if self.active is not None and self.active.winfo_exists():
            self.active.close()

    def forget_area(self, area):
        if area in self.areas:
            self.areas.remove(area)
        if self.active is area:
            self.active = self.areas[-1] if self.areas else None
        self._refresh_window_menu()

    def _refresh_window_menu(self):
        self.win_menu.delete(0, "end")
        if not self.areas:
            self.win_menu.add_command(label="（没有数据区）", state="disabled")
        for area in self.areas:
            mark = "● " if area is self.active else "   "
            self.win_menu.add_command(
                label=f"{mark}数据区 {area.index}  {area.mb.peer or '未连接'}",
                command=lambda a=area: (a.lift(), a.focus_force(), self.set_active(a)))
        self.win_menu.add_separator()
        self.win_menu.add_command(label="新建数据区", command=self.new_area)


# ------------------------------------------------------------ 共享对话框

def dlg_robot_ranges(parent):
    top = tk.Toplevel(parent)
    top.title("机器人 Modbus 从站地址表")
    top.transient(parent)
    ttk.Label(top, text="机器人控制器 Modbus 从站地址表",
              font=("", 11, "bold"), padding=(12, 10, 12, 4)).pack(anchor="w")
    tv = ttk.Treeview(top, columns=("obj", "range", "fc", "rw"), show="headings", height=4)
    for c, t, w in (("obj", "对象", 130), ("range", "地址范围", 150),
                    ("fc", "功能码", 110), ("rw", "访问", 80)):
        tv.heading(c, text=t)
        tv.column(c, width=w, anchor="w")
    tv.pack(fill="x", padx=12)
    for obj, rng, fc, rw in [
            ("离散量输入", "0 ~ 4095", "02", "只读"),
            ("线圈", "4096 ~ 8191", "01 / 05 / 15", "读写"),
            ("输入寄存器", "0 ~ 32767", "04", "只读"),
            ("保持寄存器", "32768 ~ 65535", "03 / 06 / 16", "读写")]:
        tv.insert("", "end", values=(obj, rng, fc, rw))
    ttk.Label(top, text="※ 地址为协议地址（从 0 起）。读错段从站会回异常码 02 —— \n"
                        "   这正是官方课件里 Illegal Data Address 的成因。",
              foreground="#666", justify="left", padding=(12, 10)).pack(anchor="w")


def dlg_help(parent):
    top = tk.Toplevel(parent)
    top.title("使用说明")
    top.geometry("640x520")
    txt = tk.Text(top, wrap="word", font=("", 10), padx=12, pady=10)
    txt.pack(fill="both", expand=True)
    txt.insert("end", MDI_HELP + "\n\n" + "-" * 40 + "\n\n" + core.HELP_TEXT)
    txt.configure(state="disabled")


MDI_HELP = """多窗口版说明

〇、连接方式（F3）
    ○ TCP/IP          填 IP + 端口（标准 502）        —— 机器人、PLC 的网口
    ○ 串口 RTU        选串口 + 波特率 + 数据位/校验/停止位
    ○ 串口 ASCII      同上，帧格式不同（罕见）
    串口那一组：点「刷新」重新枚举串口；没检测到就去查 USB 转串口线的驱动。
    部分 USB-RS485 转换器要勾「RTS toggle」才能收发。
    ⚠ 串口参数的波特率/校验/停止位**三项必须与从站完全一致**，
      不一致的典型症状是"有发无回、一路超时"，从站连异常码都不回。

一、多窗口
    文件 → 新建数据区（Ctrl+N）   每开一个就是一个独立窗口、独立连接、独立数据区。
    文件 → 关闭当前数据区         关掉会同时停止它的轮询线程、关闭连接和记录文件。
    窗口 菜单                     列出所有数据区，点一下切过去。

二、当前活动数据区
    菜单栏上的「数据区 / 显示 / 视图」都作用于**当前活动数据区**。
    切换方法：点一下那个窗口的标题栏，或从「窗口」菜单选。
    每个数据区自己带一排工具栏，也可以直接点窗口里的按钮，效果一样。

三、典型用法
    调机器人时同时开两个数据区：
      数据区 1 → 点「线圈 4096~8191」    盯控制命令
      数据区 2 → 点「保持寄存器 32768~65535」 盯系统参数
    两个窗口各连各的，互不影响。

四、实时曲线（显示 → 实时曲线）
    数值随时间滚动。时间窗口 10 秒 ~ 15 分钟；Y 轴可自动量程或固定上下限。
    图例：双击某一路 = 只看它（再双击恢复全部）；「全选」「全不选」批量控制。
    多路量纲差异大时，用 solo 把无关路关掉，Y 轴才能贴合你想看的那一路。

    ⚠ 波形"卡"或"点少"怎么调：
      采样密度 = 1 ÷ 采样周期。周期 1000ms 就是每秒 1 个点，60 秒窗口只有 60 点，
      看着就是折线拐来拐去。**把工具栏的「采样周期」调小（100~200ms）波形立刻变密。**
      右上角状态栏会显示当前「N 点/秒」，低于 5 点/秒会变黄告警。
      · TCP 直连可以调到 50ms；串口受波特率限制（9600 波特下轮询周期别低于 50~100ms，
        否则上一帧还没回完就发下一帧，会一路超时）
      · 扫描周期同时会写回数据区，F8 里看到的也是同一个值
      · 采样点少于 120 个时，图上会把**真实采样位置**用小圆点标出来 ——
        免得两点之间的直线段被误当成"数据"

    游标（示波器式测量，仿伺服后台）
      点「游标：关」→ 开启，图上出现 T1 / T2 两条竖线，鼠标拖动即可移动。
      拖动时抓「离鼠标最近」的那条。
      **游标固定在屏幕上** —— 波形从底下滚过去，它一动不动。
      读数显示在**画布下方的独立读数区**（不叠在波形上，不会挡图）：
          T1 / T2       —— 游标位置，用 **X 轴坐标**表示（秒，与轴上 -10s / 现在 同口径）
          绝对时刻       —— 对应的墙上时间，对 CSV 日志用
          ΔT            —— 两游标时间差（= 时间窗口 × 屏距比例，是个定值）
          各通道 Y1 → Y2 —— 游标当前位置**下方**的波形值，运行中实时刷新
          Δ               —— 幅值差
      ⚠ 游标拖到数据范围之外会显示「—」而不是 0 —— 那不是"没变化"，是没有数据。

⚠ 连接数提醒
    每个数据区是独立连接。机器人控制器**最多只支持 4 个 Modbus 连接**，
    开太多会把别人挤掉。同参数的连接复用还没做，先用 2~3 个窗口为宜。
"""


def selftest(show_dialog=True):
    """打包自检：确认关键依赖都在。

    为什么需要：pyserial 在 modbus_serial.py 里是 try/except 导入的
    （为了没装时优雅降级），缺了它程序照样起得来，只是**串口打不开**
    而且不报错。打包后必须显式验一次。

    跑法：modbus_poll_lite_mdi.exe --selftest
    结果同时写 %TEMP%\\modbus_selftest.txt 并弹窗（--windowed 没控制台）
    """
    import platform
    lines = [f"Modbus Poll Lite MDI 自检",
             f"Python {platform.python_version()}",
             f"打包运行 frozen = {getattr(sys, 'frozen', False)}",
             f"解包目录 = {getattr(sys, '_MEIPASS', '（非打包模式）')}",
             ""]
    ok = True
    for name in ("tkinter", "serial", "serial.tools.list_ports",
                 "modbus_poll_lite", "modbus_chart", "modbus_serial",
                 "modbus_testcenter"):
        try:
            mod = __import__(name)
            ver = getattr(mod, "__version__", "")
            lines.append(f"  ✔ {name}" + (f"  {ver}" if ver else ""))
        except Exception as e:
            ok = False
            lines.append(f"  ✘ {name}    {type(e).__name__}: {e}")

    try:
        import serial.tools.list_ports as lp
        ports = [p.device for p in lp.comports()]
        lines.append(f"\n串口枚举：{ports if ports else '（当前没插串口设备）'}")
    except Exception as e:
        lines.append(f"\n串口枚举失败：{type(e).__name__}: {e}")

    lines.append(f"\n结论：{'✔ 全部通过' if ok else '✘ 有缺失 —— 串口等功能可能不可用'}")
    text = "\n".join(lines)

    out = os.path.join(tempfile.gettempdir(), "modbus_selftest.txt")
    try:
        with open(out, "w", encoding="utf-8") as fh:
            fh.write(text)
    except Exception:
        out = "（写文件失败）"
    print(text)
    # 打包后没有控制台，不弹窗用户就什么都看不到；--quiet 供自动测试用
    if show_dialog and getattr(sys, "frozen", False):
        try:
            from tkinter import messagebox
            messagebox.showinfo("自检结果", text + f"\n\n报告：\n{out}")
        except Exception:
            pass
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest(show_dialog="--quiet" not in sys.argv)
    try:
        app = PollLiteMDI()
    except Exception as e:
        # 打包成 --windowed 的 exe 后没有控制台，报错必须弹窗，
        # 否则双击一闪就没、什么信息都没有，根本没法查
        import traceback
        msg = f"{type(e).__name__}: {e}\n\n{traceback.format_exc()}"
        print("启动失败:", msg)
        try:
            from tkinter import messagebox
            messagebox.showerror("Modbus Poll Lite 启动失败", msg)
        except Exception:
            pass
        return 1
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
