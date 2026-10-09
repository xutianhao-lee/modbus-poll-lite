# -*- coding: utf-8 -*-
"""
modbus_chart.py —— 实时曲线窗口（Canvas 自绘，无第三方依赖）

给 Modbus Poll Lite 用：把当前数据区的数值随时间滚动绘出来，可多路叠加。
用途：看趋势、调 PID、抓振荡、观察超调。

设计要点：
    · 直接挂在 DataArea 上，靠它的 sample_no 判断"有没有新数据"，
      所以采样点与真实轮询严格对齐，不会重复记同一个值
    · 每路保留上限 MAX_POINTS 个点，长时间运行不会吃爆内存
    · 显示点数过多时自动抽稀，重绘保持流畅
    · 数据区改了功能码/地址/数量会自动清空重开（量纲变了，混着画没意义）
"""

import collections
import time
import tkinter as tk
from tkinter import ttk

# 游标说明：
#   游标**固定在屏幕上**（存的是屏幕比例 0~1，不是时间戳）。
#   放好之后它不会随波形滚动而漂走；读数实时反映"游标当前位置下方的波形值"。
#   因此 ΔT = 时间窗口 × 两游标的屏距比例，是个定值。
#   （若做成锚定时间戳，一滚动游标就跑了，实测体验很差。）

COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
          "#008080", "#9a6324", "#800000", "#808000", "#000075",
          "#469990", "#bcbd22", "#7f7f7f", "#d4a017", "#5a5a5a"]
MAX_POINTS = 20000          # 每路最多保留的采样点
MAX_DRAW = 1500             # 单路单次最多画的点数（超出则抽稀）


def fmt_num(v):
    """数值格式化：大数加千分位，小数保留合适位数"""
    if v is None:
        return "-"
    if abs(v) >= 100000 or (v != 0 and abs(v) < 0.01):
        return f"{v:.3g}"
    if abs(v - round(v)) < 1e-9:
        return f"{int(round(v)):,}"
    return f"{v:,.3f}".rstrip("0").rstrip(".")


class ChartWindow(tk.Toplevel):
    """一个数据区对应一个曲线窗口"""

    WINDOWS = [("10 秒", 10), ("30 秒", 30), ("1 分钟", 60),
               ("2 分钟", 120), ("5 分钟", 300), ("15 分钟", 900)]

    def __init__(self, area):
        super().__init__(area)
        self.area = area
        self.title(f"实时曲线 —— 数据区 {area.index}")
        self.geometry("940x640")
        self.minsize(660, 460)

        self.history = {}            # addr -> deque[(t, value)]
        self.color_map = {}          # addr -> color
        self.hidden = set()          # 被隐藏的地址
        self.last_no = -1
        self.last_sig = None
        self.paused = False
        self.freeze_at = None        # 暂停时冻结的时刻，暂停期间视图不滚动
        self.window_sec = 60
        self.auto_y = True
        self.y_min, self.y_max = 0.0, 100.0
        self._legend_rows = {}

        # 游标（示波器式测量）
        # ⚠ 存的是**屏幕比例 0~1**（不是时间戳）—— 游标固定在屏幕上不随波形滚动，
        #   读数是"游标当前位置下方的波形值"。若锚定时间戳，一滚动游标就漂走了。
        self.cursor_on = False
        self.c1 = None               # 游标 1 的屏幕比例（0=最左，1=最右）
        self.c2 = None
        self.dragging = 0
        self.plot_geom = (68, 14, 100, 100)   # L, T, pw, ph（每次重绘更新）

        self._build()
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self._tick()

    # ---------------------------------------------------------------- 界面
    def _build(self):
        bar = ttk.Frame(self, padding=(8, 6))
        bar.pack(fill="x")

        ttk.Label(bar, text="时间窗口").pack(side="left")
        self.v_win = tk.StringVar(value="1 分钟")
        cb = ttk.Combobox(bar, textvariable=self.v_win, state="readonly", width=9,
                          values=[t for t, _ in self.WINDOWS])
        cb.pack(side="left", padx=(4, 12))
        cb.bind("<<ComboboxSelected>>", lambda e: self._set_window())

        ttk.Label(bar, text="采样周期").pack(side="left")
        self.v_rate = tk.StringVar(value=str(self.area.scan_rate))
        cb_r = ttk.Combobox(bar, textvariable=self.v_rate, state="readonly", width=7,
                            values=["50", "100", "200", "500", "1000", "2000"])
        cb_r.pack(side="left", padx=(4, 12))
        cb_r.bind("<<ComboboxSelected>>", lambda e: self._set_rate())

        self.v_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Y 轴自动量程", variable=self.v_auto,
                        command=self._set_auto).pack(side="left")

        self.v_ymin = tk.StringVar(value="0")
        self.v_ymax = tk.StringVar(value="100")
        self.e_min = ttk.Entry(bar, textvariable=self.v_ymin, width=8)
        self.e_max = ttk.Entry(bar, textvariable=self.v_ymax, width=8)
        ttk.Label(bar, text="固定范围").pack(side="left", padx=(12, 2))
        self.e_min.pack(side="left")
        ttk.Label(bar, text="~").pack(side="left", padx=2)
        self.e_max.pack(side="left")
        self.e_min.configure(state="disabled")
        self.e_max.configure(state="disabled")

        self.btn_pause = ttk.Button(bar, text="暂停", command=self._toggle_pause)
        self.btn_pause.pack(side="left", padx=(12, 4))
        ttk.Button(bar, text="清空", command=self._clear).pack(side="left")
        ttk.Button(bar, text="全选", width=6,
                   command=lambda: self._show_all(True)).pack(side="left", padx=(10, 2))
        ttk.Button(bar, text="全不选", width=7,
                   command=lambda: self._show_all(False)).pack(side="left")

        self.btn_cursor = ttk.Button(bar, text="游标：关", width=9,
                                     command=self._toggle_cursor)
        self.btn_cursor.pack(side="left", padx=(10, 0))

        self.lbl_state = ttk.Label(bar, text="", foreground="#666")
        self.lbl_state.pack(side="right")

        self.canvas = tk.Canvas(self, bg="white", highlightthickness=1,
                                highlightbackground="#ccc", cursor="crosshair")
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self.canvas.bind("<Button-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)

        # 游标读数区 —— 独立成块，不覆盖画布（否则会挡住波形）
        ro = ttk.Frame(self, padding=(8, 2, 8, 0))
        ro.pack(fill="x")
        ttk.Label(ro, text="游标读数", foreground="#666").pack(anchor="w")
        self.readout = tk.Text(ro, height=6, font=("Consolas", 9), wrap="none",
                               bg="#fcfcf4", relief="solid", borderwidth=1,
                               highlightthickness=0)
        self.readout.pack(fill="x")
        self.readout.configure(state="disabled")
        self._readout_text = None

        self.legend = ttk.Frame(self, padding=(8, 2, 8, 8))
        self.legend.pack(fill="x")

    # ---------------------------------------------------------------- 控制
    def _set_window(self):
        for t, sec in self.WINDOWS:
            if t == self.v_win.get():
                self.window_sec = sec
                break
        self._prune()

    def _set_rate(self):
        """直接改数据区的轮询周期 —— 采样密度 = 1/周期，想波形顺就调小"""
        try:
            self.area.scan_rate = max(50, int(self.v_rate.get()))
        except ValueError:
            return
        self.area._update_status()

    def _set_auto(self):
        on = self.v_auto.get()
        st = "disabled" if on else "normal"
        self.e_min.configure(state=st)
        self.e_max.configure(state=st)
        if not on:
            try:
                self.y_min = float(self.v_ymin.get())
                self.y_max = float(self.v_ymax.get())
                if self.y_max <= self.y_min:
                    self.y_max = self.y_min + 1
            except ValueError:
                self.y_min, self.y_max = 0.0, 100.0

    def _view_now(self):
        """视图的"当前时刻"。暂停时冻结 —— 否则画面会一直往左滚，游标没法量"""
        if self.paused and self.freeze_at is not None:
            return self.freeze_at
        return time.time()

    def _toggle_pause(self):
        self.paused = not self.paused
        self.freeze_at = time.time() if self.paused else None
        self.btn_pause.configure(text="继续" if self.paused else "暂停")

    # ---------------------------------------------------------------- 游标
    def _toggle_cursor(self):
        self.cursor_on = not self.cursor_on
        if self.cursor_on and self.c1 is None:
            self.c1, self.c2 = 0.30, 0.70     # 默认落在屏幕 30% / 70% 处
        self.btn_cursor.configure(text="游标：开" if self.cursor_on else "游标：关")

    def _clear_cursor(self):
        self.c1 = self.c2 = None
        self.cursor_on = False
        self.btn_cursor.configure(text="游标：关")

    # -- 屏幕比例 <-> 像素 <-> 时刻 --------------------------------------
    def _frac_to_px(self, f):
        L, T, pw, ph = self.plot_geom
        return L + pw * f

    def _px_to_frac(self, x):
        L, T, pw, ph = self.plot_geom
        if not pw:
            return 0.0
        return min(1.0, max(0.0, (x - L) / float(pw)))

    def _frac_time(self, f):
        """屏幕上某个比例位置，当前对应的时刻"""
        return self._view_now() - self.window_sec * (1.0 - f)

    def cursor_dt(self):
        """两游标的时间差（屏幕固定，所以等于窗口时长 × 屏距比例）"""
        if self.c1 is None or self.c2 is None:
            return 0.0
        return self.window_sec * abs(self.c2 - self.c1)

    def cursor_x(self):
        """两游标的 X 轴坐标（秒，相对"现在"），与轴刻度同一套口径"""
        if self.c1 is None or self.c2 is None:
            return None, None
        now = self._view_now()
        return self._frac_time(self.c1) - now, self._frac_time(self.c2) - now

    def _on_press(self, event):
        if not self.cursor_on or self.c1 is None:
            return
        x1, x2 = self._frac_to_px(self.c1), self._frac_to_px(self.c2)
        self.dragging = 1 if abs(event.x - x1) <= abs(event.x - x2) else 2

    def _on_drag(self, event):
        if not self.cursor_on or not self.dragging:
            return
        f = self._px_to_frac(event.x)
        if self.dragging == 1:
            self.c1 = f
        else:
            self.c2 = f

    def _on_release(self, _event):
        self.dragging = 0

    def _value_at(self, vis, t):
        """返回 (值, 是否落在数据时间范围内)。

        ⚠ 游标落在数据范围外时必须区分出来 —— 否则会拿边缘点当读数，
        看起来像"这段时间没变化"，是误导。
        """
        if not vis:
            return None, False
        inside = vis[0][0] - 1e-6 <= t <= vis[-1][0] + 1e-6
        best, bestd = vis[0][1], abs(vis[0][0] - t)
        for ts, v in vis:
            d = abs(ts - t)
            if d < bestd:
                best, bestd = v, d
        return best, inside

    def _clear(self):
        self.history.clear()
        self.color_map.clear()
        self.hidden.clear()
        self.last_no = -1

    def _prune(self):
        t0 = time.time() - self.window_sec
        for dq in self.history.values():
            while dq and dq[0][0] < t0:
                dq.popleft()

    def color_of(self, addr):
        if addr not in self.color_map:
            self.color_map[addr] = COLORS[len(self.color_map) % len(COLORS)]
        return self.color_map[addr]

    # ---------------------------------------------------------------- 采样
    def _tick(self):
        if not self.winfo_exists():
            return
        area = self.area
        if not area.winfo_exists():
            self.destroy()
            return

        # 数据区那边改了扫描周期（比如从 F8 改的），这里跟着同步
        if self.v_rate.get() != str(area.scan_rate):
            self.v_rate.set(str(area.scan_rate))

        sig = (area.fc, area.addr, area.qty)
        if self.last_sig is None:
            self.last_sig = sig
        elif sig != self.last_sig:
            self.last_sig = sig
            self._clear()          # 量纲变了，混着画没意义

        if not self.paused:
            with area.data_lock:
                no = getattr(area, "sample_no", 0)
                vals = list(area.values)
                err = area.err_state
            if no != self.last_no and vals:
                self.last_no = no
                t = time.time()
                step = area._row_step()
                n = len(vals)
                for row in range(max(1, area.qty // step)):
                    i = row * step
                    if i >= n:
                        break
                    v = area._plot_value(vals, i)
                    if v is None:
                        continue
                    addr = area.addr + i
                    dq = self.history.get(addr)
                    if dq is None:
                        dq = self.history[addr] = collections.deque(maxlen=MAX_POINTS)
                    dq.append((t, v))
                self._prune()

        self._redraw()
        self._update_legend()
        self._update_state()
        # 80ms ≈ 12fps：再快人眼收益不大，再慢滚动就会一跳一跳
        self.after(80, self._tick)

    def _update_state(self):
        area = self.area
        if self.paused:
            txt, color = "● 已暂停（仍可缩放/查看）", "#c60"
        elif area.fc in (5, 6, 15, 16):
            txt, color = "写模式，本窗口无数据（写模式不轮询）", "#c00"
        elif not area.mb.connected:
            txt, color = "未连接", "#c00"
        elif not area.enabled:
            txt, color = "未启用轮询（F8 勾 Read/Write Enabled）", "#c60"
        else:
            n = sum(len(d) for d in self.history.values())
            rate = 1000.0 / max(1, area.scan_rate)
            in_win = rate * self.window_sec
            txt = f"采样 {n} 点 / {len(self.history)} 路 · {rate:.1f} 点/秒"
            if in_win < 40:
                txt += f"  ⚠ 窗口内仅约 {in_win:.0f} 点，调小「采样周期」更顺"
                color = "#c60"
            else:
                color = "#060"
        self.lbl_state.configure(text=txt, foreground=color)

    # ---------------------------------------------------------------- 绘图
    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 80 or h < 80:
            return
        L, R, T, B = 68, 18, 14, 28
        pw, ph = w - L - R, h - T - B
        if pw <= 20 or ph <= 20:
            return

        self.plot_geom = (L, T, pw, ph)
        now = self._view_now()
        span = float(self.window_sec)
        t0 = now - span

        series = []
        for addr, dq in self.history.items():
            if addr in self.hidden:
                continue
            vis = [(t, v) for t, v in dq if t >= t0]
            if vis:
                series.append((addr, vis))

        allvals = [v for _, vis in series for _, v in vis]
        if self.v_auto.get() or not allvals:
            if allvals:
                lo, hi = min(allvals), max(allvals)
            else:
                lo, hi = 0.0, 1.0
            if hi - lo < 1e-9:
                hi = lo + 1.0
            pad = (hi - lo) * 0.08
            lo, hi = lo - pad, hi + pad
        else:
            lo, hi = self.y_min, self.y_max
            if hi <= lo:
                hi = lo + 1.0

        # --- 网格与刻度 ---
        for k in range(5):
            y = T + ph * k / 4.0
            c.create_line(L, y, L + pw, y, fill="#ececec")
            c.create_text(L - 6, y, text=fmt_num(hi - (hi - lo) * k / 4.0),
                          anchor="e", font=("Consolas", 8), fill="#777")
        for k in range(5):
            x = L + pw * k / 4.0
            c.create_line(x, T, x, T + ph, fill="#f4f4f4")
            ago = span * (1 - k / 4.0)
            c.create_text(x, T + ph + 13,
                          text="现在" if ago <= 0.01 else f"-{int(ago)}s",
                          font=("Consolas", 8), fill="#777")

        c.create_rectangle(L, T, L + pw, T + ph, outline="#bbb")

        if not series:
            c.create_text(L + pw / 2, T + ph / 2, text="等待数据…",
                          fill="#999", font=("", 11))
            self._draw_cursors(c, L, T, pw, ph)
            self._update_readout([])
            return

        # --- 折线 ---
        # 抽稀上限按**画布宽度**算：每 2 像素一个点就足够，多了纯属浪费 CPU。
        # （用固定值的话，窗口拉大后重绘会变慢，滚动就卡）
        denom = (hi - lo) or 1.0
        max_pts = max(64, int(pw // 2))
        for addr, vis in series:
            color = self.color_of(addr)
            stride = max(1, len(vis) // max_pts)
            pts = []
            for idx in range(0, len(vis), stride):
                t, v = vis[idx]
                x = L + pw * (1.0 - (now - t) / span)
                y = T + ph * (1.0 - (v - lo) / denom)
                pts.extend((x, y))
            # 最后一个点永远画上，避免末值被抽掉
            t, v = vis[-1]
            x_end = L + pw * (1.0 - (now - t) / span)
            y_end = T + ph * (1.0 - (v - lo) / denom)
            pts.extend((x_end, y_end))

            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=1.5)

            # 采样点稀疏时把真实采样位置标出来 —— 免得直线段被当成"数据"
            if len(vis) <= 120 and stride == 1:
                r = 1.6
                for idx in range(0, len(vis)):
                    t, v = vis[idx]
                    x = L + pw * (1.0 - (now - t) / span)
                    y = T + ph * (1.0 - (v - lo) / denom)
                    c.create_oval(x - r, y - r, x + r, y + r,
                                  fill=color, outline="")

            # 末值小圆点
            c.create_oval(x_end - 2.5, y_end - 2.5,
                          x_end + 2.5, y_end + 2.5, fill=color, outline="")

        self._draw_cursors(c, L, T, pw, ph)
        self._update_readout(series)

    def _draw_cursors(self, c, L, T, pw, ph):
        """只在画布上画两条游标线。

        读数**不画在画布上** —— 之前叠在左上角会挡住波形，现在写到下方独立读数区。
        """
        if not (self.cursor_on and self.c1 is not None):
            return
        for f, name, col in ((self.c1, "T1", "#d40000"), (self.c2, "T2", "#0044cc")):
            x = self._frac_to_px(f)
            if L - 1 <= x <= L + pw + 1:
                c.create_line(x, T, x, T + ph, fill=col, width=1.2, dash=(4, 3))
                c.create_text(x, T + 9, text=name, fill=col,
                              font=("Consolas", 8, "bold"))

    def _update_readout(self, series):
        """游标读数写到画布下方的独立读数区"""
        if not (self.cursor_on and self.c1 is not None):
            text = "游标未开启 —— 点工具栏的「游标：关」按钮打开。"
        else:
            t1, t2 = self._frac_time(self.c1), self._frac_time(self.c2)
            now = self._view_now()
            # 游标位置用**X 轴坐标**表示（与轴上 -10s / -5s / 现在 同一套刻度），
            # 不用屏幕百分比 —— 百分比还得自己换算成时间，没用
            def _abs(ts):
                return (time.strftime("%H:%M:%S", time.localtime(ts))
                        + f".{int((ts % 1) * 1000):03d}")

            rows = [f"游标测量    ΔT = {self.cursor_dt():.3f} s"
                    f"     T1 = {t1 - now:.3f} s     T2 = {t2 - now:.3f} s"
                    f"     （X 轴坐标，与刻度同口径）",
                    f"绝对时刻    T1 = {_abs(t1)}     T2 = {_abs(t2)}"
                    f"     （对 CSV 日志用）"]
            shown = 0
            out_of_range = False
            for addr, vis in sorted(series, key=lambda s: s[0]):
                v1, in1 = self._value_at(vis, t1)
                v2, in2 = self._value_at(vis, t2)
                if v1 is None or v2 is None:
                    continue
                if shown >= 20:
                    rows.append(f"…还有 {len(series) - shown} 路")
                    break
                if not (in1 and in2):
                    out_of_range = True
                s1 = fmt_num(v1) if in1 else "—"
                s2 = fmt_num(v2) if in2 else "—"
                sd = fmt_num(v2 - v1) if (in1 and in2) else "—"
                name = self.area.alias.get(addr) or str(addr)
                rows.append(f"  {name:>9}   {s1:>10} → {s2:>10}    Δ {sd:>10}")
                shown += 1
            if shown == 0:
                rows.append("  （游标范围内没有数据 —— 把游标拖进波形里）")
            elif out_of_range:
                rows.append("  — = 游标落在数据范围之外（把游标拖进波形里）")
            text = "\n".join(rows)

        if text != self._readout_text:
            self._readout_text = text
            self.readout.configure(state="normal")
            self.readout.delete("1.0", "end")
            self.readout.insert("1.0", text)
            self.readout.configure(state="disabled")

    def _update_legend(self):
        area = self.area
        addrs = sorted(self.history.keys())
        if set(addrs) != set(self._legend_rows.keys()):
            for w in self.legend.winfo_children():
                w.destroy()
            self._legend_rows.clear()
            for addr in addrs:
                f = ttk.Frame(self.legend)
                f.pack(side="left", padx=(0, 14))

                toggle = tk.BooleanVar(value=addr not in self.hidden)
                cb = ttk.Checkbutton(
                    f, text="", variable=toggle,
                    command=lambda a=addr, v=toggle: self._toggle_series(a, v))
                cb.pack(side="left")

                sw = tk.Canvas(f, width=12, height=12, highlightthickness=0)
                sw.create_rectangle(2, 2, 11, 11, fill=self.color_of(addr), outline="")
                sw.pack(side="left", padx=(0, 3))

                name = area.alias.get(addr) or str(addr)
                lbl = ttk.Label(f, text=name, foreground="#333")
                lbl.pack(side="left")

                val = ttk.Label(f, text="", font=("Consolas", 9), foreground="#000")
                val.pack(side="left", padx=(4, 0))

                for w in (f, sw, lbl, val):      # 双击图例 = 只看这一路
                    w.bind("<Double-Button-1>", lambda e, a=addr: self._solo(a))

                self._legend_rows[addr] = val
        else:
            for addr, lbl in self._legend_rows.items():
                dq = self.history.get(addr)
                lbl.configure(text=fmt_num(dq[-1][1]) if dq else "-")

    def _toggle_series(self, addr, var):
        if var.get():
            self.hidden.discard(addr)
        else:
            self.hidden.add(addr)

    def _show_all(self, on):
        self.hidden.clear() if on else self.hidden.update(self.history.keys())
        self._rebuild_legend()

    def _solo(self, addr):
        """双击图例：只看这一路；再双击恢复全部"""
        others = set(self.history.keys()) - {addr}
        if self.hidden == others and addr not in self.hidden:
            self.hidden.clear()                  # 已经在 solo，恢复
        else:
            self.hidden = others
        self._rebuild_legend()

    def _rebuild_legend(self):
        for w in self.legend.winfo_children():
            w.destroy()
        self._legend_rows.clear()
        self._update_legend()
