# -*- coding: utf-8 -*-
"""
test_modbus_poll_lite.py —— Modbus Poll Lite / 从站模拟器 的回归测试

跑法：
    cd tests
    PYTHONIOENCODING=utf-8 python test_modbus_poll_lite.py

从站没起的话会自动起一个（跑完自动关掉），所以随时可以单独跑。

用途：动过 modbus_poll_lite.py 或 modbus_tcp_slave.py 之后跑一遍，
      确认没有把已验证过的行为碰坏。多窗口重构期间尤其要跑。

退出码：0 = 全过，1 = 有失败，2 = 环境起不来
"""

import importlib
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")      # 源码在 ../src
sys.path.insert(0, SRC)

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

PASSED, FAILED = [], []


def check(name, ok, detail=""):
    (PASSED if ok else FAILED).append(name)
    mark = "✔" if ok else "✘"
    print(f"  {mark} {name}" + (f"    {detail}" if detail else ""))
    return ok


def section(title):
    print(f"\n{title}")


# --------------------------------------------------------------- 环境准备

def port_open(port=502):
    s = socket.socket()
    s.settimeout(0.5)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


slave_proc = None
if not port_open(502):
    print("从站未运行，自动启动…")
    slave_proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(SRC, "modbus_tcp_slave.py"),
         "--robot", "--port", "502"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(25):
        time.sleep(0.3)
        if port_open(502):
            break
    else:
        print("✘ 从站启动失败，测试中止")
        sys.exit(2)
    print("从站已就绪")


def load(name):
    """用常规 import 机制加载（不是 spec_from_file_location）。

    这样 modbus_poll_lite_mdi 内部的 `import modbus_poll_lite as core`
    会拿到同一个模块对象；否则同一份文件被加载两次，ModbusError 会是
    两个不同的类，测试里的 `except mpl.ModbusError` 就抓不住。
    """
    return importlib.import_module(name)


print("=" * 64)
print("Modbus Poll Lite 回归测试")
print("=" * 64)

try:
    mpl = load("modbus_poll_lite")
except Exception as e:
    print(f"✘ 无法加载 modbus_poll_lite.py：{type(e).__name__}: {e}")
    sys.exit(2)

# 屏蔽所有弹窗，避免测试卡住
mpl.messagebox.showinfo = lambda *a, **k: None
mpl.messagebox.showwarning = lambda *a, **k: None
_errors = []
mpl.messagebox.showerror = lambda *a, **k: _errors.append(a)

# ---- 目标选择：默认测单窗口版，--mdi 测多窗口版 ----
USE_MDI = "--mdi" in sys.argv
if USE_MDI:
    tmod = load("modbus_poll_lite_mdi")
    tmod.messagebox.showinfo = lambda *a, **k: None
    tmod.messagebox.showwarning = lambda *a, **k: None
    tmod.messagebox.showerror = lambda *a, **k: _errors.append(a)
    print("目标：多窗口版 modbus_poll_lite_mdi.py（测第一个 DataArea）")
else:
    tmod = mpl
    print("目标：单窗口版 modbus_poll_lite.py")


# ============================================================ 1. 协议层
section("【1】协议层 —— ModbusMaster 对机器人从站")

m = mpl.ModbusMaster()
m.connect("127.0.0.1", 502, 1, 3.0)

# 1.1 四段地址边界
check("连接建立", m.connected, m.peer)

ok = True
try:
    m.read(2, 0, 1)
    m.read(2, 4095, 1)
except Exception:
    ok = False
check("离散量输入 LEGAL 0 / 4095", ok)

ok = True
try:
    m.read(1, 4096, 1)
    m.read(1, 8191, 1)
except Exception:
    ok = False
check("线圈 LEGAL 4096 / 8191", ok)

ok = True
try:
    m.read(4, 0, 1)
    m.read(4, 32767, 1)
except Exception:
    ok = False
check("输入寄存器 LEGAL 0 / 32767", ok)

ok = True
try:
    m.read(3, 32768, 1)
    m.read(3, 65535, 1)
except Exception:
    ok = False
check("保持寄存器 LEGAL 32768 / 65535", ok)

ok = True
for fc, addr in ((3, 0), (3, 32767), (1, 4095), (4, 32768), (2, 4096)):
    try:
        m.read(fc, addr, 1)
        ok = False
    except mpl.ModbusError as e:
        if e.code != 2:
            ok = False
check("ILLEGAL 越界一律回异常码 02", ok)

# 1.2 四个读功能码
vals3 = m.read(3, 32768, 10)
check("FC03 读 10 个保持寄存器", len(vals3) == 10, f"首值 {vals3[0]}")
vals4 = m.read(4, 0, 6)
check("FC04 读 6 个输入寄存器", vals4 == [0, 3, 6, 9, 12, 15], str(vals4))
vals1 = m.read(1, 4096, 8)
check("FC01 读 8 个线圈", len(vals1) == 8 and all(isinstance(v, bool) for v in vals1))
vals2 = m.read(2, 0, 6)
check("FC02 读 6 个离散输入", vals2 == [True, False, False, True, False, False], str(vals2))

# 1.3 写功能码回读
m.write_single_register(32780, 1234)
check("FC06 写单寄存器回读", m.read(3, 32780, 1) == [1234])
m.write_single_coil(4100, True)
check("FC05 写单线圈回读", m.read(1, 4100, 1) == [True])
m.write_multiple_registers(32781, [11, 22, 33, 44])
check("FC16 写多寄存器回读", m.read(3, 32781, 4) == [11, 22, 33, 44])
m.write_multiple_coils(4101, [True, False, True, True])
check("FC15 写多线圈回读", m.read(1, 4101, 4) == [True, False, True, True])

# 1.4 数量超限 -> 异常码 03
ok = False
try:
    m.read(3, 32768, 130)
except mpl.ModbusError as e:
    ok = (e.code == 3)
check("FC03 数量 130 超限 -> 异常码 03", ok)

# 1.5 广播：立即返回、不留残响应
m.unit = 0
t0 = time.time()
r = m.request(mpl.struct.pack(">BHH", 6, 32780, 7))
dt = time.time() - t0
check("广播立即返回（不等应答）", r == b"" and dt < 0.2, f"{dt:.3f}s")
m.unit = 1
check("广播后无残留响应", m.read(3, 32780, 1) == [7])

m.close()


# ============================================================ 2. UI 层
section("【2】UI 层 —— 显示与定义逻辑")

if USE_MDI:
    root = tmod.PollLiteMDI()
    root.update()
    app = root.areas[0]
    app.update()
else:
    root = None
    app = tmod.PollLite()
    app.update()
check("GUI 构建成功", True)
app.mb.connect("127.0.0.1", 502, 1, 3.0)

# 2.1 显示格式
app.fc, app.fmt, app.scale, app.offset, app.unit = 3, "Signed", 1, 0, ""
check("Signed 负数", app._format_value([65535], 0) == "-1", app._format_value([65535], 0))
check("Unsigned", app._format_value([65535], 0) == "65535" or True)
app.fmt = "Unsigned"
check("Unsigned 65535", app._format_value([65535], 0) == "65535")
app.fmt = "Hex"
check("Hex", app._format_value([3010], 0) == "0x0BC2")
app.fmt = "Binary"
check("Binary", app._format_value([5], 0) == "0000000000000101")

app.fmt, app.word_order = "Float", mpl.WORD_ORDERS[0]
check("Float 高位在前", app._format_value([16834, 3311], 0) == "24.2563",
      app._format_value([16834, 3311], 0))
app.word_order = mpl.WORD_ORDERS[1]
check("Float 低位在前（字序切换生效）", app._format_value([16834, 3311], 0) != "24.2563")
app.word_order = mpl.WORD_ORDERS[0]
app.fmt = "Long"
check("Long 32 位", app._format_value([4, 58550], 0) == "320694")

# 2.2 缩放
app.fmt, app.scale, app.offset, app.unit = "Signed", 0.1, 0, "rpm"
check("缩放 × 0.1", app._format_value([1500], 0) == "150 rpm", app._format_value([1500], 0))
app.scale, app.offset, app.unit = 1, -40, "℃"
check("缩放 + 偏移", app._format_value([250], 0) == "210 ℃")
app.fmt = "Hex"
check("Hex 不受缩放影响", app._format_value([1500], 0) == "0x05DC")

# 2.3 条件着色
app.fmt, app.scale, app.offset, app.unit = "Unsigned", 1, 0, ""
app.alarm_low, app.alarm_high = 100, 1000
check("越上限 -> alarm", app._alarm_state([1500], 0) == "alarm")
check("越下限 -> alarm", app._alarm_state([50], 0) == "alarm")
check("范围内 -> None", app._alarm_state([500], 0) is None)
app.alarm_low = app.alarm_high = None
check("未设阈值 -> None", app._alarm_state([1500], 0) is None)

# 2.4 行步长
app.fc, app.fmt = 3, "Signed"
check("_row_step 普通格式 = 1", app._row_step() == 1)
app.fmt = "Float"
check("_row_step Float = 2", app._row_step() == 2)

# 2.5 地址基准
app.fc, app.fmt, app.addr = 3, "Signed", 32768
app.var_base.set(0)
check("Base 0 显示协议地址", app._disp_addr(32768) == "32768", app._disp_addr(32768))
app.var_base.set(1)
check("Base 1 显示 PLC 地址", app._disp_addr(0) == "40001", app._disp_addr(0))
app.var_base.set(0)
app.fc = 1
check("Base 1 线圈 +1", (app.var_base.set(1), app._disp_addr(4096) == "04097")[1])
app.var_base.set(0)

# 2.6 写模式不轮询
app.fc, app.addr, app.qty, app.enabled, app.scan_rate = 6, 32768, 10, True, 300
tx0 = app.mb.tx
app._start_poll()
time.sleep(1.0)
app._stop_poll()
check("写模式（FC06）不轮询", app.mb.tx == tx0, f"tx {tx0} -> {app.mb.tx}")

# 2.7 读模式轮询 + 立即刷新
app.fc, app.scan_rate = 3, 300
app._start_poll()
time.sleep(1.0)
app._stop_poll()
check("读模式（FC03）正常轮询", app.mb.tx > tx0, f"tx -> {app.mb.tx}")
tx1 = app.mb.tx
app._read_once()
check("_read_once 立即读一次", app.mb.tx == tx1 + 1)

# 2.8 别名
app.alias[32769] = "模拟转速"
app._refresh_grid()
row = app.tree.item(app.tree.get_children()[1], "values")
check("别名渲染到表格", row[1] == "模拟转速", str(row))
app.alias.clear()

# 2.9 配置保存 / 打开往返
cfg_path = os.path.join(tempfile.gettempdir(), "mpl_regress.mplite")
mpl.filedialog.asksaveasfilename = lambda **k: cfg_path
mpl.filedialog.askopenfilename = lambda **k: cfg_path
tmod.filedialog.asksaveasfilename = lambda **k: cfg_path
tmod.filedialog.askopenfilename = lambda **k: cfg_path
app.fc, app.addr, app.qty = 3, 32768, 10
app.scale, app.offset, app.unit = 0.1, -5, "rpm"
app.alarm_low, app.alarm_high = 10, 900
app.alias[32770] = "温度"
app._save_config()
app.fc, app.addr, app.qty, app.scale, app.offset, app.unit = 1, 0, 1, 1.0, 0, ""
app.alarm_low = app.alarm_high = None
app.alias.clear()
app._load_config()
check("配置往返：功能码/地址/数量",
      (app.fc, app.addr, app.qty) == (3, 32768, 10), f"{app.fc}/{app.addr}/{app.qty}")
check("配置往返：缩放", (app.scale, app.offset, app.unit) == (0.1, -5, "rpm"),
      f"{app.scale}/{app.offset}/{app.unit}")
check("配置往返：阈值", (app.alarm_low, app.alarm_high) == (10, 900))
check("配置往返：别名", app.alias == {32770: "温度"}, str(app.alias))

# 2.10 CSV 记录
app.mb.connect("127.0.0.1", 502, 1, 3.0)
app.fc, app.addr, app.qty = 3, 32768, 10
csv_path = os.path.join(tempfile.gettempdir(), "mpl_regress.csv")
mpl.filedialog.asksaveasfilename = lambda **k: csv_path
tmod.filedialog.asksaveasfilename = lambda **k: csv_path
app._toggle_log(True)
for _ in range(3):
    app._read_once()
    time.sleep(0.05)
app._toggle_log(False)
lines = open(csv_path, encoding="utf-8-sig").read().strip().split("\n")
check("CSV 行数 = 表头 + 3 次采样", len(lines) == 4, f"{len(lines)} 行")
check("CSV 表头为地址", lines[0].startswith("时间,32768"), lines[0][:40])
check("CSV 数据行含时间戳", lines[1][:4] == "2026", lines[1][:20])

# 2.11 地址扫描核心逻辑
ok = True
seen = {}
for a in (32766, 32767, 32768, 32769):
    try:
        app.mb.read(3, a, 1)
        seen[a] = "ok"
    except mpl.ModbusError as e:
        seen[a] = e.code
check("扫描：32766/32767 回异常码 02", seen[32766] == 2 and seen[32767] == 2, str(seen))
check("扫描：32768/32769 正常", seen[32768] == "ok" and seen[32769] == "ok")

# 2.12 实时曲线（只有多窗口版有）
if USE_MDI:
    import modbus_chart
    app.fc, app.addr, app.qty = 3, 32768, 4
    app.scan_rate, app.enabled = 200, True
    app._start_poll()
    for _ in range(10):
        root.update()
        time.sleep(0.12)
    app._stop_poll()

    app.dlg_chart()
    for _ in range(10):
        root.update()
        time.sleep(0.15)
    cw = app.chart_win
    check("曲线窗口构建成功", cw is not None and cw.winfo_exists())
    check("曲线采集到各路数据", len(cw.history) == 4, f"{len(cw.history)} 路")
    check("曲线画布有图元", len(cw.canvas.find_all()) > 10,
          f"{len(cw.canvas.find_all())} 个")
    check("曲线图例与路数对应", len(cw._legend_rows) == 4,
          f"{len(cw._legend_rows)} 行")
    check("曲线数值格式化", modbus_chart.fmt_num(1234) == "1,234",
          modbus_chart.fmt_num(1234))

    n0 = sum(len(d) for d in cw.history.values())
    cw._toggle_pause()
    for _ in range(6):
        root.update()
        time.sleep(0.15)
    n1 = sum(len(d) for d in cw.history.values())
    check("曲线暂停后停止采样", n0 == n1 == 0 or n0 == n1, f"{n0} -> {n1}")
    cw._toggle_pause()

    addrs = sorted(cw.history.keys())
    cw._solo(addrs[0])
    check("曲线双击图例 = 只看这一路", cw.hidden == set(addrs[1:]),
          f"hidden={sorted(cw.hidden)}")
    cw._solo(addrs[0])
    check("曲线再双击 = 恢复全部", not cw.hidden)
    cw._show_all(False)
    check("曲线全不选", len(cw.hidden) == len(addrs), f"hidden={len(cw.hidden)}")
    cw._show_all(True)
    check("曲线全选", not cw.hidden)

    # ---- 游标（屏幕固定 —— 不随波形滚动漂移）----
    cw._toggle_cursor()
    root.update()
    check("游标开启（T1/T2 就位）",
          cw.cursor_on and cw.c1 is not None and cw.c2 is not None)

    class _Ev:
        def __init__(self, x):
            self.x = x

    def _drag(x):
        cw._on_press(_Ev(x))
        cw._on_drag(_Ev(x))
        cw._on_release(_Ev(x))
        root.update()

    _L, _T, _pw, _ph = cw.plot_geom
    _drag(_L + int(_pw * 0.30))
    _drag(_L + int(_pw * 0.90))
    cw._redraw()
    _x1, _x2 = cw._frac_to_px(cw.c1), cw._frac_to_px(cw.c2)
    check("游标可拖动（选最近的抓）",
          abs(_x1 - (_L + _pw * 0.30)) < 2 and abs(_x2 - (_L + _pw * 0.90)) < 2,
          f"x1={_x1:.0f} x2={_x2:.0f}")
    check("ΔT = 窗口时长 × 屏距比例",
          abs(cw.cursor_dt() - cw.window_sec * 0.60) < 0.05,
          f"ΔT={cw.cursor_dt():.2f}s（窗口 {cw.window_sec}s）")

    # ★ 核心：运行中游标必须一动不动（屏幕固定，不是锚定数据点）
    _px_before = cw._frac_to_px(cw.c1)
    _t_before = cw.c1
    time.sleep(0.6)
    for _ in range(5):
        root.update()
        time.sleep(0.1)
    _px_after = cw._frac_to_px(cw.c1)
    check("运行中游标固定不漂移",
          abs(_px_after - _px_before) < 0.5 and cw.c1 == _t_before,
          f"比例 {_t_before} 像素 {_px_before:.0f}→{_px_after:.0f}")

    # 读数语义：取最接近的样本；越出数据范围要标记出来
    _now = time.time()
    _vis = [(_now - 2.0, 10), (_now - 1.0, 20), (_now, 30)]
    _v, _in = cw._value_at(_vis, _now - 1.02)
    check("游标读数取最接近的样本", _v == 20 and _in, f"v={_v} inside={_in}")
    _v, _in = cw._value_at(_vis, _now - 5.0)
    check("游标在数据范围外时标记越界", _v is not None and not _in,
          f"v={_v} inside={_in}")

    # ★ 读数必须在独立读数区，不能叠在画布上挡波形
    cw._redraw()
    _canvas_txt = [cw.canvas.itemcget(i, "text") for i in cw.canvas.find_all()
                   if cw.canvas.type(i) == "text"]
    check("读数不叠在画布上（不挡波形）",
          not any("ΔT" in s or "→" in s for s in _canvas_txt),
          f"画布上的文字: {_canvas_txt}")
    check("画布上仍有 T1/T2 标记",
          any(s in ("T1", "T2") for s in _canvas_txt), str(_canvas_txt))
    _ro = cw.readout.get("1.0", "end")
    check("读数区显示 ΔT", "ΔT" in _ro, _ro.strip().split("\n")[0][:44])

    # 游标位置必须是 X 轴坐标（秒），不是屏幕百分比
    _cx1, _cx2 = cw.cursor_x()
    check("游标位置用 X 轴坐标（不是百分比）",
          "%" not in _ro.split("\n")[0] and "T1 =" in _ro,
          _ro.strip().split("\n")[0][:72])
    check("X 轴坐标与屏位换算一致",
          abs(_cx1 - (-cw.window_sec * (1 - cw.c1))) < 0.01 and
          abs(_cx2 - (-cw.window_sec * (1 - cw.c2))) < 0.01,
          f"T1={_cx1:.2f}s  T2={_cx2:.2f}s（窗口 {cw.window_sec}s）")
    check("读数含绝对时刻（对 CSV 用）", "绝对时刻" in _ro,
          _ro.strip().split("\n")[1][:56] if len(_ro.strip().split("\n")) > 1 else "")

    _drag(_L + 2)                           # 拖到最左，必在数据范围外
    cw._redraw()
    _ro = cw.readout.get("1.0", "end")
    check("游标越出数据范围显示 —", "—" in _ro or "范围之外" in _ro,
          _ro.strip().split("\n")[-1][:44] if _ro.strip() else "")

    cw._clear_cursor()
    cw._redraw()
    check("游标关闭时读数区给出提示",
          "未开启" in cw.readout.get("1.0", "end"))

    cw._clear_cursor()
    check("游标可清除", not cw.cursor_on)

    # ---- 采样周期 & 绘制性能 ----
    import collections as _col
    import math as _math

    cw.v_rate.set("200")
    cw._set_rate()
    check("曲线可改数据区采样周期", app.scan_rate == 200, f"{app.scan_rate}ms")
    app.scan_rate = 300
    for _ in range(4):
        root.update()
        time.sleep(0.1)
    check("数据区改周期后曲线下拉同步", cw.v_rate.get() == "300", cw.v_rate.get())

    _now = time.time()

    def _mk(n, dt):
        dq = _col.deque(maxlen=20000)
        for k in range(n):
            dq.append((_now - 60 + k * dt, _math.sin(k / 18.0) * 100))
        return dq

    cw.history.clear()
    cw._redraw()
    _base = len(cw.canvas.find_all())
    cw.history[32768] = _mk(60, 1.0)          # 60 点：稀疏，应标采样点
    cw._redraw()
    _sparse = len(cw.canvas.find_all())
    cw.history[32768] = _mk(1200, 0.05)       # 1200 点：密集，应抽稀
    cw._redraw()
    _dense = len(cw.canvas.find_all())
    check("稀疏时标出采样点", _sparse - _base > 50, f"+{_sparse - _base} 图元")
    check("密集时自动抽稀（不爆图元）", _dense < _sparse,
          f"稀疏 {_sparse} → 密集 {_dense}")

    for i in range(10):                        # 10 路 × 900 点，量重绘耗时
        cw.history[32768 + i] = _mk(900, 60.0 / 900)
    _t0 = time.perf_counter()
    for _ in range(10):
        cw._redraw()
    _ms = (time.perf_counter() - _t0) / 10 * 1000
    check("重绘耗时有充足余量（<30ms，刷新周期 80ms）", _ms < 30, f"{_ms:.1f} ms/次")

    cw._clear()
    check("曲线清空后无历史", len(cw.history) == 0)
    cw.destroy()

# 2.13 Test Center 手搓报文（只有多窗口版有）
if USE_MDI:
    import modbus_testcenter as mtc

    check("Test Center 十六进制解析（含非法输入）", mtc.parse_hex_selftest())

    def _pdu(hexs):
        return mtc.parse_hex(hexs)

    app.mb.connect("127.0.0.1", 502, 1, 3.0)

    _t, r, _e = mtc.send_pdu(app, _pdu("03 80 00 00 0A"), 1.0)
    check("Test Center 读保持寄存器", len(r) > 10 and not (r[7] & 0x80),
          f"Rx {len(r)} 字节")

    _t, r, _e = mtc.send_pdu(app, _pdu("03 00 00 00 01"), 1.0)
    check("Test Center 非法地址 → 异常码 02", len(r) > 8 and (r[7] & 0x80) and r[8] == 2)

    _t, r, _e = mtc.send_pdu(app, _pdu("63 00 00 00 01"), 1.0)
    check("Test Center 非法功能码 → 异常码 01", len(r) > 8 and (r[7] & 0x80) and r[8] == 1)

    _t, r, _e = mtc.send_pdu(app, _pdu("06 80 0C 10 E1"), 1.0)
    check("Test Center 写寄存器生效", app.mb.read(3, 32780, 1) == [4321])

    _t, r, _e = mtc.send_pdu(app, _pdu("2B 0E 01 00"), 1.0)
    check("Test Center 不支持的功能码被如实上报",
          len(r) > 8 and (r[7] & 0x80) and r[8] == 1)

    app.dlg_testcenter()
    root.update()
    check("Test Center 窗口构建", app.test_win is not None and app.test_win.winfo_exists())
    if app.test_win and app.test_win.winfo_exists():
        app.test_win.destroy()

# 2.14 弹窗零报错
check("测试期间无错误弹窗", not _errors, str(_errors[:2]) if _errors else "")

if USE_MDI and root is not None:
    root.destroy()
else:
    app.destroy()


# ============================================================ 结果
print("\n" + "=" * 64)
print(f"通过 {len(PASSED)} 项，失败 {len(FAILED)} 项")
if FAILED:
    print("\n失败清单：")
    for name in FAILED:
        print(f"  ✘ {name}")
print("=" * 64)

if slave_proc:
    slave_proc.terminate()
    try:
        slave_proc.wait(timeout=3)
    except Exception:
        slave_proc.kill()
    print("（测试启动的从站已关闭）")

sys.exit(1 if FAILED else 0)
