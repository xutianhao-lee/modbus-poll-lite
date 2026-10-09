# -*- coding: utf-8 -*-
"""
modbus_testcenter.py —— Test Center：手工构造报文发出去，看从站的原始应答

两种模式：
    自动封装：你只写 PDU（如 `03 80 00 00 0A`），工具按当前连接方式
              自动补 从站号 / MBAP 头 / CRC16 或 LRC，并把应答解析给你看
    原始字节：你写什么就发什么，收到什么就显示什么（不封装、不校验）
              用途：非标帧、调试从站实现、验证自己算的 CRC 对不对

用途：Modbus Poll 的 Test Center 是用来啃"标准功能码覆盖不到"的场合 ——
      私有功能码、畸形帧、从站实现的边界行为。

挂在 DataArea 上，复用它的连接。
"""

import socket
import struct
import tkinter as tk
from tkinter import ttk, messagebox

import modbus_poll_lite as core

# 常用报文模板（PDU，十六进制）
TEMPLATES = [
    ("读保持寄存器 03 @32768 数量10", "03 80 00 00 0A"),
    ("读保持寄存器 03 @0（非法地址）", "03 00 00 00 01"),
    ("读线圈 01 @4096 数量8", "01 10 00 00 08"),
    ("写单寄存器 06 @32768 = 1234", "06 80 00 04 D2"),
    ("写单线圈 05 @4096 = ON", "05 10 00 FF 00"),
    ("读设备标识 43/14", "2B 0E 01 00"),
    ("非法功能码 99（测异常码 01）", "63 00 00 00 01"),
]


def parse_hex(text):
    """把 '03 80 00 00 0A' / '03,80,00,00,0A' / '0380 0000 0A' 解析成字节"""
    cleaned = text.replace(",", " ").replace("\n", " ").replace("\t", " ")
    cleaned = "".join(cleaned.split())
    if not cleaned:
        raise ValueError("没填内容")
    if len(cleaned) % 2:
        raise ValueError(f"十六进制位数为奇数（{len(cleaned)} 位），没法凑成字节")
    try:
        return bytes.fromhex(cleaned)
    except ValueError:
        bad = [c for c in cleaned if c not in "0123456789abcdefABCDEF"]
        raise ValueError(f"含非十六进制字符：{''.join(sorted(set(bad)))[:20]}")


def send_pdu(area, pdu, timeout=1.0):
    """按当前连接方式封装 PDU 并发送，返回 (发出去的完整帧, 收到的完整帧, 错误说明)"""
    mb = area.mb
    if not mb.connected:
        return b"", b"", "未连接"
    frame = b""
    is_tcp = hasattr(mb, "sock")
    with mb.lock:
        try:
            if is_tcp:
                mb.tid = (mb.tid + 1) & 0xFFFF
                frame = struct.pack(">HHHB", mb.tid, 0, len(pdu) + 1, mb.unit) + pdu
                mb.tx += 1
                mb.sock.sendall(frame)
                if mb.unit == 0:
                    return frame, b"", "广播（从站号 0）：按规范不应答"
                old = mb.sock.gettimeout()
                mb.sock.settimeout(timeout)
                try:
                    hdr = mb._recv_exact(7)
                    _tid, _pid, length, _uid = struct.unpack(">HHHB", hdr)
                    body = mb._recv_exact(length - 1) if length > 1 else b""
                finally:
                    mb.sock.settimeout(old)
                return frame, hdr + body, ""
            else:
                frame = mb.codec.build(mb.unit, pdu)
                mb.tx += 1
                mb.transport.sendall(frame)
                if mb.unit == 0:
                    return frame, b"", "广播（从站号 0）：按规范不应答"
                return frame, mb._recv_frame(), ""
        except Exception as e:
            return frame, b"", f"{type(e).__name__}: {e}"


def send_raw(area, data, timeout=1.0):
    """原样发送，收回什么算什么"""
    mb = area.mb
    if not mb.connected:
        return b"", "未连接"
    is_tcp = hasattr(mb, "sock")
    with mb.lock:
        try:
            if is_tcp:
                old = mb.sock.gettimeout()
                mb.sock.settimeout(0.2)
                try:
                    mb.sock.sendall(data)
                    rx = b""
                    deadline = timeout
                    while deadline > 0:
                        try:
                            chunk = mb.sock.recv(4096)
                        except (socket.timeout, TimeoutError):
                            deadline = 0
                            break
                        if not chunk:
                            break
                        rx += chunk
                        if len(rx) > 8192:
                            break
                        deadline -= 0.2
                finally:
                    mb.sock.settimeout(old)
                return rx, ""
            else:
                mb.transport.sendall(data)
                if hasattr(mb.transport, "read_available"):
                    return mb.transport.read_available(timeout), ""
                return mb.transport.read_exact(256), ""
        except Exception as e:
            return b"", f"{type(e).__name__}: {e}"


def describe(area, rx_frame):
    """把应答帧翻译成人话"""
    out = []
    if not rx_frame:
        out.append("（无应答）")
        out.append("  可能原因：超时 / 从站号不匹配（从站不理你）/ 广播 / "
                   "CRC 或 LRC 算错被从站丢弃")
        return out
    mb = area.mb
    if hasattr(mb, "sock"):
        if len(rx_frame) < 8:
            out.append(f"响应过短（{len(rx_frame)} 字节），不是合法的 Modbus TCP 应答")
            return out
        tid, pid, length, uid = struct.unpack(">HHHB", rx_frame[:7])
        pdu = rx_frame[7:]
        out.append(f"MBAP：事务号={tid}  协议号={pid}  长度={length}  从站号={uid}")
    else:
        try:
            uid, pdu = mb.codec.parse(rx_frame)
        except ValueError as e:
            out.append(f"✘ 帧校验失败：{e}")
            return out
        out.append(f"{mb.codec.name} 帧校验通过   从站号={uid}")

    if not pdu:
        out.append("PDU 为空")
        return out
    fc = pdu[0]
    if fc & 0x80:
        code = pdu[1] if len(pdu) > 1 else 0
        out.append(f"⚠ 异常响应：功能码 {fc & 0x7F:02d} → 异常码 {code:02d}  "
                   f"{core.exc_text(code)}")
    else:
        out.append(f"✔ 正常响应：功能码 {fc:02d}，PDU 共 {len(pdu)} 字节 → {pdu.hex(' ').upper()}")
        if fc in (1, 2, 3, 4) and len(pdu) >= 2:
            n = pdu[1]
            data = pdu[2:2 + n]
            if fc in (3, 4) and len(data) >= 2:
                vals = struct.unpack(">" + "H" * (len(data) // 2), data)
                out.append(f"   寄存器值：{list(vals)}")
            elif fc in (1, 2):
                bits = [bool(data[i // 8] >> (i % 8) & 1) for i in range(n * 8)]
                out.append(f"   位值：{[int(b) for b in bits]}")
    return out


class TestCenter(tk.Toplevel):
    """一个数据区一个 Test Center 窗口"""

    def __init__(self, area):
        super().__init__(area)
        self.area = area
        self.title(f"Test Center —— 数据区 {area.index}")
        self.geometry("820x560")
        self.minsize(640, 440)
        self._build()

    def _build(self):
        top = ttk.Frame(self, padding=(10, 8, 10, 4))
        top.pack(fill="x")

        self.v_mode = tk.StringVar(value="pdu")
        ttk.Radiobutton(top, text="自动封装（只写 PDU，工具补从站号/MBAP/CRC）",
                        variable=self.v_mode, value="pdu").pack(anchor="w")
        ttk.Radiobutton(top, text="原始字节（写什么发什么，不封装不校验）",
                        variable=self.v_mode, value="raw").pack(anchor="w")

        mid = ttk.Frame(self, padding=(10, 4))
        mid.pack(fill="x")
        ttk.Label(mid, text="十六进制").pack(side="left")
        self.ent = tk.Text(mid, height=3, font=("Consolas", 11))
        self.ent.pack(side="left", fill="x", expand=True, padx=8)
        self.ent.insert("1.0", "03 80 00 00 0A")

        btns = ttk.Frame(self, padding=(10, 2))
        btns.pack(fill="x")
        ttk.Button(btns, text="发送", command=self._send).pack(side="left")
        ttk.Button(btns, text="清空输出", command=self._clear).pack(side="left", padx=6)
        ttk.Label(btns, text="超时 (s)").pack(side="left", padx=(16, 4))
        self.v_to = tk.StringVar(value="1.0")
        ttk.Entry(btns, textvariable=self.v_to, width=6).pack(side="left")

        tpl = ttk.LabelFrame(self, text="常用报文（点一下填进输入框）", padding=(8, 4))
        tpl.pack(fill="x", padx=10, pady=(6, 0))
        for i, (label, hexs) in enumerate(TEMPLATES):
            ttk.Button(tpl, text=label, width=30,
                       command=lambda h=hexs: self._fill(h)).grid(
                row=i // 2, column=i % 2, sticky="w", padx=2, pady=1)

        ttk.Label(self, text="发送记录", padding=(10, 6, 10, 0)).pack(anchor="w")
        self.out = tk.Text(self, font=("Consolas", 9), wrap="word",
                           bg="#fbfbfb", state="disabled")
        self.out.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _fill(self, hexs):
        self.ent.delete("1.0", "end")
        self.ent.insert("1.0", hexs)

    def _clear(self):
        self.out.configure(state="normal")
        self.out.delete("1.0", "end")
        self.out.configure(state="disabled")

    def _log(self, text, tag=""):
        self.out.configure(state="normal")
        self.out.insert("end", text + "\n")
        self.out.see("end")
        self.out.configure(state="disabled")

    def _send(self):
        try:
            data = parse_hex(self.ent.get("1.0", "end"))
        except ValueError as e:
            messagebox.showerror("输入有误", str(e), parent=self)
            return
        try:
            timeout = max(0.1, float(self.v_to.get()))
        except ValueError:
            timeout = 1.0

        raw_mode = self.v_mode.get() == "raw"
        self._log("─" * 68)
        if raw_mode:
            tx, err = send_raw(self.area, data, timeout)
            self._log(f"[原始模式] 发送 {len(data)} 字节：{data.hex(' ').upper()}")
            if err:
                self._log(f"  发送/接收出错：{err}")
            self._log(f"  收到 {len(tx)} 字节：{tx.hex(' ').upper() if tx else '（空）'}")
            return

        tx, rx, err = send_pdu(self.area, data, timeout)
        self._log(f"[自动封装] PDU  {data.hex(' ').upper()}")
        self._log(f"  Tx  {tx.hex(' ').upper() if tx else '（未发出）'}")
        if err:
            self._log(f"  ✘ {err}")
        self._log(f"  Rx  {rx.hex(' ').upper() if rx else '（空）'}")
        for line in describe(self.area, rx):
            self._log("  " + line)
        # 同步进数据区的报文窗口
        try:
            mb = self.area.mb
            if tx:
                mb._log("Tx", tx, "Test Center")
            if rx:
                mb._log("Rx", rx, "Test Center")
        except Exception:
            pass


def parse_hex_selftest():
    """给回归测试用：验证十六进制解析的边界"""
    assert parse_hex("03 80 00 00 0A") == bytes.fromhex("038000000A")
    assert parse_hex("03,80,00,00,0A") == bytes.fromhex("038000000A")
    assert parse_hex("0380 00000A") == bytes.fromhex("038000000A")
    for bad in ("", "038000000A0", "0G 01"):
        try:
            parse_hex(bad)
            raise AssertionError(f"应当拒绝：{bad!r}")
        except ValueError:
            pass
    return True
