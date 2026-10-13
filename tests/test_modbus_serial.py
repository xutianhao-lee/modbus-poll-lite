# -*- coding: utf-8 -*-
"""
test_modbus_serial.py —— 串口 / RTU / ASCII / UDP 协议栈回归测试

没有串口硬件也能跑：
    帧格式层（CRC、LRC、分帧、异常帧）通过 "RTU/ASCII over TCP" 和
    "RTU/ASCII over UDP" 做**端到端**验证，
    只留 pyserial 的 OS 读写那一薄层未覆盖（会单独验证它的接口与报错提示）。
    Modbus UDP（MBAP over UDP）单独用 UdpTransport 端到端验证。

跑法：cd tests && PYTHONIOENCODING=utf-8 python test_modbus_serial.py
退出码：0 = 全过，1 = 有失败，2 = 环境起不来
"""

import os
import socket
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")      # 源码在 ../src
sys.path.insert(0, SRC)

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import modbus_poll_lite as core          # noqa: E402
import modbus_serial as ms               # noqa: E402

PASSED, FAILED = [], []


def check(name, ok, detail=""):
    (PASSED if ok else FAILED).append(name)
    print(f"  {'✔' if ok else '✘'} {name}" + (f"    {detail}" if detail else ""))
    return ok


def section(t):
    print(f"\n{t}")


def port_open(port):
    s = socket.socket()
    s.settimeout(0.4)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def start_slave(port, frame, udp=False):
    cmd = [sys.executable, "-u", os.path.join(SRC, "modbus_tcp_slave.py"),
           "--robot", "--frame", frame, "--port", str(port)]
    if udp:
        cmd.append("--udp")
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(30):
        time.sleep(0.3)
        if port_open(port):
            return p
    p.kill()
    raise RuntimeError(f"从站 {frame}@{port} 起不来")


print("=" * 64)
print("串口 / RTU / ASCII 协议栈回归测试")
print("=" * 64)

slaves = []
try:
    # ==================================================== 1. 校验算法
    section("【1】校验算法")

    check("CRC16 标准向量 \"123456789\" -> 0x4B37",
          ms.crc16(b"123456789") == 0x4B37,
          f"实得 0x{ms.crc16(b'123456789'):04X}")

    # 另一个已知向量：01 03 00 00 00 0A -> CRC 0xCDC5
    f = bytes.fromhex("01030000000A")
    check("CRC16 向量 01 03 00 00 00 0A -> 0xC5CD(低前)",
          ms.crc16(f) == 0xCDC5, f"实得 0x{ms.crc16(f):04X}")

    check("LRC 向量 01 03 00 00 00 0A -> 0xF2",
          ms.lrc(bytes.fromhex("01030000000A")) == 0xF2,
          f"实得 0x{ms.lrc(bytes.fromhex('01030000000A')):02X}")

    # ==================================================== 2. 帧编解码
    section("【2】帧编解码")

    pdu = struct.pack(">BHH", 3, 32768, 10)

    rtu = ms.RtuCodec.build(1, pdu)
    check("RTU 帧长度 = unit + pdu + 2 字节 CRC", len(rtu) == 1 + len(pdu) + 2,
          rtu.hex(" ").upper())
    u, back = ms.RtuCodec.parse(rtu)
    check("RTU 往返：unit 与 pdu 一致", u == 1 and back == pdu)

    bad = bytearray(rtu)
    bad[-1] ^= 0xFF
    try:
        ms.RtuCodec.parse(bytes(bad))
        check("RTU CRC 错帧应被拒绝", False, "竟然通过了")
    except ValueError as e:
        check("RTU CRC 错帧被拒绝", "CRC" in str(e), str(e))

    asc = ms.AsciiCodec.build(1, pdu)
    check("ASCII 帧以 ':' 开头 CRLF 结尾", asc.startswith(b":") and asc.endswith(b"\r\n"),
          asc.decode())
    u, back = ms.AsciiCodec.parse(asc)
    check("ASCII 往返：unit 与 pdu 一致", u == 1 and back == pdu)

    bad = bytearray(asc)
    bad[3] = ord("0") if bad[3] != ord("0") else ord("1")
    try:
        ms.AsciiCodec.parse(bytes(bad))
        check("ASCII LRC 错帧应被拒绝", False, "竟然通过了")
    except ValueError as e:
        check("ASCII LRC 错帧被拒绝", True, str(e)[:40])

    # 分帧长度推算
    check("RTU 长度推算 05/06/15/16 定长 8",
          all(ms.RtuCodec.frame_length(bytes([1, fc])) == 8 for fc in (5, 6, 15, 16)))
    check("RTU 长度推算 01~04 需读 byte_count",
          all(ms.RtuCodec.frame_length(bytes([1, fc])) is None for fc in (1, 2, 3, 4)))
    check("RTU 长度推算 异常帧定长 5",
          ms.RtuCodec.frame_length(bytes([1, 0x83])) == 5)
    check("RTU 长度推算 23 应答同读响应（看 byte_count）",
          ms.RtuCodec.frame_length(bytes([1, 23])) is None)
    check("RTU 长度推算 43/14 变长（专门逐对象读）",
          ms.RtuCodec.frame_length(bytes([1, 0x2B])) is None)

    # ==================================================== 3. 端到端
    def protocol_suite(master, label):
        section(f"【3】端到端 —— {label}")
        try:
            ok = True
            for fc, a in ((2, 0), (1, 4096), (4, 0), (3, 32768)):
                master.read(fc, a, 1)
            check(f"{label}：四段 LEGAL 边界可读", ok)

            bad_ok = True
            for fc, a in ((3, 0), (3, 32767), (1, 4095), (4, 32768)):
                try:
                    master.read(fc, a, 1)
                    bad_ok = False
                except core.ModbusError as e:
                    if e.code != 2:
                        bad_ok = False
            check(f"{label}：越界回异常码 02", bad_ok)

            v = master.read(4, 0, 6)
            check(f"{label}：FC04 读数正确", v == [0, 3, 6, 9, 12, 15], str(v))

            v = master.read(3, 32768, 10)
            check(f"{label}：FC03 读 10 个", len(v) == 10, f"首值 {v[0]}")

            master.write_single_register(32780, 4321)
            check(f"{label}：FC06 写回读", master.read(3, 32780, 1) == [4321])

            master.write_multiple_registers(32781, [7, 8, 9])
            check(f"{label}：FC16 写多回读", master.read(3, 32781, 3) == [7, 8, 9])

            master.write_single_coil(4200, True)
            check(f"{label}：FC05 写线圈回读", master.read(1, 4200, 1) == [True])

            master.write_multiple_coils(4201, [True, False, True])
            check(f"{label}：FC15 写多线圈回读",
                  master.read(1, 4201, 3) == [True, False, True])

            # 数量超限 -> 异常码 03
            q_ok = False
            try:
                master.read(3, 32768, 130)
            except core.ModbusError as e:
                q_ok = (e.code == 3)
            check(f"{label}：数量 130 -> 异常码 03", q_ok)

            # 功能码 23：写 2 + 同帧读回 2（覆盖 FC23 的分帧路径）
            r = master.request(struct.pack(">BHHHHB", 23, 32790, 2, 32790, 2, 4)
                               + struct.pack(">HH", 4242, 2424))
            check(f"{label}：FC23 写 2 + 读回 2",
                  r[0] == 23 and r[1] == 4 and struct.unpack(">HH", r[2:6]) == (4242, 2424),
                  r.hex(" ").upper())

            # 功能码 22：掩码写（响应回显，定长 10 字节帧——之前没在 RTU 下测过）
            master.write_single_register(32780, 0xFF00)
            r = master.request(struct.pack(">BHHH", 22, 32780, 0xFF0F, 0x00F0))
            check(f"{label}：FC22 掩码写回显",
                  r == struct.pack(">BHHH", 22, 32780, 0xFF0F, 0x00F0)
                  and master.read(3, 32780, 1) == [0xFFF0], r.hex(" ").upper())

            # 功能码 43/14：基本读取（覆盖变长应答的分帧路径）
            r = master.request(struct.pack(">BBBB", 0x2B, 0x0E, 1, 0))
            check(f"{label}：FC43/14 基本读取可解析",
                  r[0] == 0x2B and r[1] == 0x0E and len(r) > 7 and r[6] >= 3,
                  r.hex(" ").upper()[:60])

            # 广播：立即返回
            master.unit = 0
            t0 = time.time()
            r = master.request(struct.pack(">BHH", 6, 32780, 5))
            dt = time.time() - t0
            master.unit = 1
            check(f"{label}：广播立即返回", r == b"" and dt < 0.2, f"{dt:.3f}s")

            check(f"{label}：报文记录有收发", len(master.traffic) >= 20,
                  f"{len(master.traffic)} 条")
            return True
        finally:
            master.close()

    # --- RTU over TCP（覆盖串口的帧格式层）---
    p_rtu = start_slave(5031, "rtu")
    slaves.append(p_rtu)
    rtu = ms.ModbusSerialMaster("RTU")
    rtu.connect_tcp("127.0.0.1", 5031, timeout=2.0)
    check("RTU over TCP 连接建立", rtu.connected, rtu.peer)
    protocol_suite(rtu, "RTU over TCP")

    # --- ASCII over TCP ---
    p_asc = start_slave(5032, "ascii")
    slaves.append(p_asc)
    asc = ms.ModbusSerialMaster("ASCII")
    asc.connect_tcp("127.0.0.1", 5032, timeout=2.0)
    check("ASCII over TCP 连接建立", asc.connected, asc.peer)
    protocol_suite(asc, "ASCII over TCP")

    # ==================================================== 3b. UdpTransport
    section("【3b】UdpTransport 传输层（本机 UDP 回显套接字）")
    echo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    echo.bind(("127.0.0.1", 0))
    _ea = echo.getsockname()
    t = ms.UdpTransport(_ea[0], _ea[1], 1.0)
    check("UdpTransport 构造", True, t.description)
    t.sendall(b"\x01\x02\x03")
    _d, _peer = echo.recvfrom(64)
    check("UdpTransport 发送数据报", _d == b"\x01\x02\x03", _d.hex(" ").upper())
    echo.sendto(_d + b"\xAA", _peer)
    _back = t.read_exact(4)
    check("UdpTransport read_exact 缓冲累积", _back == b"\x01\x02\x03\xAA",
          _back.hex(" ").upper())
    t.sendall(b"AB\r\n")
    _d, _peer = echo.recvfrom(64)
    echo.sendto(b"AB\r\n", _peer)
    _got = t.read_until(b"\r\n", limit=16)
    check("UdpTransport read_until 按终止符收", _got == b"AB\r\n", str(_got))
    t.close()
    echo.close()

    # ==================================================== 3c. UDP 系列
    section("【3c】UDP 系列 —— Modbus UDP / RTU over UDP / ASCII over UDP")

    p_udp = start_slave(5041, "mbap", udp=True)
    slaves.append(p_udp)
    udp = ms.ModbusUdpMaster()
    udp.connect("127.0.0.1", 5041, 1, timeout=2.0)
    check("Modbus UDP 连接建立", udp.connected, udp.peer)
    protocol_suite(udp, "Modbus UDP")

    p_rtu_udp = start_slave(5042, "rtu", udp=True)
    slaves.append(p_rtu_udp)
    rtu_udp = ms.ModbusSerialMaster("RTU")
    rtu_udp.connect_transport(ms.UdpTransport("127.0.0.1", 5042, 2.0))
    check("RTU over UDP 连接建立", rtu_udp.connected, rtu_udp.peer)
    protocol_suite(rtu_udp, "RTU over UDP")

    p_asc_udp = start_slave(5043, "ascii", udp=True)
    slaves.append(p_asc_udp)
    asc_udp = ms.ModbusSerialMaster("ASCII")
    asc_udp.connect_transport(ms.UdpTransport("127.0.0.1", 5043, 2.0))
    check("ASCII over UDP 连接建立", asc_udp.connected, asc_udp.peer)
    protocol_suite(asc_udp, "ASCII over UDP")

    _need = ["connect", "close", "request", "read", "write_single_register",
             "write_single_coil", "write_multiple_registers", "write_multiple_coils"]
    _missing = [n for n in _need if not hasattr(ms.ModbusUdpMaster(), n)]
    check("UDP 主站与 TCP 主站接口一致", not _missing,
          f"缺 {_missing}" if _missing else "")

    # ==================================================== 4. 串口层
    section("【4】pyserial 传输层（无硬件，只验接口与报错）")

    try:
        import serial  # noqa: F401
        check("pyserial 可导入", True, serial.__version__)
    except ImportError as e:
        check("pyserial 可导入", False, str(e))

    ports = ms.available_ports()
    check("串口枚举接口可用", isinstance(ports, list),
          f"当前检测到 {len(ports)} 个串口" + (f"：{ports}" if ports else "（没插设备）"))

    try:
        t = ms.SerialTransport("COM_NONEXISTENT_999", timeout=0.5)
        check("不存在的串口应抛异常", False, "竟然打开了")
    except Exception as e:
        check("不存在的串口抛出明确异常", True, f"{type(e).__name__}: {str(e)[:50]}")

    # 主站接口一致性：串口主站与 TCP 主站应暴露同一套方法
    need = ["connect", "close", "request", "read", "write_single_register",
            "write_single_coil", "write_multiple_registers", "write_multiple_coils"]
    missing = [n for n in need if not hasattr(ms.ModbusSerialMaster("RTU"), n)]
    check("串口主站与 TCP 主站接口一致", not missing, f"缺 {missing}" if missing else "")

    # ==================================================== 5. 串口传输层
    # 没有串口硬件，用 pyserial 自带的 loop:// 回环验证 SerialTransport。
    # 覆盖：能不能开、参数字段认不认、写进去读不读得回来、关得掉。
    # **不覆盖**：Windows 串口驱动那一层（要有真硬件或虚拟串口对才行）。
    section("【5】SerialTransport 传输层（pyserial loop:// 回环）")
    try:
        t = ms.SerialTransport("loop://", baud=19200, bytesize=8,
                               parity="E", stopbits=1, timeout=0.5)
        check("SerialTransport 能打开 loop://", True, t.description)

        payload = ms.RtuCodec.build(1, struct.pack(">BHH", 3, 32768, 10))
        t.sendall(payload)
        back = t.read_exact(len(payload))
        check("sendall / read_exact 回环一致", back == payload,
              f"发 {payload.hex(' ').upper()}")

        t.sendall(b"\x01\x02\x03")
        got = t.read_available(0.5)
        check("read_available 收回全部字节", got == b"\x01\x02\x03",
              got.hex(" ").upper())

        t.sendall(b"AB\r\n")
        got = t.read_until(b"\r\n", limit=16)
        check("read_until 按终止符收", got == b"AB\r\n", got)

        t.close()
        check("SerialTransport 能关闭", True)
    except Exception as e:
        check("SerialTransport 传输层", False, f"{type(e).__name__}: {e}")

finally:
    for p in slaves:
        p.terminate()
        try:
            p.wait(timeout=3)
        except Exception:
            p.kill()

print("\n" + "=" * 64)
print(f"通过 {len(PASSED)} 项，失败 {len(FAILED)} 项")
if FAILED:
    print("\n失败清单：")
    for n in FAILED:
        print(f"  ✘ {n}")
print("=" * 64)
sys.exit(1 if FAILED else 0)
