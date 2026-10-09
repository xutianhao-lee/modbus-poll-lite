# -*- coding: utf-8 -*-
"""
modbus_tcp_slave.py —— 零依赖 Modbus TCP 从站模拟器（练手 / 自测用）

没有真实设备时，在本机起一个 Modbus 从站，用 Modbus Poll 连
127.0.0.1:502 就能练读写、看报文、试异常码。

支持功能码：01 读线圈 / 02 读离散输入 / 03 读保持寄存器 / 04 读输入寄存器
            05 写单线圈 / 06 写单寄存器 / 15 写多线圈 / 16 写多寄存器
            22 掩码写寄存器
不支持的功能码回异常码 01。

两种模式：
  【默认模式】地址随便用，方便练软件操作
    保持寄存器 03：0 递增计数 / 1 运行秒数 / 2 模拟转速 / 3 负载率
                   4-5 32位长整型 / 6-7 32位浮点温度 / 8-99 可写空白区
    输入寄存器 04：地址×3    线圈 01：0=运行标志(闪烁)    离散输入 02：3的倍数
  【--robot 模式】按机器人控制器从站地址表分段，用于实操预习
    离散量输入 0~4095      FC02 只读
    线圈       4096~8191    FC01/05/15 读写
    输入寄存器 0~32767     FC04 只读
    保持寄存器 32768~65535 FC03/06/16 读写
    → FC03 读地址 0 会回异常码 02，与官方课件里的 "Illegal Data Address" 现象一致

运行：PYTHONIOENCODING=utf-8 python src/modbus_tcp_slave.py [--robot] [--port 502] [--unit 1]
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

SIZE = 32768
ROBOT = False          # --robot：按机器人从站地址表分段
# 机器人从站地址表：功能码 -> (起始地址, 长度)
ROBOT_RANGE = {1: (4096, 4096), 2: (0, 4096), 3: (32768, 32768), 4: (0, 32768)}

HR = [0] * SIZE          # 保持寄存器 03
IR = [0] * SIZE          # 输入寄存器 04
CO = [False] * SIZE      # 线圈 01
DI = [False] * SIZE      # 离散输入 02
LOCK = threading.Lock()

FC_NAME = {1: "读线圈", 2: "读离散输入", 3: "读保持寄存器", 4: "读输入寄存器",
           5: "写单线圈", 6: "写单寄存器", 15: "写多线圈", 16: "写多寄存器",
           22: "掩码写寄存器"}
EXC_NAME = {1: "非法功能码", 2: "非法数据地址", 3: "非法数据值", 4: "从站设备故障"}


def _hi_lo(v):
    """32 位整数 → 两个寄存器（高字在前）"""
    return struct.unpack(">HH", struct.pack(">I", v & 0xFFFFFFFF))


def _f32_regs(v):
    """32 位浮点 → 两个寄存器（高字在前）"""
    return struct.unpack(">HH", struct.pack(">f", float(v)))


def init_data():
    with LOCK:
        for i in range(SIZE):
            IR[i] = (i * 3) % 10000          # 输入寄存器：固定图案，方便验证地址算法
            DI[i] = (i % 3 == 0)             # 离散输入：3 的倍数为 ON
        for i in range(8, 100):
            HR[i] = i * 7 % 10000            # 空白区预置内容，免得全是 0 看不出在读


def updater():
    """后台刷新模拟量，让画面上的数字动起来"""
    t0 = time.time()
    n = 0
    while True:
        time.sleep(0.2)
        n += 1
        t = time.time() - t0
        phase = (t % 8) / 8.0
        rpm = int(1500 * (phase * 2 if phase < 0.5 else (1 - phase) * 2))
        temp = 25.0 + 15.0 * math.sin(t / 5.0)
        total = int(t * 123)
        with LOCK:
            HR[0] = n % 10000
            HR[1] = int(t) % 65536
            HR[2] = rpm
            HR[3] = int(rpm / 15)
            HR[4], HR[5] = _hi_lo(total)
            HR[6], HR[7] = _f32_regs(temp)
            CO[0] = (n % 10) < 5             # 运行标志：1 秒闪一次
            CO[1] = False


def crc16(data):
    """Modbus RTU 的 CRC16（0xA001，初值 0xFFFF）；b"123456789" -> 0x4B37"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def exception(fc, code):
    return bytes([fc | 0x80, code])


def to_index(fc, addr, qty):
    """请求地址 -> 内部数组下标；地址非法返回 None"""
    if not ROBOT:
        return addr if addr + qty <= SIZE else None
    base, length = ROBOT_RANGE[fc]
    if addr < base or addr + qty > base + length:
        return None
    return addr - base


def handle_pdu(pdu, tag):
    """处理一个请求 PDU，返回响应 PDU"""
    fc = pdu[0]

    if fc in (1, 2, 3, 4):
        if len(pdu) < 5:
            return exception(fc, 3)
        addr, qty = struct.unpack(">HH", pdu[1:5])
        limit = 2000 if fc in (1, 2) else 125
        if qty < 1 or qty > limit:
            print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty} → 异常码 03（数量超限，上限{limit}）")
            return exception(fc, 3)
        with LOCK:
            src = {1: CO, 2: DI, 3: HR, 4: IR}[fc]
            idx = to_index(fc, addr, qty)
            if idx is None:
                print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty} → 异常码 02（非法数据地址）")
                return exception(fc, 2)
            if fc in (1, 2):
                nbytes = (qty + 7) // 8
                buf = bytearray(nbytes)
                for i in range(qty):
                    if src[idx + i]:
                        buf[i // 8] |= 1 << (i % 8)
                resp = bytes([fc, nbytes]) + bytes(buf)
            else:
                data = b"".join(struct.pack(">H", src[idx + i]) for i in range(qty))
                resp = bytes([fc, len(data)]) + data
                preview = ", ".join(str(src[idx + i]) for i in range(min(qty, 6)))
                preview += "..." if qty > 6 else ""
                print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty} → [{preview}]")
                return resp
            print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty} → {qty} 位")
            return resp

    if fc == 5:
        if len(pdu) < 5:
            return exception(fc, 3)
        addr, val = struct.unpack(">HH", pdu[1:5])
        if val not in (0x0000, 0xFF00):
            print(f"  {tag} 写单线圈 地址{addr} 值0x{val:04X} → 异常码 03（线圈值只能是 0xFF00/0x0000）")
            return exception(fc, 3)
        with LOCK:
            idx = to_index(1, addr, 1)
            if idx is None:
                print(f"  {tag} {FC_NAME[fc]} 地址{addr} → 异常码 02（非法数据地址）")
                return exception(fc, 2)
            CO[idx] = (val == 0xFF00)
        print(f"  {tag} {FC_NAME[fc]} 地址{addr} = {'ON' if CO[idx] else 'OFF'}")
        return pdu

    if fc == 6:
        if len(pdu) < 5:
            return exception(fc, 3)
        addr, val = struct.unpack(">HH", pdu[1:5])
        with LOCK:
            idx = to_index(3, addr, 1)
            if idx is None:
                print(f"  {tag} 写单寄存器 地址{addr} → 异常码 02（非法数据地址）")
                return exception(fc, 2)
            HR[idx] = val
        print(f"  {tag} {FC_NAME[fc]} 地址{addr} = {val}")
        return pdu

    if fc == 15:
        if len(pdu) < 6:
            return exception(fc, 3)
        addr, qty, nb = struct.unpack(">HHB", pdu[1:6])
        if qty < 1 or qty > 1968 or nb != (qty + 7) // 8 or len(pdu) < 6 + nb:
            return exception(fc, 3)
        with LOCK:
            idx = to_index(1, addr, qty)
            if idx is None:
                return exception(fc, 2)
            for i in range(qty):
                CO[idx + i] = bool(pdu[6 + i // 8] >> (i % 8) & 1)
        print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty}")
        return struct.pack(">BHH", fc, addr, qty)

    if fc == 16:
        if len(pdu) < 6:
            return exception(fc, 3)
        addr, qty, nb = struct.unpack(">HHB", pdu[1:6])
        if qty < 1 or qty > 123 or nb != qty * 2 or len(pdu) < 6 + nb:
            return exception(fc, 3)
        with LOCK:
            idx = to_index(3, addr, qty)
            if idx is None:
                return exception(fc, 2)
            for i in range(qty):
                HR[idx + i] = struct.unpack(">H", pdu[6 + i * 2:8 + i * 2])[0]
        print(f"  {tag} {FC_NAME[fc]} 地址{addr} 数量{qty}")
        return struct.pack(">BHH", fc, addr, qty)

    if fc == 22:
        if len(pdu) < 7:
            return exception(fc, 3)
        addr, and_m, or_m = struct.unpack(">HHH", pdu[1:7])
        with LOCK:
            idx = to_index(3, addr, 1)
            if idx is None:
                print(f"  {tag} 掩码写寄存器 地址{addr} → 异常码 02（非法数据地址）")
                return exception(fc, 2)
            old = HR[idx]
            # 规范语义：结果 = (当前值 AND 掩码) OR (置位值 AND NOT 掩码)
            HR[idx] = (old & and_m) | (or_m & ~and_m & 0xFFFF)
        print(f"  {tag} {FC_NAME[fc]} 地址{addr} AND=0x{and_m:04X} OR=0x{or_m:04X}"
              f" → {old} → {HR[idx]}")
        return pdu

    print(f"  {tag} 功能码 {fc} → 异常码 01（本模拟器不支持）")
    return exception(fc, 1)


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def lrc(data):
    """Modbus ASCII 的 LRC：所有字节求和取两补"""
    return (-sum(data)) & 0xFF


def read_rtu_request(conn):
    """从站侧读 RTU **请求**帧：<unit><fc><数据…><crc_lo><crc_hi>

    ⚠ 请求和响应的分帧规则**不一样**，别混用：
        请求 01/02/03/04/05/06  → 定长 8 字节（unit+fc+4 数据+crc）
        请求 15/16              → 9 + byte_count（多了 1 字节字节数）
        响应 01~04              → 第 3 字节才是 byte_count
    """
    head = recv_exact(conn, 2)
    if head is None:
        return None
    if head[1] in (15, 16):
        mid = recv_exact(conn, 5)          # 起始地址(2) + 数量(2) + 字节数(1)
        if mid is None:
            return None
        head += mid
        need = 7 + mid[4] + 2
    else:
        need = 8
    rest = recv_exact(conn, need - len(head))
    if rest is None:
        return None
    return head + rest


def read_ascii_frame(conn):
    """ASCII 帧：':' + 十六进制 + CRLF"""
    buf = b""
    while len(buf) < 512:
        ch = recv_exact(conn, 1)
        if ch is None:
            return None
        buf += ch
        if buf.endswith(b"\r\n"):
            break
    return buf or None


def take_frame(conn, mode, tag):
    """读一帧，返回 (uid, pdu) —— 帧坏或连接断返回 None"""
    if mode == "mbap":
        hdr = recv_exact(conn, 7)
        if hdr is None:
            return None
        tid, _pid, length, uid = struct.unpack(">HHHB", hdr)
        body = recv_exact(conn, length - 1) if length > 1 else b""
        if body is None:
            return None
        return uid, body, tid

    if mode == "rtu":
        frame = read_rtu_request(conn)
        if frame is None:
            return None
        body, got = frame[:-2], frame[-2:]
        want = crc16(body)
        if bytes([want & 0xFF, (want >> 8) & 0xFF]) != got:
            print(f"  {tag} RTU CRC 校验失败，丢弃该帧")
            return None, None, None
        return body[0], body[1:], 0

    # ascii
    frame = read_ascii_frame(conn)
    if frame is None:
        return None
    try:
        raw = bytes.fromhex(frame[1:-2].decode("ascii", "replace").strip())
    except ValueError:
        print(f"  {tag} ASCII 帧十六进制非法，丢弃")
        return None, None, None
    if len(raw) < 2:
        print(f"  {tag} ASCII 帧太短，丢弃")
        return None, None, None
    payload, got = raw[:-1], raw[-1]
    if lrc(payload) != got:
        print(f"  {tag} ASCII LRC 校验失败，丢弃该帧")
        return None, None, None
    return payload[0], payload[1:], 0


def serve_conn(conn, peer, unit_id, any_unit, mode="mbap"):
    tag = f"[{peer[0]}:{peer[1]}]"
    print(f"++ 主站接入 {tag}（帧格式 {mode.upper()}）")
    try:
        while True:
            got = take_frame(conn, mode, tag)
            if got is None:
                break
            uid, pdu, tid = got
            if uid is None:                       # "retry"：坏帧
                continue

            if uid == 0:
                # 广播：执行命令但不发送应答（Modbus 规范）
                handle_pdu(pdu, tag)
                print(f"  {tag} 广播帧（从站号 0）→ 已执行，不应答")
                continue

            if not any_unit and uid != unit_id:
                # 真实设备收到不匹配的从站号就是不理你 —— 主站上表现为超时
                print(f"  {tag} 从站号 {uid} ≠ {unit_id} → 不应答"
                      f"（真实设备同样如此，主站侧表现为超时）")
                continue

            resp = handle_pdu(pdu, tag)
            if mode == "mbap":
                conn.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, uid) + resp)
            elif mode == "rtu":
                out = bytes([uid]) + resp
                c = crc16(out)
                conn.sendall(out + bytes([c & 0xFF, (c >> 8) & 0xFF]))
            else:
                raw = bytes([uid]) + resp
                conn.sendall(b":" + (raw + bytes([lrc(raw)])).hex().upper().encode() + b"\r\n")
    except (ConnectionResetError, OSError):
        pass
    finally:
        conn.close()
        print(f"-- 主站断开 {tag}")


def main():
    global ROBOT
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1, help="本从站的从站号（Unit ID）")
    ap.add_argument("--any-unit", action="store_true", help="对任意从站号都应答")
    ap.add_argument("--robot", action="store_true",
                    help="按机器人控制器从站地址表分段（复现 Illegal Data Address）")
    ap.add_argument("--frame", choices=["mbap", "rtu", "ascii"], default="mbap",
                    help="帧格式：mbap = 标准 Modbus TCP（默认）；"
                         "rtu / ascii = 去掉 MBAP 头，配 TCP 端口用即 RTU/ASCII over TCP，"
                         "用于在没有串口硬件时验证串口协议栈")
    args = ap.parse_args()
    ROBOT = args.robot

    init_data()
    threading.Thread(target=updater, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", args.port))
    except OSError as e:
        print(f"绑定端口 {args.port} 失败：{e}")
        print("换个端口试试： --port 5020（Modbus Poll 里 Port 也要跟着改）")
        return 1
    srv.listen(8)

    print("=" * 66)
    print(f"Modbus TCP 从站模拟器已启动   模式：{'机器人地址表' if ROBOT else '通用练手'}")
    print(f"  监听      0.0.0.0:{args.port}   从站号(Unit ID) = {args.unit}")
    print(f"  应答模式  {'任意从站号都应答' if args.any_unit else '仅应答本从站号，其他号不理（模拟真实设备）'}")
    print("  Modbus Poll 侧：F3 → TCP/IP → IP 127.0.0.1   Port %d" % args.port)
    print("                  F8 → Slave ID %d  Function 03  Address 0  Quantity 10" % args.unit)
    if ROBOT:
        print("-" * 66)
        print("  机器人从站地址表（--robot 模式）")
        print("    离散量输入  0~4095      FC02 只读    系统状态/标志")
        print("    线圈        4096~8191    FC01/05/15   读写  控制命令")
        print("    输入寄存器  0~32767     FC04 只读    系统参数/变量值")
        print("    保持寄存器  32768~65535 FC03/06/16   读写  系统参数/变量值")
        print("  → FC03 读地址 0 会回异常码 02，和课件里 Illegal Data Address 现象一致")
        print("  → 想看跳动数据：FC03 读地址 32768，或 FC04 读地址 0")
        print("  → 想练写命令：FC05 写线圈地址 4096")
        print("-" * 66)
    print("  Ctrl+C 退出")
    print("=" * 66)

    try:
        while True:
            conn, peer = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=serve_conn,
                             args=(conn, peer, args.unit, args.any_unit, args.frame),
                             daemon=True).start()
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
