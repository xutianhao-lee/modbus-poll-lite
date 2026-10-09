# -*- coding: utf-8 -*-
"""
modbus_serial.py —— 串口 RTU / ASCII 支持（Modbus Poll Lite 家族）

刻意拆成两层，好测试：

    ① RtuCodec / AsciiCodec  —— 帧的组装与解析，**与传输无关**
       → 可以架在 TCP 上做端到端测试（即 "RTU over TCP"），覆盖 CRC、分帧、异常帧
    ② SerialTransport        —— pyserial 的字节读写
       → 这一薄层需要真实串口才能真正验证，没有硬件时只能保证接口正确

ModbusSerialMaster 的公开接口与 modbus_poll_lite.ModbusMaster 一致
（connect/close/read/write_*/tx/err/connected/peer/traffic/on_traffic/unit），
可以直接替换而不改上层代码。

依赖：pyserial（`python -m pip install pyserial`）
"""

import socket
import struct
import threading
import time

try:
    import serial
except ImportError:                     # pragma: no cover
    serial = None


# ---------------------------------------------------------------- 常量

BAUD_CHOICES = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]
PARITY_CHOICES = [("无校验 None (N)", "N"), ("偶校验 Even (E)", "E"), ("奇校验 Odd (O)", "O")]
BYTE_CHOICES = [("8 位", 8), ("7 位", 7)]
STOP_CHOICES = [("1 位", 1), ("2 位", 2)]

# 字符间隔（秒/字符）：RTU 用 3.5 字符间隔分帧
def char_time(baud, bits=11):
    return bits / float(baud)


def available_ports():
    """列出可用串口 [(设备名, 描述), ...]"""
    try:
        import serial.tools.list_ports as lp
        return [(p.device, p.description or "") for p in lp.comports()]
    except Exception:
        return []


# ---------------------------------------------------------------- CRC / LRC

def crc16(data):
    """Modbus RTU 的 CRC16（多项式 0xA001，初值 0xFFFF）
    标准向量：b"123456789" -> 0x4B37"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def lrc(data):
    """Modbus ASCII 的 LRC：所有字节求和取两补"""
    return (-sum(data)) & 0xFF


# ---------------------------------------------------------------- 帧编解码

class RtuCodec:
    """RTU 帧： <unit><pdu><crc_lo><crc_hi>"""

    name = "RTU"

    @staticmethod
    def build(unit, pdu):
        body = bytes([unit & 0xFF]) + bytes(pdu)
        c = crc16(body)
        return body + bytes([c & 0xFF, (c >> 8) & 0xFF])

    @staticmethod
    def parse(frame):
        """返回 (unit, pdu)；帧非法抛 ValueError"""
        if len(frame) < 4:
            raise ValueError(f"RTU 帧太短（{len(frame)} 字节）")
        body, got = frame[:-2], frame[-2:]
        want = crc16(body)
        if bytes([want & 0xFF, (want >> 8) & 0xFF]) != got:
            raise ValueError(
                f"RTU CRC 校验失败：收到 {got.hex(' ')}，应为 "
                f"{(want & 0xFF):02X} {(want >> 8):02X}")
        return body[0], body[1:]

    @staticmethod
    def frame_length(head):
        """已读到前 2 字节（unit, fc），推算整帧还差多少字节；None = 还需再读 1 字节"""
        fc = head[1]
        if fc & 0x80:
            return 5
        if fc in (1, 2, 3, 4):
            return None                      # 要看第 3 字节 byte_count
        return 8                             # 05/06/15/16 定长


class AsciiCodec:
    """ASCII 帧： ':' + 十六进制 + CRLF，内容为 <unit><pdu><lrc>"""

    name = "ASCII"

    @staticmethod
    def build(unit, pdu):
        body = bytes([unit & 0xFF]) + bytes(pdu)
        return b":" + (body + bytes([lrc(body)])).hex().upper().encode() + b"\r\n"

    @staticmethod
    def parse(frame):
        if not frame.startswith(b":") or not frame.endswith(b"\r\n"):
            raise ValueError("ASCII 帧格式不对（应以 ':' 开头、CRLF 结尾）")
        hexpart = frame[1:-2].decode("ascii", "replace").strip()
        if len(hexpart) % 2:
            raise ValueError("ASCII 帧十六进制长度为奇数")
        try:
            body = bytes.fromhex(hexpart)
        except ValueError:
            raise ValueError(f"ASCII 帧含非法十六进制字符：{hexpart[:40]}")
        if len(body) < 2:
            raise ValueError("ASCII 帧太短")
        payload, got = body[:-1], body[-1]
        want = lrc(payload)
        if want != got:
            raise ValueError(f"ASCII LRC 校验失败：收到 {got:02X}，应为 {want:02X}")
        return payload[0], payload[1:]


# ---------------------------------------------------------------- 传输层

class SerialTransport:
    """pyserial 包装：只负责字节进出，不管协议"""

    def __init__(self, port, baud=9600, bytesize=8, parity="N", stopbits=1,
                 timeout=1.0, rts_toggle=False):
        if serial is None:
            raise RuntimeError(
                "没装 pyserial。请先运行：\n    python -m pip install pyserial")
        # 用 serial_for_url 而不是 serial.Serial：
        #   ① 传普通设备名（COM3）行为完全一样
        #   ② 但可以用 pyserial 的 URL 形式，其中两种很有用：
        #        loop://           回环，用来在没有串口硬件时验证本模块
        #        socket://主机:端口 把 TCP 当串口用（某些串口服务器就是这样）
        self.ser = serial.serial_for_url(
            port, baudrate=int(baud), bytesize=int(bytesize),
            parity={"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN,
                    "O": serial.PARITY_ODD}.get(parity.upper(), serial.PARITY_NONE),
            stopbits=serial.STOPBITS_TWO if int(stopbits) == 2 else serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout)
        self.rts_toggle = rts_toggle
        self.description = f"{port} {baud}-{bytesize}{parity}{stopbits}"

    def sendall(self, data):
        if self.rts_toggle:
            self.ser.rts = True
        self.ser.write(data)
        self.ser.flush()
        if self.rts_toggle:
            self.ser.rts = False

    def read_exact(self, n):
        """读满 n 字节；超时读不满返回已读到的（可能短）"""
        buf = b""
        deadline = time.time() + (self.ser.timeout or 1.0) + 0.05
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
            elif time.time() > deadline:
                break
        return buf

    def read_until(self, terminator, limit=512):
        buf = b""
        deadline = time.time() + (self.ser.timeout or 1.0) + 0.05
        while len(buf) < limit:
            chunk = self.ser.read(1)
            if chunk:
                buf += chunk
                if buf.endswith(terminator):
                    break
            elif time.time() > deadline:
                break
        return buf

    def read_available(self, timeout=1.0):
        """在 timeout 内尽量收；一旦收到数据，再多等一小会儿就收工。

        用于 Test Center 的"原始字节"模式 —— 应收多少事先不知道，
        不能像 read_exact 那样死等固定字节数。
        """
        buf = b""
        old = self.ser.timeout
        self.ser.timeout = 0.05
        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                chunk = self.ser.read(4096)
                if chunk:
                    buf += chunk
                    deadline = time.time() + 0.15
        finally:
            self.ser.timeout = old
        return buf

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SocketTransport:
    """把 TCP socket 当字节流用。

    两个用途：
      · 实现 **RTU / ASCII over TCP** 协议变体（Modbus Poll 支持，我们之前也缺）
      · 让串口的帧格式层可以架在 TCP 上做端到端测试，不必接真串口
    """

    def __init__(self, ip, port, timeout=1.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((ip, port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout
        self.description = f"{ip}:{port} (over TCP)"

    def sendall(self, data):
        self.sock.sendall(data)

    def read_exact(self, n):
        buf = b""
        deadline = time.time() + self.timeout + 0.05
        while len(buf) < n:
            try:
                chunk = self.sock.recv(n - len(buf))
            except (socket.timeout, TimeoutError):
                break
            if not chunk:
                break
            buf += chunk
        return buf

    def read_until(self, terminator, limit=512):
        buf = b""
        deadline = time.time() + self.timeout + 0.05
        while len(buf) < limit and time.time() < deadline:
            try:
                chunk = self.sock.recv(1)
            except (socket.timeout, TimeoutError):
                break
            if not chunk:
                break
            buf += chunk
            if buf.endswith(terminator):
                break
        return buf

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------- 主站

class ModbusSerialMaster:
    """串口 RTU / ASCII 主站，公开接口与 modbus_poll_lite.ModbusMaster 一致"""

    def __init__(self, mode="RTU"):
        self.mode = mode.upper()
        self.codec = RtuCodec if self.mode == "RTU" else AsciiCodec
        self.transport = None
        self.lock = threading.Lock()
        self.unit = 1
        self.tx = 0
        self.err = 0
        self.connected = False
        self.peer = ""
        self.last_error = ""
        self.traffic = []
        self.on_traffic = None

    # -- 连接 ------------------------------------------------------------
    def connect(self, port, baud=9600, bytesize=8, parity="N", stopbits=1,
                timeout=1.0, rts_toggle=False):
        self.close()
        self.transport = SerialTransport(port, baud, bytesize, parity, stopbits,
                                         timeout, rts_toggle)
        self.connected = True
        self.peer = self.transport.description
        self.tx = self.err = 0
        self.last_error = ""

    def connect_transport(self, transport, peer=""):
        """用任意传输层接管（用于测试，或 RTU/ASCII over TCP）"""
        self.close()
        self.transport = transport
        self.connected = True
        self.peer = peer or getattr(transport, "description", "transport")
        self.tx = self.err = 0
        self.last_error = ""

    def connect_tcp(self, ip, port, timeout=1.0):
        """RTU / ASCII over TCP —— 帧格式同串口，只是字节走网线"""
        self.connect_transport(SocketTransport(ip, port, timeout))

    def close(self):
        if self.transport:
            self.transport.close()
        self.transport = None
        self.connected = False

    # -- 收发 ------------------------------------------------------------
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

    def _recv_frame(self):
        """按功能码推算长度，读满一整帧（RTU）/ 读到 CRLF（ASCII）"""
        if self.mode == "ASCII":
            return self.transport.read_until(b"\r\n")

        head = self.transport.read_exact(2)
        if len(head) < 2:
            raise TimeoutError(f"只收到 {len(head)} 字节，从站无响应")
        need = self.codec.frame_length(head)
        if need is None:
            bc = self.transport.read_exact(1)
            if len(bc) < 1:
                raise TimeoutError("读 byte_count 超时")
            head += bc
            need = 3 + bc[0] + 2
        frame = head + self.transport.read_exact(need - len(head))
        if len(frame) < need:
            raise TimeoutError(f"帧不完整：收到 {len(frame)}/{need} 字节")
        return frame

    def request(self, pdu):
        """发一帧，返回响应 PDU。异常响应抛 ModbusError；帧错抛 ValueError"""
        import modbus_poll_lite as core          # 复用它的异常类，保证 except 能抓到

        with self.lock:
            if not self.transport:
                raise ConnectionError("未连接")
            if not 0 <= self.unit <= 255:
                raise ValueError(f"从站号 {self.unit} 超出范围（0 ~ 255）")

            frame = self.codec.build(self.unit, pdu)
            self.tx += 1
            self._log("Tx", frame)
            try:
                self.transport.sendall(frame)
            except Exception as e:
                self.err += 1
                self.last_error = f"发送失败：{e}"
                raise

            if self.unit == 0:                   # 广播：不应答
                self.last_error = ""
                return b""

            try:
                resp = self._recv_frame()
            except Exception as e:
                self.err += 1
                self.last_error = str(e)
                raise
            self._log("Rx", resp)

            try:
                _unit, rpdu = self.codec.parse(resp)
            except ValueError as e:
                self.err += 1
                self.last_error = str(e)
                raise

            if rpdu and (rpdu[0] & 0x80):
                code = rpdu[1] if len(rpdu) > 1 else 0
                self.err += 1
                self.last_error = core.exc_text(code)
                raise core.ModbusError(code)
            self.last_error = ""
            return rpdu

    # -- 功能码（与 core 完全一致）--------------------------------------
    def read(self, fc, addr, qty):
        resp = self.request(struct.pack(">BHH", fc, addr, qty))
        if fc in (1, 2):
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


def make_master(conn_type, **kw):
    """按连接类型造主站。conn_type ∈ {'TCP','RTU','ASCII'}"""
    if conn_type.upper() == "TCP":
        import modbus_poll_lite as core
        return core.ModbusMaster()
    return ModbusSerialMaster(mode=conn_type)
