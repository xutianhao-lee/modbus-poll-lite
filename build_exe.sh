#!/usr/bin/env bash
# 把 Modbus 工具集打包成单文件 exe（免 Python 运行）。
#
# 用法：bash build_exe.sh               在项目根目录下执行
# 产物：dist/modbus_poll_lite_mdi.exe   主站（GUI，多窗口）
#       dist/modbus_slave.exe          从站模拟器（控制台）
#
# 注意：exe 正在运行 / 被杀软扫描时会锁住文件，先关掉再打包。

set -e
cd "$(dirname "$0")"          # 脚本就在项目根目录
ROOT="$(pwd)"
SRC="$ROOT/src"

echo "项目目录：$ROOT"
echo "清理旧产物…"
rm -f "$ROOT/dist/modbus_poll_lite_mdi.exe" "$ROOT/dist/modbus_slave.exe" 2>/dev/null || true

# ⚠️ 这几个必须显式声明，否则会**静默失效**（不报错，功能就是不工作）：
#
#   modbus_poll_lite / modbus_chart / modbus_serial / modbus_testcenter
#     主脚本先 sys.path.insert 再 import，PyInstaller 的静态分析可能漏掉
#
#   serial / serial.tools.list_ports
#     源码里包在 try/except ImportError 内（为了没装 pyserial 时优雅降级），
#     不声明的话 exe 里串口会打不开，而且**不报错**
#
#   sv_ttk（用 --collect-data 收集，见下方命令）
#     Windows 11 风格主题的数据文件在 sv_ttk/theme/，
#     不打进去的话 exe 里主题静默失效（回退默认外观，同样不报错）
HIDDEN=(
  --hidden-import modbus_poll_lite
  --hidden-import modbus_chart
  --hidden-import modbus_serial
  --hidden-import modbus_testcenter
  --hidden-import serial
  --hidden-import serial.tools.list_ports
)

echo ""
echo "[1/2] 打包主站（GUI，约 40 秒）…"
PYTHONIOENCODING=utf-8 python -m PyInstaller \
  --noconfirm --clean --onefile --windowed \
  --name modbus_poll_lite_mdi \
  --distpath "$ROOT/dist" \
  --workpath "$ROOT/build" \
  --specpath "$ROOT/build" \
  --paths "$SRC" \
  --exclude-module numpy \
  --exclude-module pytest \
  --exclude-module matplotlib \
  "${HIDDEN[@]}" \
  --collect-data sv_ttk \
  "$SRC/modbus_poll_lite_mdi.py" 2>&1 | tail -3

echo ""
echo "[2/2] 打包从站模拟器（控制台，约 40 秒）…"
PYTHONIOENCODING=utf-8 python -m PyInstaller \
  --noconfirm --clean --onefile --console \
  --name modbus_slave \
  --distpath "$ROOT/dist" \
  --workpath "$ROOT/build" \
  --specpath "$ROOT/build" \
  --paths "$SRC" \
  --exclude-module numpy \
  --exclude-module pytest \
  --exclude-module matplotlib \
  "$SRC/modbus_tcp_slave.py" 2>&1 | tail -3

echo ""
echo "完成："
ls -la "$ROOT/dist/"modbus*.exe 2>/dev/null || echo "（没找到产物，检查上面的报错）"
echo ""
echo "自检（确认依赖都在）："
echo "  $ROOT/dist/modbus_poll_lite_mdi.exe --selftest"
