# Modbus Poll Lite

一个自主实现的 Modbus 主站调试工具，功能对标 Modbus Poll。

免安装、免费使用、中文界面，内置机器人控制器 Modbus 从站地址表预设。

---

## 目录结构

```
modbus_poll_lite/
├── src/                            源码（模块互相 import，须保持同目录）
│   ├── modbus_poll_lite_mdi.py     主站 · 多窗口版       ← 主开发入口
│   ├── modbus_poll_lite.py         核心 · 单窗口版（已冻结，请勿修改）
│   ├── modbus_chart.py             实时曲线 + 双游标
│   ├── modbus_serial.py            串口 RTU/ASCII + RTU/ASCII over TCP
│   ├── modbus_testcenter.py        Test Center 手动构造报文
│   └── modbus_tcp_slave.py         从站模拟器
├── tests/                          回归测试（217 项）
│   ├── test_modbus_poll_lite.py    单窗口 56 项 / 多窗口 116 项
│   └── test_modbus_serial.py       串口协议栈 45 项
├── dist/                           打包产物（exe 不纳入版本库，须自行打包）与用户文档
│   ├── modbus_poll_lite_mdi.exe    主站（免 Python 运行）
│   ├── modbus_slave.exe            从站模拟器
│   ├── Modbus工具使用说明.txt
│   └── 更新说明.txt                 本版功能与修复记录
├── docs/
│   └── TODO.md                     开发记录（11 批迭代及验证结果）
├── build/                          PyInstaller 中间产物（可删除）
├── build_exe.sh                    打包脚本
├── start_slave.bat                 启动从站（独立窗口，关闭即停止）
├── stop_slave.bat                  停止从站（可清理后台残留进程）
└── README.md                       本文件
```

> `src/modbus_poll_lite.py` 为**已冻结的旧版核心**，多窗口版复用其
> `ModbusMaster` / 功能码常量 / 异常码表。**请勿删除或修改**——
> 多窗口版出现问题时可直接回退至此版本。

---

## 快速开始

**主站**（两种运行方式，任选其一）：

```bash
# 方式一：直接运行源码（需 Python 3.8+，tkinter 为标准库自带）：
cd src
PYTHONIOENCODING=utf-8 python modbus_poll_lite_mdi.py

# 方式二：打包为免 Python 的 exe（需 PyInstaller，约 80 秒）：
bash build_exe.sh
双击  dist/modbus_poll_lite_mdi.exe
```

**从站模拟器**（无真实设备时用于练习）：

```
双击  start_slave.bat          ← 独立窗口运行，关闭窗口即停止
双击  stop_slave.bat           ← 从站以后台方式启动、无窗口可关闭时，用于清理
```

> ⚠️ **请勿直接双击 `dist/modbus_slave.exe`**：该程序为控制台应用，
> 双击运行时窗口将瞬间退出（进程实际转入后台），无法定位其运行状态。
> 请使用 `start_slave.bat` 启动，或在 cmd 中运行。
> 若进程已在后台运行，可用 `stop_slave.bat` 清理（兼容打包版与源码版两种方式）。

基本操作流程：`F3` 连接 → `F8` 读写定义 → 显示 → 实时曲线

---

## 开发

### 回归测试（源码修改后必须执行）

```bash
cd tests
PYTHONIOENCODING=utf-8 python test_modbus_poll_lite.py          # 单窗口 56 项
PYTHONIOENCODING=utf-8 python test_modbus_poll_lite.py --mdi    # 多窗口 116 项
PYTHONIOENCODING=utf-8 python test_modbus_serial.py             # 串口 45 项
```

从站未运行时，测试会**自动启动临时从站并在测试结束后关闭**，因此可独立执行，
全程十余秒。

### 依赖

- Python 3.8+（开发环境为 3.14）
- **pyserial**（仅串口功能需要）：`python -m pip install pyserial`
- tkinter（Python 标准库自带）

### 源码修改注意事项

| 位置 | 注意事项 |
|---|---|
| `modbus_poll_lite.py` | **已冻结，请勿修改**；核心功能扩展请在 MDI 版中实现 |
| `_row_step()` | 返回 1 或 2（Float/Long 占用两个寄存器）。如需支持 64 位浮点应改为 4，并回归验证所有调用点 |
| `_format_value` / `_raw_number` | 数值显示与条件着色共用，修改一处需同步另一处 |
| 曲线的 `_tick` | 每 80ms 重绘一次。提高刷新率前应评估 CPU 开销：`_redraw` 会重绘整个画布 |
| 抽稀上限 | 按画布宽度动态计算，请勿改回固定值（窗口放大时会产生卡顿） |
| 跨线程操作 tk | 必须通过队列传递至主线程，不可直接调用 `after` |

---

## 打包

```bash
cd modbus_poll_lite
bash build_exe.sh
```

产物位于 `dist/`。**打包前请关闭正在运行的 exe**，否则产物文件会被占用。

### 打包时的两个已知问题（当前版本已规避，请勿回退相关修改）

1. **`--hidden-import` 参数不可删除**
   `modbus_serial.py` 中 `import serial` 位于 `try/except` 内（用于在未安装
   pyserial 时优雅降级），PyInstaller 静态分析无法识别 → 将导致 exe 中串口
   功能**静默失效（无任何报错）**。同理，`modbus_poll_lite` / `modbus_chart` /
   `modbus_testcenter` 因主脚本先执行 `sys.path.insert` 再 import，也可能被遗漏。

2. **`--windowed` 模式下启动失败将无提示退出**
   该模式无控制台输出，错误信息不可见。`main()` 中已捕获异常并以对话框形式呈现。

### 打包后自检

```bash
dist/modbus_poll_lite_mdi.exe --selftest
```

以对话框形式列出各模块状态（含串口库），并将报告写入 `%TEMP%\modbus_selftest.txt`。
`--selftest --quiet` 仅写入文件、不弹窗，供脚本调用。

---

## 文档

| 文件 | 内容 |
|---|---|
| `dist/Modbus工具使用说明.txt` | 操作说明，分发时请一并拷贝 |
| `dist/更新说明.txt` | 本版功能清单、修复记录与已知限制 |
| `docs/TODO.md` | 开发记录（11 批迭代、验证结果与待办事项） |

机器人控制器 Modbus 从站四段地址表已内置（工具栏提供四个按钮一键填入），
地址口径可对照机器人厂商的通信手册。

---

## 与 Modbus Poll 的功能对比

**已实现**：多窗口、串口 RTU/ASCII、RTU/ASCII over TCP、实时曲线、
Test Center、掩码写 (22)、脉冲写（点动）、设备扫描、字节序 4 档、暂停轮询、
RTS toggle、地址扫描、缩放、条件着色、数据记录、报文监视、配置保存

**尚未实现**：
- UDP 系列（3 种）
- 显示格式 6 种 vs 28 种（缺 64 位浮点、字符串、日期时间）
- 功能码 23 / 43 的原生支持（可通过 Test Center 手动构造）
- 记录直接写 Excel、OLE 自动化、打印

**已知局限**：串口的**真实 COM 口收发尚未经硬件验证**。
帧格式、CRC/LRC、帧分割及主站逻辑均已测试，尚未覆盖 pyserial
驱动程序层的实际串口读写。

---

## 安全须知

⚠️ **连接真实设备时，写寄存器会立即引起设备动作。**

- 写入前确认目标地址对应的设备及其动作后果
- 确认机械危险区域内无人，急停按钮处于可触及位置
- 对不确定的地址，应遵循先读后写的原则
- 请勿对广播地址（从站号 0）进行试探性操作

---

## 许可证

[MIT License](LICENSE) —— 允许免费使用、修改及商业应用，仅需保留版权声明。
