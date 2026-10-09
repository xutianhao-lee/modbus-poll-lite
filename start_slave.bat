@echo off
chcp 65001 >nul
title Modbus 从站模拟器 - 关闭本窗口即停止
cd /d "%~dp0"

echo ============================================================
echo   Modbus 从站模拟器
echo ============================================================
echo   地址   127.0.0.1 : 502      从站号 1
echo   模式   机器人控制器四段地址表（--robot）
echo.
echo   主站连这个地址就能练手。
echo   要停止：直接关掉本窗口，或按 Ctrl+C
echo ============================================================
echo.

if not exist "%~dp0dist\modbus_slave.exe" (
    echo [错误] 找不到 dist\modbus_slave.exe
    echo        先跑一次 build_exe.sh 打包，或者改用源码方式：
    echo        python src\modbus_tcp_slave.py --robot
    echo.
    pause
    exit /b 1
)

"%~dp0dist\modbus_slave.exe" --robot --port 502

echo.
echo 从站已退出。
pause
