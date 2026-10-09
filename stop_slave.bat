@echo off
chcp 65001 >nul
title 停止 Modbus 从站

echo ============================================================
echo   停止 Modbus 从站模拟器
echo ============================================================
echo.

echo [1/3] 停止打包版从站（modbus_slave.exe）...
taskkill /F /IM modbus_slave.exe >nul 2>&1
if errorlevel 1 (echo       没有在运行) else (echo       已停止)

echo.
echo [2/3] 停止源码版从站（python ... modbus_tcp_slave.py）...
powershell -NoProfile -Command "$p = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*modbus_tcp_slave*' }; if ($p) { $p | ForEach-Object { Stop-Process -Id $_.ProcessId -Force; Write-Host ('       已停止 PID ' + $_.ProcessId) } } else { Write-Host '       没有在运行' }"

echo.
echo [3/3] 检查 502 端口...
netstat -ano | findstr ":502 " | findstr LISTENING >nul 2>&1
if errorlevel 1 (
    echo       [完成] 502 已释放，从站停干净了
) else (
    echo       [注意] 502 仍被占用，下面是占用它的进程：
    netstat -ano | findstr ":502 " | findstr LISTENING
)

echo.
pause
