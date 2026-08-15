@echo off
chcp 65001 >nul
title MotorScope - J-Link
cd /d "%~dp0"

echo.
echo  Clearing old motor_scope processes ...
powershell -NoProfile -Command "$self = $PID; Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'motor_scope\.py' -and $_.ProcessId -ne $self } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
timeout /t 3 /nobreak >nul
echo  Done. Starting a fresh instance ...

echo.
echo ================================================
echo   MotorScope - J-Link RTT  (real hardware mode)
echo   Device : HC32F460    Interface : SWD
echo   Speed  : 1000 kHz    Channel : 0
echo   Close this window to exit
echo ================================================
echo.
py -3 motor_scope.py --mode jlink --device HC32F460 --speed-khz 1000
echo.
echo Exited.
pause