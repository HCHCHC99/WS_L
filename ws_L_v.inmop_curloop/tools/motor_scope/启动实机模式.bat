@echo off
chcp 65001 >nul
title MotorScope - J-Link
cd /d "%~dp0"
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