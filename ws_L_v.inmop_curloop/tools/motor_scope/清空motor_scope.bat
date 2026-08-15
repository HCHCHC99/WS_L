@echo off
chcp 65001 >nul
title Kill MotorScope
echo.
echo  Killing all motor_scope processes ...
echo.
powershell -NoProfile -Command "$self = $PID; Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'motor_scope\.py' -and $_.ProcessId -ne $self } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"
echo.
echo  Done. All motor_scope processes cleared.
echo.
pause