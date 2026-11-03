@echo off
setlocal
cd /d "%~dp0"
set "PYTHON=.venv\Scripts\python.exe"
if not exist "%PYTHON%" set "PYTHON=C:\Users\zhangyuxuan\Downloads\uart_gimbal_imu\uart_gimbal_imu\.venv\Scripts\python.exe"
if not exist "%PYTHON%" (
    echo Python environment with PySide6 was not found.
    echo Create .venv and install pc\requirements.txt first.
    pause
    exit /b 1
)
"%PYTHON%" -X utf8 -B app.py %*
if errorlevel 1 pause
