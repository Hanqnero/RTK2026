@echo off
REM Launches run_isaac_lab.ps1 with ExecutionPolicy Bypass (avoids "script execution disabled" on Windows).
cd /d "%~dp0\.."
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_isaac_lab.ps1"
pause
