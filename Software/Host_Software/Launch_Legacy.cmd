@echo off
setlocal
cd /d "%~dp0"
"%~dp0Runtime\python.exe" -E -s -B "%~dp0App\demo_gui.py"
if errorlevel 1 pause
