@echo off
setlocal
cd /d "%~dp0"
start "" "%~dp0Runtime\pythonw.exe" -E -s -B "%~dp0App\desktop_server.py"
