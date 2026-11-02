@echo off
setlocal
"%~dp0Runtime\python.exe" -E -s -B "%~dp0Tests\self_test.py"
if errorlevel 1 (
    pause
    exit /b 1
)
"%~dp0Runtime\python.exe" -E -s -B "%~dp0Tests\test_gimbal_view.py"
if errorlevel 1 (
    pause
    exit /b 1
)
"%~dp0Runtime\python.exe" -E -s -B "%~dp0Tests\test_gimbal_mesh.py"
if errorlevel 1 (
    pause
    exit /b 1
)
"%~dp0Runtime\python.exe" -E -s -B "%~dp0Tests\test_css_console.py"
if errorlevel 1 pause
