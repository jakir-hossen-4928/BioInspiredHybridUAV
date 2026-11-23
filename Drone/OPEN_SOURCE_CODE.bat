@echo off
cd /d "%~dp0"
where code >nul 2>&1
if %errorlevel% == 0 (code "%~dp0") else (echo VS Code command not in PATH. Open the Drone folder manually in VS Code. & pause)
