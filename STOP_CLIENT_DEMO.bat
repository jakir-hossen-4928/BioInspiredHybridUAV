@echo off
cd /d "%~dp0"
where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%~dp0Drone\tools\client_demo.py" --stop
) else (
    python "%~dp0Drone\tools\client_demo.py" --stop
)
exit /b %ERRORLEVEL%
