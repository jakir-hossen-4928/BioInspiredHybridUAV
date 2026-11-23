@echo off
cd /d "%~dp0"
where py >nul 2>&1
if %ERRORLEVEL% EQU 0 (py -3 tools\add_wheels_worlds.py) else (python tools\add_wheels_worlds.py)
if ERRORLEVEL 1 echo Wheel-world build failed.
pause
