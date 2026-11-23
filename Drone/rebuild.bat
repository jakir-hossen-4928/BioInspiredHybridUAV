@echo off
cd /d "%~dp0"
where py >nul 2>&1
if %ERRORLEVEL% EQU 0 (py -3 tools\create_worlds.py) else (python tools\create_worlds.py)
if ERRORLEVEL 1 echo World build failed; previous files were restored
pause
