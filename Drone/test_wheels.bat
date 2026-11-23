@echo off
cd /d "%~dp0"
where py >nul 2>&1
if %ERRORLEVEL% EQU 0 (py -3 -m unittest discover -s tests -p test_wheels.py -v) else (python -m unittest discover -s tests -p test_wheels.py -v)
pause
