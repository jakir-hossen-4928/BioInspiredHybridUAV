@echo off
cd /d "%~dp0"
if not exist dashboard.html (echo Start the Webots simulation first
pause
exit /b 1)
start "" "%~dp0dashboard.html"
