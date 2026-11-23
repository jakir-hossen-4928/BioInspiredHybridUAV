@echo off
cd /d "%~dp0"
python -m unittest discover -s tests -p test_hybrid.py -v
pause
