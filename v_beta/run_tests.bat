@echo off
REM Script to activate virtual environment and run tests with verbose output

echo Activating virtual environment...
call venv\Scripts\activate.bat

echo Python version:
python --version

echo.
echo Running pytest with verbose output...
python -m pytest tests/ -v -s

pause
