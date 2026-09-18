@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 (
  echo Python 3.11 or newer is required. Install Python with the Windows launcher.
  exit /b 1
)
if not exist ".venv\Scripts\python.exe" (
  py -3 -m venv .venv
  if errorlevel 1 exit /b 1
)
".venv\Scripts\python.exe" -m pip install -e ".[vision]" platformio==6.1.18
if errorlevel 1 exit /b 1
if not exist "local_state\hardware.json" (
  ".venv\Scripts\python.exe" -m robo_control.hardware init --output local_state/hardware.json
  if errorlevel 1 exit /b 1
)
echo Setup complete. No robot was contacted or flashed.
echo Read docs\HARDWARE_START_KO.md before enabling outputs.
endlocal
