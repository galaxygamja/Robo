@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"
set "ROBO_EXIT=1"

where py >nul 2>nul
if not errorlevel 1 (
  py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>nul
  if not errorlevel 1 (
    py -3 -m robo_control %*
    set "ROBO_EXIT=!errorlevel!"
    goto :done
  )
)

where python >nul 2>nul
if not errorlevel 1 (
  python -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>nul
  if not errorlevel 1 (
    python -m robo_control %*
    set "ROBO_EXIT=!errorlevel!"
    goto :done
  )
)

echo Python 3.11 or newer is required.
echo Install Python from https://www.python.org/downloads/ and try again.
if /I not "%CI%"=="true" pause

:done
endlocal & exit /b %ROBO_EXIT%
