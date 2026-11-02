@echo off
setlocal
cd /d "%~dp0.."
if not defined HIVEMIND_ISAAC_PYTHON if exist "runtimes\isaac-python.txt" set /p HIVEMIND_ISAAC_PYTHON=<"runtimes\isaac-python.txt"
if not defined HIVEMIND_ISAAC_PYTHON set "HIVEMIND_ISAAC_PYTHON=%CD%\runtimes\isaac-env\Scripts\python.exe"
if not exist "%HIVEMIND_ISAAC_PYTHON%" (
  echo Isaac runtime missing. See documentation\visual-twin.md.
  pause
  exit /b 1
)
if "%~1"=="" (
  echo Usage: scripts\open_visual_twin.cmd "path\to\world.usda"
  pause
  exit /b 1
)
"%HIVEMIND_ISAAC_PYTHON%" "scripts\view_visual_twin.py" --scene "%~1"
if errorlevel 1 pause
