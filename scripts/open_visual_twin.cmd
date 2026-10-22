@echo off
setlocal
cd /d "%~dp0.."
if not exist "runtimes\isaac-env\Scripts\python.exe" (
  echo Isaac runtime missing. See documentation\visual-twin.md.
  pause
  exit /b 1
)
if "%~1"=="" (
  echo Usage: scripts\open_visual_twin.cmd "path\to\world.usda"
  pause
  exit /b 1
)
"runtimes\isaac-env\Scripts\python.exe" "scripts\view_visual_twin.py" --scene "%~1"
if errorlevel 1 pause
