@echo off
setlocal EnableDelayedExpansion
title FAA Rock Slope Analysis

echo ============================================================
echo  FAA Rock Slope Kinematic Analysis  ^|  GEO TN 4/2024
echo ============================================================
echo.

:: ── Check Python is installed ──────────────────────────────────────────────
python --version >nul 2>&1
if errorlevel 1 (
    echo  ERROR: Python is not installed or not on PATH.
    echo.
    echo  Please download Python 3.11 from:
    echo    https://www.python.org/downloads/
    echo.
    echo  During install, tick "Add Python to PATH"  ^<-- important
    echo  Then run this file again.
    echo.
    pause
    exit /b 1
)

:: ── Check Python version (open3d needs 3.8 - 3.12) ────────────────────────
for /f "tokens=2 delims= " %%v in ('python --version 2^>^&1') do set PYVER=%%v
for /f "tokens=1,2 delims=." %%a in ("!PYVER!") do (
    set PYMAJ=%%a
    set PYMIN=%%b
)

if !PYMAJ! NEQ 3 (
    echo  ERROR: Python 3 is required  ^(found !PYVER!^)
    pause
    exit /b 1
)
if !PYMIN! LSS 8 (
    echo  ERROR: Python 3.8 or newer is required  ^(found !PYVER!^)
    echo  Please install Python 3.11 from python.org
    pause
    exit /b 1
)
if !PYMIN! GTR 12 (
    echo  WARNING: Python 3.13+ detected  ^(!PYVER!^)
    echo  open3d currently supports up to Python 3.12.
    echo  Please install Python 3.11 from python.org for best compatibility.
    echo.
    echo  Press any key to try anyway, or close this window to cancel.
    pause
)

echo  Python !PYVER! found.

:: ── Create virtual environment if needed ───────────────────────────────────
if not exist ".venv\Scripts\activate.bat" (
    echo  Creating virtual environment...
    python -m venv .venv
    if errorlevel 1 (
        echo  ERROR: Could not create virtual environment.
        pause
        exit /b 1
    )
)

:: ── Activate venv ──────────────────────────────────────────────────────────
call .venv\Scripts\activate.bat

:: ── Install / update dependencies ──────────────────────────────────────────
echo  Checking dependencies...
python -c "import open3d, PyQt5, matplotlib, laspy" >nul 2>&1
if errorlevel 1 (
    echo  Installing dependencies  ^(first run only, may take a few minutes^)...
    echo.
    pip install --upgrade pip --quiet
    pip install -r requirements.txt
    if errorlevel 1 (
        echo.
        echo  ERROR: Failed to install dependencies.
        echo  Check your internet connection and try again.
        pause
        exit /b 1
    )
    echo.
    echo  Dependencies installed successfully.
)

:: ── Launch app ─────────────────────────────────────────────────────────────
echo  Launching FAA Rock Slope Analysis...
echo.
python faa_gui.py

:: If the app exits with an error, keep the window open
if errorlevel 1 (
    echo.
    echo  The application exited with an error.
    pause
)

endlocal
