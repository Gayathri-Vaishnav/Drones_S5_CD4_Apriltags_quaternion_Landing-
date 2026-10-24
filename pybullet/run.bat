@echo off
setlocal
cd /d "%~dp0"
set "PY=.venv\Scripts\python.exe"
if /i "%~1"=="install" goto install
if not exist "%PY%" goto setup
"%PY%" -c "import cv2,numpy,pandas,pybullet,pupil_apriltags" >nul 2>nul
if errorlevel 1 goto setup
goto dispatch

:setup
call setup.bat
if errorlevel 1 exit /b 1
if /i "%~1"=="install" exit /b 0
goto dispatch

:dispatch
if /i "%~1"=="check" goto check
if /i "%~1"=="doctor" goto doctor
if /i "%~1"=="determinism" goto determinism
if /i "%~1"=="analyse" goto analyse
if /i "%~1"=="analyze" goto analyse
if /i "%~1"=="evaluate" goto evaluate
if /i "%~1"=="world" goto world
if /i "%~1"=="record" goto record
if /i "%~1"=="plots" goto plots
if /i "%~1"=="paper" goto paper
if "%~1"=="" goto dashboard
"%PY%" demo.py %*
exit /b %errorlevel%
:dashboard
"%PY%" panel.py
:install
call setup.bat
exit /b %errorlevel%
:check
"%PY%" selftest.py
exit /b %errorlevel%
:doctor
"%PY%" doctor.py %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%
:determinism
"%PY%" determinism_test.py
exit /b %errorlevel%
:analyse
"%PY%" analysis.py
exit /b %errorlevel%
:evaluate
"%PY%" evaluate.py
exit /b %errorlevel%
:world
"%PY%" world\build_world.py
exit /b %errorlevel%
:record
"%PY%" record.py %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%
:plots
"%PY%" run_with_plots.py %2 %3 %4 %5 %6 %7 %8 %9
exit /b %errorlevel%
:paper
"%PY%" demo.py %2 %3 %4 %5 %6 %7 %8 %9 --paper
exit /b %errorlevel%
