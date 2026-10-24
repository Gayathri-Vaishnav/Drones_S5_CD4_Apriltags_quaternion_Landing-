@echo off
REM ===================================================================
REM   run_matlab.bat  -  the MATLAB build of the landing simulation
REM
REM     run_matlab.bat            3D animation with the onboard camera
REM     run_matlab.bat batch      every address x 4 seeds, results table
REM     run_matlab.bat ablation   what each controller term buys
REM ===================================================================
setlocal
cd /d "%~dp0"
set "ML=matlab"
if exist "C:\Program Files\MATLAB\R2024a\bin\matlab.exe" set "ML=C:\Program Files\MATLAB\R2024a\bin\matlab.exe"
set "WHAT=%~1"
if "%WHAT%"=="" set "WHAT=demo"
echo.
echo   MATLAB build - %WHAT%
echo.
if /i "%WHAT%"=="batch"    "%ML%" -batch "T = run_batch(0:3); writetable(T, 'matlab_results.csv');" & goto :done
if /i "%WHAT%"=="ablation" "%ML%" -batch "run_ablation(0:1);" & goto :done
"%ML%" -nosplash -nodesktop -r "cd('%~dp0'); run_demo3d([6 3 9], 1);"
goto :eof
:done
echo.
pause
endlocal
