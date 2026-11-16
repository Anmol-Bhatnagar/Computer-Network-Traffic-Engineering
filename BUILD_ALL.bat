@echo off
title Build All Cross-Domain Simulators
setlocal
cd /d "%~dp0"

echo ======================================================================
echo   COMPILING CROSS-DOMAIN SIMULATOR SUITE
echo ======================================================================

:: Close any running simulator instances to release write lock
taskkill /F /IM CN_VisualSimulator.exe >nul 2>&1
taskkill /F /IM CN_ResearchRunner.exe >nul 2>&1
taskkill /F /IM CN.exe >nul 2>&1

set PATH=C:\msys64\ucrt64\bin;%PATH%
where g++ >nul 2>&1
if errorlevel 1 (
    echo [ERROR] g++ compiler not found in C:\msys64\ucrt64\bin or PATH!
    pause
    exit /b 1
)

echo.
echo [1/3] Compiling Native Win32 Visual Simulator (CN_VisualSimulator.exe)...
g++ -std=c++17 -O2 -mwindows -static -static-libgcc -static-libstdc++ -o CN_VisualSimulator.exe src\CN_VisualSimulator.cpp
if errorlevel 1 ( echo [FAIL] Could not build CN_VisualSimulator.exe & pause & exit /b 1 ) else ( echo [OK] Built CN_VisualSimulator.exe )

echo.
echo [2/3] Compiling Headless Research Engine (CN_ResearchRunner.exe)...
g++ -std=c++17 -O2 -static -static-libgcc -static-libstdc++ -o CN_ResearchRunner.exe src\CN_ResearchRunner.cpp
if errorlevel 1 ( echo [FAIL] Could not build CN_ResearchRunner.exe & pause & exit /b 1 ) else ( echo [OK] Built CN_ResearchRunner.exe )

echo.
echo [3/3] Compiling Console Research Platform (CN.exe)...
g++ -std=c++17 -O2 -static -static-libgcc -static-libstdc++ -o CN.exe src\CN.cpp
if errorlevel 1 ( echo [FAIL] Could not build CN.exe & pause & exit /b 1 ) else ( echo [OK] Built CN.exe )

echo.
echo ======================================================================
echo   ALL TARGETS BUILT SUCCESSFULLY (Statically linked, zero external DLLs)!
echo ======================================================================
pause
