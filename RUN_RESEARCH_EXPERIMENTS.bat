@echo off
title Cross-Domain Research Runner
cd /d "%~dp0"
echo ======================================================================
echo   Running Automated Batch Research Engine (24 Scenarios, 48,000 Ticks)...
echo   Results will be saved to research_logs/
echo ======================================================================
"%~dp0CN_ResearchRunner.exe"
echo.
echo ======================================================================
echo   Batch Execution Complete! Logs updated in research_logs/
echo   - raw_ticks.csv
echo   - summary.csv
echo   - summary.json
echo   - hypothesis_eval.txt
echo ======================================================================
pause
