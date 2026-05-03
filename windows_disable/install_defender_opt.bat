@echo off
SETLOCAL ENABLEEXTENSIONS

echo Registering Defender optimization task...
schtasks /Create /TN "DefenderOptimization" /XML DefenderOptTask.xml /F
if %errorlevel% neq 0 (
    echo Failed to register Defender optimization task.
    exit /b %errorlevel%
)