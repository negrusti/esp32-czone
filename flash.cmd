@echo off
setlocal

set "UPLOAD_PORT=%~1"
if "%UPLOAD_PORT%"=="" set "UPLOAD_PORT=COM7"

echo Flashing ESP32-CZone on %UPLOAD_PORT%
pio run -t upload --upload-port %UPLOAD_PORT%
if errorlevel 1 exit /b %ERRORLEVEL%

echo Resetting ESP32-CZone into application on %UPLOAD_PORT%
pio pkg exec -p tool-esptoolpy -- esptool.py --port %UPLOAD_PORT% --after watchdog_reset run
exit /b %ERRORLEVEL%
