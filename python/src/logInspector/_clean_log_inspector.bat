@echo off

echo Clean Log Inspector
echo.

rd /s /q build
del /q log_reader.cp37-win32.pyd

timeout 5 /nobreak