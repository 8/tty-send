@echo ****************
@echo *** Building ***
@echo ****************
@C:\SysGCC\Raspberry\bin\arm-linux-gnueabihf-g++.exe ..\*.c -lpthread -o ..\tty-send

@IF %ERRORLEVEL% NEQ 0 EXIT %ERRORLEVEL%
