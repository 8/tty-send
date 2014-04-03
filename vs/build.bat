@echo ****************
@echo *** Building ***
@echo ****************
@arm-linux-gnueabihf-g++.exe *.c -o tty-send

@IF %ERRORLEVEL% NEQ 0 EXIT %ERRORLEVEL%

@echo *****************
@echo *** Deploying ***
@echo *****************
@scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no ./tty-send pi@192.168.2.111:~/