@echo *****************
@echo *** Deploying ***
@echo *****************
@scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no ../tty-send pi@192.168.2.119:~/