# Name of the output file
NAME = tty-send

SRCS = $(wildcard *.h) $(wildcard *.c)

all: tty-send

tty-send: $(SRCS)
	$(CC)  $^ -lpthread -o $@

clean:
	rm -f tty-send
