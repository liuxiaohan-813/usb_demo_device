
# source and target

# 源文件
SRCS = uart.c usb.c util.c
OBJS = $(SRCS:.c=.o)

CFLAGS  += -nostdlib -Wall -Os -fno-builtin -mcmodel=medany -march=rv32imac -mabi=ilp32

.PHONY: all clean

all: $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS)