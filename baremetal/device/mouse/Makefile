# submodle path
TINYPRINTF 	:= ../../Components/tinyprintf
DRIVERS	:= ../../drivers/

# source and target
SRCS    := main.c
OBJS    := $(SRCS:.c=.o)
OBJS    := $(OBJS:.s=.o)
TARGET  := main.elf

# compile and link args
CFLAGS  += -nostdlib -static -Wall -Os -fno-builtin -mcmodel=medany -march=rv32imac -mabi=ilp32
LDFLAGS += -nostdlib -static -m elf32lriscv -e main

.PHONY: all clean

all: $(TARGET)

# rules of compile
%.o: %.c
	$(MAKE) -C $(TINYPRINTF) CFLAGS="$(filter-out -Wextra,$(CFLAGS))" compiletests
	$(MAKE) -C $(DRIVERS)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

# rules of link
$(TARGET): $(OBJS) 
	$(LD) $(LDFLAGS) -o $@ $^ $(TINYPRINTF)/tinyprintf_only_tfp_printf.o $(DRIVERS)/*.o
	$(SIZE) $@

# rules of clean
clean:
	$(MAKE) -C $(TINYPRINTF) clean
	$(MAKE) -C $(DRIVERS) clean
	rm -f $(OBJS) $(TARGET)