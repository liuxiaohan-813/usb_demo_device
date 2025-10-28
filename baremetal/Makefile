# toolchain
CROSS_COMPILE ?= riscv-nuclei-elf-
CC      := $(CROSS_COMPILE)gcc
LD      := $(CROSS_COMPILE)ld
OBJDUMP := $(CROSS_COMPILE)objdump
OBJCOPY := $(CROSS_COMPILE)objcopy
GDB     := $(CROSS_COMPILE)gdb
SIZE    := $(CROSS_COMPILE)size

export CC LD OBJDUMP OBJCOPY GDB SIZE


# dir
TARGET		  ?= mouse
FW_NEXT		  := device/$(TARGET)

# build firmware
firmware: $(FW_NEXT)
	$(MAKE) -C $(FW_NEXT) all

.PHONY: all clean

all: firmware

clean:
	$(MAKE) -C $(FW_NEXT) clean