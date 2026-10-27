# Standalone GNU Arm Embedded toolchain: make all
PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE = $(PREFIX)size
CMSIS = ../Drivers/CMSIS
DEVICE = ../Drivers/CMSIS/Device/ST/STM32F1xx/Include
CFLAGS = -mcpu=cortex-m3 -mthumb -Os -ffunction-sections -fdata-sections \
         -fno-common -Wall -Wextra -Werror -DSTM32F103xB \
         -I$(CMSIS)/Include -I$(DEVICE)
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostdlib -Wl,--gc-sections \
          -Wl,-Map=bootloader.map -Tbootloader.ld
.PHONY: all clean
all: bootloader.bin
bootloader.elf: startup.s bootloader.c bootloader.ld
	$(CC) $(CFLAGS) $(LDFLAGS) startup.s bootloader.c -o $@
	$(SIZE) $@
bootloader.bin: bootloader.elf
	$(OBJCOPY) -O binary $< $@
clean:
	rm -f bootloader.elf bootloader.bin bootloader.map
