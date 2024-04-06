# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump


AFLAGS = -static -nostartfiles -mlittle-endian -nostdlib -Wa,-EL
CFLAGS = -c -mthumb -nostdlib -nostartfiles -Os -g -fPIC

all: air001_erase.inc air001_write_async.inc air001_write_sync.inc	\
 	 air105.inc air105_write.inc

.PHONY: clean

%.elf: %.S
	$(CC) $(AFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc
