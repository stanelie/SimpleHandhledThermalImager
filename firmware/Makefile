CROSS  ?= arm-none-eabi-
CFLAGS  = -mcpu=cortex-m3 -mthumb -O2 -ffreestanding -nostdlib -nostartfiles
OOCD    = openocd -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg

all: firmware.elf

# main.c textually includes cal.h and gfx.h
firmware.elf: main.c cal.h gfx.h startup.c link.ld
	$(CROSS)gcc $(CFLAGS) -T link.ld -o $@ startup.c main.c -lgcc
	$(CROSS)size $@

firmware.bin: firmware.elf
	$(CROSS)objcopy -O binary $< $@

flash: firmware.elf
	$(OOCD) -c "init" -c "reset halt" -c "program firmware.elf verify" -c "reset run" -c "shutdown"

# read the dbg[] telemetry array out of a running target
debug: firmware.elf
	$(OOCD) -c "init" -c "halt" \
	  -c "mdw $$($(CROSS)nm firmware.elf | awk '/ [bB] dbg$$/{print "0x"$$1}') 24" \
	  -c "resume" -c "shutdown"

clean:
	rm -f firmware.elf firmware.bin

.PHONY: all flash debug clean

# sensor characterisation build: raw frames only, streaming per-pixel statistics
# override the sensor config, e.g.  make diag DIAGFLAGS='-DDIAG_RES=2 -DDIAG_RATE=6'
diag.elf: main.c cal.h gfx.h startup.c link.ld
	$(CROSS)gcc $(CFLAGS) -DDIAG $(DIAGFLAGS) -T link.ld -o $@ startup.c main.c -lgcc
	$(CROSS)size $@

diag: diag.elf
	$(OOCD) -c "init" -c "reset halt" -c "program diag.elf verify" -c "reset run" -c "shutdown"
