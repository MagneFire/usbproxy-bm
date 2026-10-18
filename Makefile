# usbproxy-bm: bare-metal usb-proxy firmware for the Orange Pi Zero (H2+/H3).
# Loaded by boot0 (src/boot0: the uImage at SD sector 0x18800) or over FEL
# (tools/flash.py). See README.md.
CROSS     ?= arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
OBJDUMP   := $(CROSS)objdump
SIZE      := $(CROSS)size
MKIMAGE   ?= mkimage
# FEL loader/flasher (needs sunxi-fel, SUNXI_FEL= to override; uv fetches pyserial)
FLASH     ?= uv run tools/flash.py

BUILD   := build
TUSB    := third_party/tinyusb/src
GITREV  := $(shell git rev-parse --short=10 HEAD 2>/dev/null || echo nogit)
DIRTY   := $(shell git diff --quiet HEAD 2>/dev/null || echo +)
LOAD    := 0x40008000
# 0x18800, src/boot0/boot0.h FW_SECTOR
KSECTOR := 100352

SRCS := src/start.S src/h3/cache.S \
        src/main.c src/h3/uart.c src/h3/gic.c src/h3/timer.c src/h3/wdt.c \
        src/h3/rtc.c src/h3/mmu.c src/h3/exceptions.c src/h3/led.c src/h3/ccu.c \
        src/console/console.c src/console/log.c src/lib/libc.c src/lib/fmt.c \
        $(TUSB)/portable/sunxi/h3/dcd_h3_musb.c src/usb/usbphy.c src/usb/usb_descriptors.c \
        src/usb/gadget_slots.c src/usb/tusb_port.c \
        $(TUSB)/portable/ehci/h3/ehci_h3.c src/usb/hcd_h3_ehci.c src/usb/host_mgr.c src/bridge/bridge.c \
        $(TUSB)/host/usbh.c $(TUSB)/host/hub.c \
        $(TUSB)/tusb.c $(TUSB)/common/tusb_fifo.c $(TUSB)/device/usbd.c \
        $(TUSB)/class/cdc/cdc_device.c
GADGET_SERIAL ?= USBPROXYBM

OBJS := $(patsubst %,$(BUILD)/%.o,$(SRCS))

CFLAGS := -mcpu=cortex-a7 -marm -mfloat-abi=soft -ffreestanding -nostdlib \
          -Os -g -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
          -ffunction-sections -fdata-sections -fno-common \
          -Isrc -Isrc/usb -I$(TUSB) -I$(TUSB)/portable/ehci \
          -I$(TUSB)/portable/sunxi/h3 -I$(TUSB)/portable/ehci/h3 \
          -I$(BUILD) -DCFG_TUSB_MCU=OPT_MCU_NONE -DGADGET_SERIAL=\"$(GADGET_SERIAL)\"
CFLAGS += $(EXTRA_CFLAGS)
ASFLAGS := $(CFLAGS)
# $(LINK) links the .o prerequisites with the .ld one, and writes a map.
LINK = $(CC) $(CFLAGS) -T $(filter %.ld,$^) -Wl,--gc-sections -Wl,--no-warn-rwx-segments \
       -Wl,-Map=$(@:.elf=.map) -o $@ $(filter %.o,$^) -lgcc

all: $(BUILD)/fw.bin boot0 felflash size

# The build id lives in a generated header, rewritten at parse time only when
# it changes, so only the objects that include it rebuild (-MMD).
GITREV_H := $(shell mkdir -p $(BUILD); echo '\#define GITREV "$(GITREV)$(DIRTY)"' > $(BUILD)/gitrev.h.tmp; \
            cmp -s $(BUILD)/gitrev.h.tmp $(BUILD)/gitrev.h || mv -f $(BUILD)/gitrev.h.tmp $(BUILD)/gitrev.h; \
            rm -f $(BUILD)/gitrev.h.tmp; echo $(BUILD)/gitrev.h)

$(BUILD)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -MMD -c $< -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/fw.elf: $(OBJS) ld/fw.ld
	$(LINK)

size: $(BUILD)/fw.bin $(BUILD)/boot0.bin $(BUILD)/felflash.bin
	@$(SIZE) $(BUILD)/fw.elf $(BUILD)/boot0.elf $(BUILD)/felflash.elf
	@sz=$$(( $$(wc -c < $(BUILD)/fw.bin) )); echo "fw.bin: $$sz bytes"; \
	 if [ $$sz -gt 98304 ]; then echo "WARNING: over the 96 KiB soft target"; fi

uimage: $(BUILD)/fw.bin
	$(MKIMAGE) -A arm -O linux -T kernel -C none -a $(LOAD) -e $(LOAD) \
	  -n "usbproxy-bm $(GITREV)$(DIRTY)" -d $(BUILD)/fw.bin $(BUILD)/fw.uimg

# boot0: eGON image for the BROM at SD sector 16; clocks, DRAM, reads
# fw.uimg from sector 0x18800, jumps. Shares the H3 helpers and libc with
# the firmware.
BOOT0_SRCS := src/boot0/start.S src/h3/cache.S src/boot0/main.c src/h3/clock.c \
              src/h3/dram.c src/h3/mmc.c src/h3/uart.c src/h3/ccu.c src/h3/wdt.c \
              src/h3/rtc.c src/h3/timer.c src/lib/libc.c src/lib/fmt.c src/lib/crc32.c
BOOT0_OBJS := $(patsubst %,$(BUILD)/%.o,$(BOOT0_SRCS))

$(BUILD)/boot0.elf: $(BOOT0_OBJS) ld/boot0.ld
	$(LINK)

$(BUILD)/boot0.bin: $(BUILD)/boot0.elf tools/egon.py
	$(OBJCOPY) -O binary $< $(BUILD)/boot0.raw
	python3 tools/egon.py $(BUILD)/boot0.raw $@

boot0: $(BUILD)/boot0.bin

# felflash: writes a card region over FEL once boot0 brought DRAM up;
# tools/flash.py drives it. The same objects as boot0, linked for DRAM.
FELFLASH_SRCS := src/felflash/start.S src/felflash/main.c src/h3/mmc.c src/h3/uart.c \
                 src/h3/ccu.c src/h3/timer.c src/lib/libc.c src/lib/fmt.c src/lib/crc32.c
FELFLASH_OBJS := $(patsubst %,$(BUILD)/%.o,$(FELFLASH_SRCS))

$(BUILD)/felflash.elf: $(FELFLASH_OBJS) ld/felflash.ld
	$(LINK)

felflash: $(BUILD)/felflash.bin

# Release onto the card over FEL (tools/flash.py: the firmware's `fel`, boot0
# for DRAM, felflash for the sectors, a watchdog reset, the console's status).
# `make flash BOOT0=1` rewrites boot0 at sector 16 as well.
flash: uimage boot0 felflash
	$(FLASH) flash $(BUILD)/fw.uimg $(if $(filter-out 0,$(BOOT0)),--boot0,)

# Dev loop: run a build from RAM over FEL, card untouched.
run: $(BUILD)/fw.bin boot0
	$(FLASH) run $(BUILD)/fw.bin

# Card out of the board, in the Mac: dd straight at the Falcon sector.
flash-sd: uimage
	@test -n "$(DISK)" || { echo "usage: make flash-sd DISK=/dev/rdiskN"; exit 2; }
	diskutil unmountDisk $(DISK)
	sudo dd if=$(BUILD)/fw.uimg of=$(DISK) bs=512 seek=$(KSECTOR) conv=sync
	sudo dd if=$(DISK) of=$(BUILD)/readback.bin bs=512 skip=$(KSECTOR) count=$$(( ($$(stat -f %z $(BUILD)/fw.uimg) + 511) / 512 ))
	cmp -n $$(stat -f %z $(BUILD)/fw.uimg) $(BUILD)/fw.uimg $(BUILD)/readback.bin && echo "card holds fw.uimg"
	diskutil eject $(DISK)

disasm: $(BUILD)/fw.elf
	$(OBJDUMP) -d $< > $(BUILD)/fw.dis

clean:
	rm -rf $(BUILD)

-include $(sort $(OBJS:.o=.d) $(BOOT0_OBJS:.o=.d) $(FELFLASH_OBJS:.o=.d))
.PHONY: all size uimage boot0 felflash flash run flash-sd disasm clean
