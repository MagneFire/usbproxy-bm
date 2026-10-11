# usbproxy-bm: bare-metal usb-proxy firmware for the Orange Pi Zero (H2+/H3).
# Loaded by boot0 (src/boot0: the uImage at SD sector 0x18800) or over FEL
# (tools/flash.py). See README.md and BARE-METAL.md in meta-usbproxy.
CROSS     ?= /opt/homebrew/bin/arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
OBJDUMP   := $(CROSS)objdump
SIZE      := $(CROSS)size
MKIMAGE   ?= mkimage
SUNXI_FEL ?= $(HOME)/.local/bin/sunxi-fel
# FEL loader/flasher (needs sunxi-fel; uv fetches pyserial)
FLASH     ?= uv run tools/flash.py
# what `make fel` runs first to bring DRAM up (the U-Boot SPL still works here)
SPL       ?= $(BUILD)/boot0.bin

BUILD   := build
TUSB    := third_party/tinyusb/src
GITREV  := $(shell git rev-parse --short=10 HEAD 2>/dev/null || echo nogit)
DIRTY   := $(shell git diff --quiet HEAD 2>/dev/null || echo +)
LOAD    := 0x40008000
# 0x18800, applib.KERNEL_SECTOR
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
LDFLAGS := -nostdlib -T ld/fw.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map=$(BUILD)/fw.map

all: $(BUILD)/fw.bin size boot0 felflash

# The build id lives in a generated header, rewritten at parse time only when
# it changes, so the objects that include it rebuild (-MMD) when the tree
# moves (a -D on the command line left stale "a849d72+" strings in a newer
# binary once; a phony-driven rule for the header never retriggered under
# Apple's make).
GITREV_H := $(shell mkdir -p $(BUILD); echo '\#define GITREV "$(GITREV)$(DIRTY)"' > $(BUILD)/gitrev.h.tmp; \
            cmp -s $(BUILD)/gitrev.h.tmp $(BUILD)/gitrev.h || mv -f $(BUILD)/gitrev.h.tmp $(BUILD)/gitrev.h; \
            rm -f $(BUILD)/gitrev.h.tmp; echo $(BUILD)/gitrev.h)

$(BUILD)/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -MMD -c $< -o $@

$(BUILD)/fw.elf: $(OBJS) ld/fw.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) -lgcc

$(BUILD)/fw.bin: $(BUILD)/fw.elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILD)/fw.elf
	@$(SIZE) $<
	@sz=$$(stat -f %z $(BUILD)/fw.bin); echo "fw.bin: $$sz bytes"; \
	 if [ $$sz -gt 71680 ]; then echo "WARNING: over the 70 KiB soft target"; fi

uimage: $(BUILD)/fw.bin
	$(MKIMAGE) -A arm -O linux -T kernel -C none -a $(LOAD) -e $(LOAD) \
	  -n "usbproxy-bm $(GITREV)$(DIRTY)" -d $(BUILD)/fw.bin $(BUILD)/fw.uimg

# boot0: our replacement for the U-Boot SPL (src/boot0/, BARE-METAL.md section
# 8c). eGON image for the BROM at SD sector 16; clocks, DRAM, reads fw.uimg
# from sector 0x18800, jumps. Shares the H3 helpers and libc with the firmware.
BOOT0_SRCS := src/boot0/start.S src/h3/cache.S src/boot0/main.c src/h3/clock.c \
              src/h3/dram.c src/h3/mmc.c src/h3/uart.c src/h3/ccu.c src/h3/wdt.c \
              src/h3/rtc.c src/h3/timer.c src/lib/libc.c src/lib/fmt.c src/lib/crc32.c
BOOT0_OBJS := $(patsubst %,$(BUILD)/boot0/%.o,$(BOOT0_SRCS))
BOOT0_LDFLAGS := -nostdlib -T ld/boot0.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map=$(BUILD)/boot0.map

$(BUILD)/boot0/%.c.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -DBOOT0 -MMD -c $< -o $@

$(BUILD)/boot0/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -DBOOT0 -MMD -c $< -o $@

$(BUILD)/boot0.elf: $(BOOT0_OBJS) ld/boot0.ld
	$(CC) $(CFLAGS) $(BOOT0_LDFLAGS) -o $@ $(BOOT0_OBJS) -lgcc

$(BUILD)/boot0.bin: $(BUILD)/boot0.elf tools/egon.py
	$(OBJCOPY) -O binary $< $(BUILD)/boot0.raw
	python3 tools/egon.py $(BUILD)/boot0.raw $@
	@$(SIZE) $<

boot0: $(BUILD)/boot0.bin

# felflash: writes a card region over FEL (after `sunxi-fel spl boot0.bin`
# brought DRAM up); tools/flash.py drives it. Same objects as
# boot0, linked for DRAM.
FELFLASH_SRCS := src/felflash/start.S src/felflash/main.c src/h3/mmc.c src/h3/uart.c \
                 src/h3/ccu.c src/h3/timer.c src/lib/libc.c src/lib/fmt.c src/lib/crc32.c
FELFLASH_OBJS := $(patsubst %,$(BUILD)/boot0/%.o,$(FELFLASH_SRCS))

$(BUILD)/felflash.elf: $(FELFLASH_OBJS) ld/felflash.ld
	$(CC) $(CFLAGS) -nostdlib -T ld/felflash.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map=$(BUILD)/felflash.map -o $@ $(FELFLASH_OBJS) -lgcc

$(BUILD)/felflash.bin: $(BUILD)/felflash.elf
	$(OBJCOPY) -O binary $< $@
	@$(SIZE) $<

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

# Recovery / card-less dev: BROM FEL over the OTG cable.
fel: $(BUILD)/fw.bin $(SPL)
	$(SUNXI_FEL) -v spl $(SPL)
	$(SUNXI_FEL) -p write $(LOAD) $(BUILD)/fw.bin
	$(SUNXI_FEL) exe $(LOAD)

# Leave FEL without touching the board: a 24-byte blob that arms the H3
# watchdog for a whole-system reset (the card boots normally after it).
$(BUILD)/felreset.bin: tools/felreset.S
	@mkdir -p $(BUILD)
	$(CROSS)as -o $(BUILD)/felreset.o $<
	$(CROSS)objcopy -O binary $(BUILD)/felreset.o $@
fel-reset: $(BUILD)/felreset.bin
	$(SUNXI_FEL) write 0x2000 $(BUILD)/felreset.bin
	$(SUNXI_FEL) exe 0x2000

disasm: $(BUILD)/fw.elf
	$(OBJDUMP) -d $< > $(BUILD)/fw.dis

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d) $(BOOT0_OBJS:.o=.d) $(FELFLASH_OBJS:.o=.d)
.PHONY: all size uimage boot0 felflash flash run flash-sd fel fel-reset disasm clean
