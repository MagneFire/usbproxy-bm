# usbproxy-bm: bare-metal usb-proxy firmware for the Orange Pi Zero (H2+/H3).
# Loaded by the U-Boot SPL (Falcon payload at SD sector 0x18800, a Linux legacy
# uImage) or by U-Boot proper (`loady 0x40008000` + `go 0x40008000`) or over
# FEL. One binary, all three entries. See BARE-METAL.md in meta-usbproxy.
CROSS     ?= /opt/homebrew/bin/arm-none-eabi-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
OBJDUMP   := $(CROSS)objdump
SIZE      := $(CROSS)size
MKIMAGE   ?= mkimage
SUNXI_FEL ?= $(HOME)/.local/bin/sunxi-fel
APPLIANCE ?= uv run $(HOME)/Downloads/meta-usbproxy/scripts/appliance.py
SPL       ?= $(HOME)/Downloads/devcard-u-boot-sunxi-with-spl.bin

BUILD   := build
TUSB    := third_party/tinyusb/src
GITREV  := $(shell git rev-parse --short=10 HEAD 2>/dev/null || echo nogit)
DIRTY   := $(shell git diff --quiet 2>/dev/null || echo +)
LOAD    := 0x40008000
KSECTOR := 100352          # 0x18800, applib.KERNEL_SECTOR

SRCS := src/start.S src/h3/cache.S \
        src/main.c src/h3/uart.c src/h3/gic.c src/h3/timer.c src/h3/wdt.c \
        src/h3/rtc.c src/h3/mmu.c src/h3/exceptions.c src/h3/led.c src/h3/ccu.c \
        src/console/console.c src/console/log.c src/lib/libc.c src/lib/fmt.c

OBJS := $(patsubst %,$(BUILD)/%.o,$(SRCS))

CFLAGS := -mcpu=cortex-a7 -marm -mfloat-abi=soft -ffreestanding -nostdlib \
          -Os -g -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
          -ffunction-sections -fdata-sections -fno-common \
          -Isrc -Isrc/usb -I$(TUSB) \
          -DGITREV=\"$(GITREV)$(DIRTY)\" -DCFG_TUSB_MCU=OPT_MCU_NONE
ASFLAGS := $(CFLAGS)
LDFLAGS := -nostdlib -T ld/fw.ld -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map=$(BUILD)/fw.map

all: $(BUILD)/fw.bin size

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

# Dev loop: U-Boot prompt over the UART dongle, Y-modem into RAM, go.
run-uboot: $(BUILD)/fw.bin
	$(APPLIANCE) bm run $(BUILD)/fw.bin

# Release onto the card's Falcon kernel sector, then reset and check.
flash-uboot: uimage
	$(APPLIANCE) bm flash $(BUILD)/fw.uimg

# Card out of the board, in the Mac: dd straight at the Falcon sector.
flash-sd: uimage
	@test -n "$(DISK)" || { echo "usage: make flash-sd DISK=/dev/rdiskN"; exit 2; }
	diskutil unmountDisk $(DISK)
	sudo dd if=$(BUILD)/fw.uimg of=$(DISK) bs=512 seek=$(KSECTOR) conv=sync
	sudo dd if=$(DISK) of=$(BUILD)/readback.bin bs=512 skip=$(KSECTOR) count=$$(( ($$(stat -f %z $(BUILD)/fw.uimg) + 511) / 512 ))
	cmp -n $$(stat -f %z $(BUILD)/fw.uimg) $(BUILD)/fw.uimg $(BUILD)/readback.bin && echo "card holds fw.uimg"
	diskutil eject $(DISK)

# Recovery / card-less dev: BROM FEL over the OTG cable.
fel: $(BUILD)/fw.bin
	$(SUNXI_FEL) -v spl $(SPL)
	$(SUNXI_FEL) -p write $(LOAD) $(BUILD)/fw.bin
	$(SUNXI_FEL) exe $(LOAD)

disasm: $(BUILD)/fw.elf
	$(OBJDUMP) -d $< > $(BUILD)/fw.dis

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all size uimage run-uboot flash-uboot flash-sd fel disasm clean
