# usbproxy-bm

Bare-metal USB proxy firmware for the Orange Pi Zero (Allwinner H2+/H3).
A device plugged into the board's USB-A port (adb / fastboot) is bridged to
the board's OTG port, where the board presents one fixed USB gadget. No
operating system, no bootloader: `boot0` (our own first stage) loads the
firmware from the SD card and jumps to it. The firmware is a superloop on
TinyUSB (musb gadget on the OTG port, EHCI host on the USB-A port) with a
CDC-ACM console on the gadget. Design and measurements: `BARE-METAL.md` in
the meta-usbproxy repository.

## Build

Needs `arm-none-eabi-gcc` (Homebrew: `/opt/homebrew/bin/arm-none-eabi-*`),
`mkimage` (u-boot-tools) and Python 3.

    make            # build/fw.bin, build/boot0.bin, build/felflash.bin
    make uimage     # build/fw.uimg: fw.bin with the legacy uImage header boot0 expects

Override the toolchain prefix with `CROSS=`.

## Card layout

| sector | content |
|---|---|
| 16 (8 KiB) | `boot0.bin`, eGON image the BROM loads (tools/egon.py adds the checksum) |
| 0x18800 (49 MiB) | `fw.uimg` |

Nothing else on the card is read.

## Flashing

Over the OTG cable, no serial adapter, with [sunxi-tools](https://github.com/linux-sunxi/sunxi-tools)
and `scripts/appliance.py` from meta-usbproxy (`APPLIANCE=` in the Makefile):

    make flash              # firmware only
    make flash BOOT0=1      # firmware, then boot0

The firmware's `fel` console command puts the SoC into the BROM's FEL mode,
`boot0.bin` brings DRAM up, `felflash.bin` writes and verifies the sectors,
a watchdog reset boots the card, and the console is checked. A board that is
already in FEL (empty card, bad image, firmware that died three boots in a
row) is handled the same way. `make fel` runs a build from RAM over FEL
without writing the card; `make fel-reset` leaves FEL.

A fresh card from a reader:

    make uimage
    dd if=build/boot0.bin of=/dev/rdiskN bs=512 seek=16
    dd if=build/fw.uimg  of=/dev/rdiskN bs=512 seek=100352
