# usbproxy-bm

Bare-metal USB proxy firmware for the Orange Pi Zero (Allwinner H2+/H3).
A device plugged into the board's USB-A port (adb / fastboot) is bridged to
the board's OTG port, where the board presents one fixed USB gadget. No
operating system, no bootloader: `boot0` (our own first stage) loads the
firmware from the SD card and jumps to it. The firmware is a superloop on
TinyUSB (musb gadget on the OTG port, EHCI host on the USB-A port) with a
CDC-ACM console on the gadget.

## The gadget

With a watch attached the gadget is the console plus adb and fastboot,
carrying the watch's serial, so `adb devices` names the watch. It keeps that
serial while the watch reboots or switches between adb and fastboot, so the
console stays open; a different watch re-enumerates it. With no watch for
60 s (console `sticky [ms]`) it is the console alone with serial
`USBPROXYBM`. The console's port name changes with the gadget: on macOS
`/dev/cu.usbmodemUSBPROXYBM1` idle, and with a watch a name from the serial
or, for a long serial, from the port (`/dev/cu.usbmodem101`).

## Build

Needs `arm-none-eabi-gcc`, `mkimage` (u-boot-tools) and Python 3.

    make            # build/fw.bin, build/boot0.bin, build/felflash.bin
    make uimage     # build/fw.uimg: fw.bin with the legacy uImage header boot0 expects
    make size       # sizes of the three images
    make disasm     # build/fw.dis

Override the toolchain prefix with `CROSS=`.

## Card layout

| sector | content |
|---|---|
| 0x00010 | `boot0.bin`, eGON image the BROM loads (tools/egon.py adds the checksum) |
| 0x18800 | `fw.uimg` |

## Flashing

Over the OTG cable, no serial adapter; needs `sunxi-fel` from
[sunxi-tools](https://github.com/linux-sunxi/sunxi-tools) (`SUNXI_FEL=` if not
in `~/.local/bin`) and `uv` for `tools/flash.py`. The console is the first
`/dev/cu.usbmodem*`; set `BM_CONSOLE=` or pass `--console` to pick another.

    make flash              # firmware only
    make flash BOOT0=1      # firmware, then boot0
    make run                # a build from RAM, card untouched

The firmware's `fel` console command puts the SoC into the BROM's FEL mode,
`boot0.bin` brings DRAM up, `felflash.bin` writes and verifies the sectors,
a watchdog reset boots the card, and the console is checked. A board that is
already in FEL (empty card, bad image, firmware that died three boots in a
row) is handled the same way. `tools/flash.py fel` / `reset` enter and leave FEL by hand.

A fresh card from a reader:

    make uimage boot0
    dd if=build/boot0.bin of=/dev/rdiskN bs=512 seek=16
    dd if=build/fw.uimg  of=/dev/rdiskN bs=512 seek=100352

`make flash-sd DISK=/dev/rdiskN` writes and verifies `fw.uimg` only.
