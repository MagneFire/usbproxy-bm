#!/usr/bin/env python3
"""Finish an eGON.BT0 image for the Allwinner BROM (U-Boot tools/mksunxiboot.c):
pad to a multiple of 8 KiB, write the length, then the checksum = sum of all
32-bit words with the checksum field holding the stamp 0x5F0A6C39.

usage: egon.py in.bin out.bin"""
import struct, sys

STAMP = 0x5F0A6C39
BLOCK = 0x2000

src, dst = sys.argv[1], sys.argv[2]
img = bytearray(open(src, "rb").read())
if img[4:12] != b"eGON.BT0":
    sys.exit(f"{src}: no eGON.BT0 header at offset 4")
img += b"\0" * (-len(img) % BLOCK)
struct.pack_into("<I", img, 0x10, len(img))
struct.pack_into("<I", img, 0x0c, STAMP)
total = sum(struct.unpack_from(f"<{len(img) // 4}I", img)) & 0xFFFFFFFF
struct.pack_into("<I", img, 0x0c, total)
open(dst, "wb").write(img)
print(f"{dst}: {len(img)} bytes, checksum {total:08x}")
