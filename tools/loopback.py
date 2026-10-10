#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb", "pyserial"]
# ///
"""loopback.py — drive the firmware's adb-slot loopback from the Mac (M1).

Enables `loop 0 1` over the gadget's CDC console, claims the adb bridge
interface (class ff/42/01) with libusb, and pushes patterns through:
full 512-byte packets, short packets, zero-length packets, then a sustained
ping-pong of N packets, md5-checking every echo. Stop adb first (it holds
the interface): `adb kill-server`.

    uv run tools/loopback.py [--serial USBPROXYBM] [--packets 20000] [--size 512]
"""
import argparse
import glob
import hashlib
import os
import sys
import time

import serial
import usb.core
import usb.util


def console(cmd, serial_no):
    nodes = glob.glob(f"/dev/cu.usbmodem{serial_no}*")
    if not nodes:
        sys.exit(f"no CDC console node for {serial_no}")
    with serial.Serial(nodes[0], 115200, timeout=0.5) as s:
        s.write((cmd + "\n").encode())
        time.sleep(0.3)
        return s.read(4096).decode(errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="USBPROXYBM")
    ap.add_argument("--packets", type=int, default=20000)
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--timeout", type=int, default=2000)
    ap.add_argument("--flood-mb", type=int, default=0,
                    help="OUT flood with loop off; needs the idle sink (M4a): with the M3 bridge the 4 buffers fill and the write stalls")
    args = ap.parse_args()

    console("loop 0 1", args.serial)
    dev = usb.core.find(idVendor=0x1d6b, idProduct=0x0104,
                        custom_match=lambda d: usb.util.get_string(d, d.iSerialNumber) == args.serial)
    if dev is None:
        sys.exit("gadget not found (serial %s)" % args.serial)
    cfg = dev.get_active_configuration()
    itf = next(i for i in cfg if i.bInterfaceClass == 0xff and i.bInterfaceSubClass == 0x42
               and i.bInterfaceProtocol == 0x01)
    usb.util.claim_interface(dev, itf.bInterfaceNumber)
    ep_out = next(e for e in itf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
    ep_in = next(e for e in itf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
    print(f"adb slot: iface {itf.bInterfaceNumber} out {ep_out.bEndpointAddress:#04x} in {ep_in.bEndpointAddress:#04x} "
          f"maxp {ep_out.wMaxPacketSize}")

    # Drain anything a previous run left pending on the IN side.
    try:
        while True:
            ep_in.read(512, timeout=200)
    except usb.core.USBTimeoutError:
        pass

    def xfer(payload):
        ep_out.write(payload, timeout=args.timeout)
        got = bytes(ep_in.read(max(len(payload), 1) if len(payload) else 512, timeout=args.timeout)) if len(payload) else b""
        return got

    # 1. full packets, 2. short packets, 3. a zero-length packet is not
    # echoed as a read (reading a ZLP needs a pending IN; skip the read).
    for n in (512, 1, 7, 64, 511, 512):
        pl = os.urandom(n)
        got = xfer(pl)
        print(f"  {n:4d} B: {'OK' if got == pl else 'MISMATCH %d' % len(got)}")
    ep_out.write(b"", timeout=args.timeout)
    got = bytes(ep_in.read(512, timeout=args.timeout))
    print(f"  ZLP: echoed as {len(got)} B {'OK' if not got else 'MISMATCH'}")
    # ping-pong
    bad = 0
    t0 = time.time()
    h_out = hashlib.md5(); h_in = hashlib.md5()
    for i in range(args.packets):
        pl = os.urandom(args.size)
        h_out.update(pl)
        got = xfer(pl)
        h_in.update(got)
        if got != pl:
            bad += 1
            if bad < 5:
                print(f"  packet {i}: mismatch ({len(got)} B)")
    dt = time.time() - t0
    print(f"ping-pong {args.packets} x {args.size} B: {args.packets*args.size/dt/1e6:.2f} MB/s each way, "
          f"{dt/args.packets*1e6:.0f} us per round trip, {bad} mismatches, md5 {'OK' if h_in.digest()==h_out.digest() else 'DIFF'}")
    # Flood: loop off, the firmware just re-arms and counts; measures how
    # fast the musb ISR + task can swallow packets (service time ceiling).
    console("loop 0 0", args.serial)
    time.sleep(0.2)
    before = console("status", args.serial)
    chunk = os.urandom(64 * 1024)
    if not args.flood_mb:
        usb.util.release_interface(dev, itf.bInterfaceNumber)
        console("loop 0 0", args.serial)
        return
    n_chunks = args.flood_mb * 16
    t0 = time.time()
    for _ in range(n_chunks):
        ep_out.write(chunk, timeout=5000)
    dt = time.time() - t0
    after = console("status", args.serial)
    def outpkts(st):
        return int(st.split("slot0=")[-1].split("out:")[1].split("/")[0])
    got = outpkts(after) - outpkts(before)
    print(f"flood {args.flood_mb} MB OUT, loop off: {args.flood_mb/dt:.2f} MB/s, "
          f"{dt/max(got,1)*1e6:.1f} us per packet at the gadget, {got} packets counted "
          f"({n_chunks*128} sent)")
    usb.util.release_interface(dev, itf.bInterfaceNumber)
    print(console("status", args.serial).split("slot0=")[-1].split("\n")[0])
    console("loop 0 0", args.serial)


if __name__ == "__main__":
    main()
