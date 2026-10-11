#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Load or flash usbproxy-bm over the OTG cable through the BROM's FEL mode.

    tools/flash.py flash build/fw.uimg [--boot0]                   write the card
    tools/flash.py run   build/fw.bin                                run from RAM, card untouched
    tools/flash.py fel                                               into FEL and stay there
    tools/flash.py reset                                             leave FEL (watchdog reset)

Needs sunxi-fel (sunxi-tools). Sequence: the firmware's `fel` console command
(a running firmware; a board already in FEL is fine), `sunxi-fel spl
boot0.bin` to bring DRAM up, then either write + exe the firmware, or
felflash.bin per card region (data + command block in DRAM, exe, status
read back: src/felflash/main.c), a watchdog-reset blob to leave FEL, and the
console's `status` once it is back. boot0 and felflash default to the files
next to the firmware image."""
import argparse, glob, os, pathlib, struct, subprocess, sys, tempfile, time, zlib

SUNXI_FEL = os.environ.get("SUNXI_FEL") or os.path.expanduser("~/.local/bin/sunxi-fel")
CONSOLE_GLOB = os.environ.get("BM_CONSOLE") or "/dev/cu.usbmodem*"

FW_LOAD = 0x40008000            # ld/fw.ld IMG, the uImage's load/entry
FW_SECTOR = 0x18800             # src/boot0/boot0.h FW_SECTOR
BOOT0_SECTOR = 0x10
FELFLASH_ADDR = 0x4ff00000      # ld/felflash.ld
FELFLASH_CMD = 0x4ff10000       # src/felflash/main.c command block
FELFLASH_DATA = 0x41000000      # data, readback right after it
FELFLASH_MAGIC = 0x48534c46     # "FLSH"
FELFLASH_STATUS = {0: "ok", 1: "bad magic", 2: "data crc (transfer)", 3: "mmc init",
                   4: "write", 5: "readback", 6: "verify crc"}
# tools/felreset.S: arm the watchdog for a whole-system reset
FELRESET_BLOB = bytes.fromhex("0c009fe50110a0e3141080e5181080e5feffffeaa00cc201")
FELRESET_ADDR = 0x2000


def say(s):
    print(s, flush=True)


def die(s):
    sys.exit(f"flash.py: {s}")


def fel(*args, timeout=120, stdout_only=False):
    try:
        r = subprocess.run([SUNXI_FEL, *args], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return -1, f"sunxi-fel {args[0]} timed out after {timeout} s"
    return r.returncode, (r.stdout or "") + ("" if stdout_only else (r.stderr or ""))


def fel_present():
    return fel("ver", timeout=10)[0] == 0


def fel_wait(timeout):
    end = time.time() + timeout
    while time.time() < end:
        if fel_present():
            return True
        time.sleep(0.5)
    return False


def console_nodes():
    return sorted(glob.glob(CONSOLE_GLOB))


def console_cmd(node, cmd, read_s=1.0):
    import serial
    with serial.Serial(node, 115200, timeout=0.2) as s:
        s.write(b"\n")
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write(cmd.encode() + b"\n")
        s.flush()
        end = time.time() + read_s
        out = b""
        while time.time() < end:
            d = s.read(4096)
            if d:
                out += d
        return out.decode("utf-8", "replace")


def fel_request(console):
    if fel_present():
        say("board is in FEL")
        return
    nodes = [console] if console else console_nodes()
    if not nodes:
        die(f"no FEL device and no console ({CONSOLE_GLOB}); is the board plugged in?")
    try:
        out = console_cmd(nodes[0], "fel", 0.5)
    except Exception as e:      # the node dies with the reset; that is fine
        out = str(e)
    say(f"asked the firmware on {nodes[0]} for FEL")
    if not fel_wait(30 if "fel requested" in out else 10):
        die(f"no FEL device appeared; the console answered:\n{out[-300:]}")


def fel_write(addr, data, what):
    with tempfile.NamedTemporaryFile(delete=False, prefix="fel-") as f:
        f.write(data)
        path = f.name
    try:
        rc, out = fel("-p", "write", f"0x{addr:x}", path)
    finally:
        os.unlink(path)
    if rc != 0:
        die(f"sunxi-fel write of {what} failed:\n{out[-400:]}")
    say(f"  {what}: {len(data)} bytes at 0x{addr:x}")


def fel_readl(addr):
    rc, out = fel("readl", f"0x{addr:x}", timeout=20, stdout_only=True)
    try:
        return int(out.split()[-1], 16) if rc == 0 else None
    except (IndexError, ValueError):
        return None


def fel_exe(addr):
    rc, out = fel("exe", f"0x{addr:x}")
    if rc != 0:
        die(f"sunxi-fel exe failed:\n{out[-200:]}")


def dram_up(boot0):
    if boot0.read_bytes()[4:12] != b"eGON.BT0":
        die(f"{boot0}: not an eGON.BT0 image")
    rc, out = fel("spl", str(boot0))
    if rc != 0:
        die(f"sunxi-fel spl {boot0} failed:\n{out[-400:]}")
    say(f"DRAM up via {boot0.name}")


def fel_reset():
    fel_write(FELRESET_ADDR, FELRESET_BLOB, "felreset")
    fel("exe", f"0x{FELRESET_ADDR:x}", timeout=10)
    say("left FEL (watchdog reset)")


def flash_region(sector, data, what):
    data = data + b"\0" * (-len(data) % 512)
    crc = zlib.crc32(data) & 0xffffffff
    fel_write(FELFLASH_DATA, data, what)
    cmd = struct.pack("<7I", FELFLASH_MAGIC, sector, len(data) // 512, crc, FELFLASH_DATA, 0xffffffff, 0)
    fel_write(FELFLASH_CMD, cmd, "cmd")
    fel_exe(FELFLASH_ADDR)
    st = fel_readl(FELFLASH_CMD + 0x14)
    if st is None:
        die("felflash did not come back to the BROM: FEL stopped answering. The BROM has no watchdog "
            "in FEL mode, so power-cycle the board and rerun (boot0 enters FEL by itself on a bad image).")
    ms = fel_readl(FELFLASH_CMD + 0x18)
    if st != 0:
        die(f"{what} at sector 0x{sector:x}: {FELFLASH_STATUS.get(st, st)} (status {st}); rerun")
    took = f" in {ms} ms" if ms is not None else ""
    say(f"  {what}: {len(data)} bytes written and verified at sector 0x{sector:x}{took}")


def wait_console(node=None, timeout=90):
    """The console node the firmware brings back after the reset: the one asked
    for, or the first that appears. A CDC node went away with the FEL entry; a
    UART node never does, so it is there before the firmware is (see report)."""
    end = time.time() + timeout
    while time.time() < end:
        nodes = [node] if node else console_nodes()
        if nodes and os.path.exists(nodes[0]):
            time.sleep(1.5)
            return nodes[0]
        time.sleep(0.5)
    return None


def report(console):
    node = wait_console(console)
    if not node:
        die(f"no console {console or CONSOLE_GLOB} within 90 s after the reset")
    # a UART node is there from the start: ask until the firmware answers
    end = time.time() + 30
    while True:
        try:
            out = console_cmd(node, "status", 1.5)
        except Exception as e:
            out = str(e)
        build = [l for l in out.splitlines() if l.startswith("build=")]
        if build or time.time() > end:
            break
        time.sleep(1)
    say(f"console {node}: {build[0] if build else 'no build line in status'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("op", choices=["flash", "run", "fel", "reset"])
    ap.add_argument("image", nargs="?", help="fw.uimg (flash) or fw.bin (run)")
    ap.add_argument("--boot0", action="store_true", help="flash: also write boot0 (--boot0-bin) to sector 16")
    ap.add_argument("--boot0-bin", help="boot0 image for the FEL DRAM init and --boot0 (default: boot0.bin next to the image)")
    ap.add_argument("--felflash", help="felflash image (default: felflash.bin next to the image)")
    ap.add_argument("--console", help=f"console node for the fel request and the check (default: first {CONSOLE_GLOB})")
    a = ap.parse_args()

    if not os.path.exists(SUNXI_FEL):
        die(f"{SUNXI_FEL} not found (sunxi-tools; set SUNXI_FEL)")
    if a.op == "reset":
        if not fel_present():
            die("no FEL device")
        fel_reset()
        return
    if a.op == "fel":
        fel_request(a.console)
        return
    if not a.image:
        die("image file required")
    img = pathlib.Path(a.image)
    data = img.read_bytes()
    build = img.parent
    boot0 = pathlib.Path(a.boot0_bin) if a.boot0_bin else build / "boot0.bin"
    if not boot0.exists():
        die(f"{boot0}: not found (make boot0)")

    fel_request(a.console)
    dram_up(boot0)
    if a.op == "run":
        if data[:4] == b"\x27\x05\x19\x56":
            die("run takes the raw fw.bin, not the uImage")
        fel_write(FW_LOAD, data, img.name)
        fel_exe(FW_LOAD)
        say(f"running from RAM (the card's image is back after the next reset)")
    else:
        if data[:4] != b"\x27\x05\x19\x56":
            die(f"{img}: not a uImage (make uimage)")
        felflash = pathlib.Path(a.felflash) if a.felflash else build / "felflash.bin"
        if not felflash.exists():
            die(f"{felflash}: not found (make felflash)")
        fel_write(FELFLASH_ADDR, felflash.read_bytes(), felflash.name)
        flash_region(FW_SECTOR, data, img.name)
        if a.boot0:
            # dram_up checked it is an eGON image; last: a torn boot0 is rejected by the BROM
            flash_region(BOOT0_SECTOR, boot0.read_bytes(), boot0.name)
        fel_reset()
    report(a.console)


if __name__ == "__main__":
    main()
