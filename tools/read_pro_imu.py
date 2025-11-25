#!/usr/bin/env python3
"""
Read raw IMU samples from a Nintendo Switch Pro Controller (or Pico spoof) over USB.

Uses the `hidapi` (pyhidapi) package. Press Ctrl+C to exit.
"""

import argparse
import struct
import sys
from typing import List, Tuple

import hid  # from pyhidapi

DEFAULT_VENDOR_ID = 0x057E
DEFAULT_PRODUCT_ID = 0x2009  # Switch Pro Controller (USB)


def list_devices(filter_vid=None, filter_pid=None):
    devices = hid.enumerate()
    for d in devices:
        if filter_vid and d["vendor_id"] != filter_vid:
            continue
        if filter_pid and d["product_id"] != filter_pid:
            continue
        print(
            f"VID=0x{d['vendor_id']:04X} PID=0x{d['product_id']:04X} "
            f"path={d.get('path')} "
            f"serial={d.get('serial_number')} "
            f"manufacturer={d.get('manufacturer_string')} "
            f"product={d.get('product_string')} "
            f"interface={d.get('interface_number')}"
        )
    return devices


def find_device(vendor_id: int, product_id: int):
    for dev in hid.enumerate():
        if dev["vendor_id"] == vendor_id and dev["product_id"] == product_id:
            return dev
    return None


def main():
    parser = argparse.ArgumentParser(description="Read raw 0x30 reports (IMU) from a Switch Pro Controller / Pico.")
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=DEFAULT_VENDOR_ID, help="Vendor ID (default 0x057E)")
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=DEFAULT_PRODUCT_ID, help="Product ID (default 0x2009)")
    parser.add_argument("--path", help="Explicit HID path to open (overrides VID/PID).")
    parser.add_argument("--count", type=int, default=0, help="Stop after this many 0x30 reports (0 = infinite).")
    parser.add_argument("--timeout", type=int, default=3000, help="Read timeout ms (default 3000).")
    parser.add_argument("--list", action="store_true", help="List detected HID devices and exit.")
    parser.add_argument("--plot", action="store_true", help="Plot accel/gyro traces after capture (requires matplotlib).")
    parser.add_argument("--save-prefix", help="If set, save accel/gyro plots as '<prefix>_accel.png' and '<prefix>_gyro.png'.")
    args = parser.parse_args()

    if args.list:
        list_devices()
        return

    if args.path:
        dev_info = {"path": bytes(args.path, encoding="utf-8"), "vendor_id": args.vid, "product_id": args.pid}
    else:
        dev_info = find_device(args.vid, args.pid)
        if not dev_info:
            print(
                f"No HID device found for VID=0x{args.vid:04X} PID=0x{args.pid:04X}. "
                "Use --list to inspect devices or --path to target a specific one.",
                file=sys.stderr,
            )
            sys.exit(1)

    device = hid.device()
    device.open_path(dev_info["path"])
    device.set_nonblocking(False)
    print(
        f"Reading raw 0x30 reports from device (VID=0x{args.vid:04X} PID=0x{args.pid:04X})... "
        "Ctrl+C to stop."
    )
    accel_series: List[Tuple[int, int, int]] = []
    gyro_series: List[Tuple[int, int, int]] = []
    try:
        read_count = 0
        while args.count == 0 or read_count < args.count:
            data = device.read(64, timeout_ms=args.timeout)
            if not data:
                print(f"(timeout after {args.timeout} ms, no data)")
                continue
            if data[0] != 0x30:
                print(f"(non-0x30 report id=0x{data[0]:02X}, len={len(data)})")
                continue
            samples = []
            offset = 13  # accel_x starts at byte 13
            for _ in range(3):
                ax, ay, az, gx, gy, gz = struct.unpack_from("<hhhhhh", bytes(data), offset)
                samples.append((ax, ay, az, gx, gy, gz))
                offset += 12
            print(samples)
            accel_series.extend((s[0], s[1], s[2]) for s in samples)
            gyro_series.extend((s[3], s[4], s[5]) for s in samples)
            read_count += 1
    except KeyboardInterrupt:
        pass
    finally:
        device.close()

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - optional dependency
            print(f"Unable to plot (matplotlib not available): {exc}", file=sys.stderr)
            return

        if accel_series and gyro_series:
            # Each sample is a tuple of three axes; plot per axis vs sample index.
            accel_x = [s[0] for s in accel_series]
            accel_y = [s[1] for s in accel_series]
            accel_z = [s[2] for s in accel_series]
            gyro_x = [s[0] for s in gyro_series]
            gyro_y = [s[1] for s in gyro_series]
            gyro_z = [s[2] for s in gyro_series]

            fig1, ax1 = plt.subplots()
            ax1.plot(accel_x, label="ax")
            ax1.plot(accel_y, label="ay")
            ax1.plot(accel_z, label="az")
            ax1.set_title("Accel (counts)")
            ax1.set_xlabel("Sample")
            ax1.set_ylabel("Counts")
            ax1.legend()

            fig2, ax2 = plt.subplots()
            ax2.plot(gyro_x, label="gx")
            ax2.plot(gyro_y, label="gy")
            ax2.plot(gyro_z, label="gz")
            ax2.set_title("Gyro (counts)")
            ax2.set_xlabel("Sample")
            ax2.set_ylabel("Counts")
            ax2.legend()

            if args.save_prefix:
                fig1.savefig(f"{args.save_prefix}_accel.png", dpi=150, bbox_inches="tight")
                fig2.savefig(f"{args.save_prefix}_gyro.png", dpi=150, bbox_inches="tight")
                print(f"Saved plots to {args.save_prefix}_accel.png and {args.save_prefix}_gyro.png")

            plt.show()


if __name__ == "__main__":
    main()
