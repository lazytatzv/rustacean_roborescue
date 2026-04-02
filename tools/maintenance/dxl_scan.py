#!/usr/bin/env python3
"""Scan Dynamixel IDs on a serial bus.

Examples:
  uv run python dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --protocol 2.0
  uv run python dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols
"""

from __future__ import annotations

import argparse
import sys
from typing import Iterable

from dynamixel_sdk import PacketHandler, PortHandler


def _scan_ids(device: str, baud: int, protocol: float, min_id: int, max_id: int) -> list[tuple[int, int]]:
    port = PortHandler(device)
    if not port.openPort():
        raise RuntimeError(f"openPort failed: {device}")
    try:
        if not port.setBaudRate(baud):
            raise RuntimeError(f"setBaudRate failed: {baud}")

        pkt = PacketHandler(protocol)
        found: list[tuple[int, int]] = []

        for dxl_id in range(min_id, max_id + 1):
            model, comm_result, error = pkt.ping(port, dxl_id)
            if comm_result == 0 and error == 0:
                found.append((dxl_id, model))
        return found
    finally:
        port.closePort()


def _run_protocols(
    device: str,
    baud: int,
    protocols: Iterable[float],
    min_id: int,
    max_id: int,
) -> tuple[bool, int]:
    any_found = False

    for proto in protocols:
        print(f"[dxl-scan] scanning protocol={proto} device={device} baud={baud} ids={min_id}-{max_id}")
        try:
            found = _scan_ids(device, baud, proto, min_id, max_id)
        except RuntimeError as e:
            print(f"[dxl-scan] error: {e}", file=sys.stderr)
            return False, 2

        if not found:
            print("[dxl-scan] no IDs found")
            continue

        any_found = True
        for dxl_id, model in found:
            print(f"FOUND id={dxl_id} model={model}")

    return any_found, (0 if any_found else 1)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan Dynamixel IDs on serial bus")
    parser.add_argument("--device", required=True, help="Serial device, e.g. /dev/dynamixel_flipper")
    parser.add_argument("--baud", type=int, default=1000000, help="Baud rate")
    parser.add_argument("--protocol", type=float, default=2.0, choices=[1.0, 2.0], help="Protocol")
    parser.add_argument("--both-protocols", action="store_true", help="Scan protocol 2.0 and 1.0")
    parser.add_argument(
        "--bauds",
        type=int,
        nargs="+",
        help="Scan multiple baud rates (space-separated), e.g. --bauds 1000000 57600 115200",
    )
    parser.add_argument("--min-id", type=int, default=1, help="Minimum Dynamixel ID")
    parser.add_argument("--max-id", type=int, default=252, help="Maximum Dynamixel ID")
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="Return success even if no IDs are found",
    )
    args = parser.parse_args()

    if args.min_id < 0 or args.max_id > 252 or args.min_id > args.max_id:
        print("[dxl-scan] invalid ID range", file=sys.stderr)
        return 2

    protocols = [2.0, 1.0] if args.both_protocols else [args.protocol]
    bauds = args.bauds if args.bauds else [args.baud]

    any_found_total = False
    for baud in bauds:
        found, code = _run_protocols(args.device, baud, protocols, args.min_id, args.max_id)
        any_found_total = any_found_total or found
        if code == 2:
            return 2

    if any_found_total:
        return 0
    if args.allow_empty:
        print("[dxl-scan] no IDs found across all scans (allow-empty enabled)")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
