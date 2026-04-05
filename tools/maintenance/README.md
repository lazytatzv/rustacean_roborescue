# Maintenance Scripts (uv managed)

This directory contains utility scripts for hardware bring-up and diagnostics.

## Setup

```bash
cd tools/maintenance
uv sync
```

Dependencies are managed by `uv`. If you are in the `nix develop` environment, `uv` is already available.

## Dynamixel ID scan

Scan flipper bus at 1 Mbps using both protocol versions:

```bash
cd tools/maintenance
uv run python dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols
```

If you want success exit code even when nothing is found (for iterative bring-up):

```bash
uv run python dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty
```

Sweep common baud rates:

```bash
uv run python dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000
```

Scan arm bus:

```bash
cd tools/maintenance
uv run python dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols
```

Exit codes:

- `0`: at least one ID found
- `1`: scan completed but no IDs found
- `2`: runtime error (open port / baud / invalid args)

Root-level Just recipes:

- `just dxl-scan-flipper` / `just dxl-scan-arm`
	- default scan, but non-fatal when no IDs are found
- `just dxl-scan-flipper-sweep` / `just dxl-scan-arm-sweep`
	- sweep common baud rates and both protocol versions
