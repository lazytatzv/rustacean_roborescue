# Maintenance Scripts (uv managed)

This directory contains utility scripts for hardware bring-up and diagnostics.

## Setup

```bash
cd tools/maintenance
uv sync
```

## Dynamixel ID scan

Scan flipper bus at 1 Mbps using both protocol versions:

```bash
cd tools/maintenance
uv run python dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols
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
