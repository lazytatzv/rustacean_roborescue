#!/bin/bash

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo "==========================================="
echo "USB Device List (lsusb)"
echo "==========================================="
if command -v lsusb >/dev/null 2>&1; then
    lsusb
else
    # lsusbがない場合の代替表示
    for dev in /sys/bus/usb/devices/*; do
        if [ -f "$dev/idVendor" ]; then
            vid=$(cat "$dev/idVendor")
            pid=$(cat "$dev/idProduct")
            bus=$(cat "$dev/busnum")
            devnum=$(cat "$dev/devnum")
            prod=$(cat "$dev/product" 2>/dev/null || echo "")
            manu=$(cat "$dev/manufacturer" 2>/dev/null || echo "")
            printf "Bus %03d Device %03d: ID %s:%s %s %s\n" "$bus" "$devnum" "$vid" "$pid" "$manu" "$prod"
        fi
    done
fi

echo ""
echo "==========================================="
echo "Device Mapping (/dev -> Physical Device)"
echo "==========================================="

# マッピングを確認する関数
check_dev() {
    local symlink=$1
    local description=$2
    if [ -L "$symlink" ]; then
        local target=$(readlink -f "$symlink")
        # udevadmから情報を取得
        local vid=$(udevadm info -a -n "$target" 2>/dev/null | grep 'ATTRS{idVendor}=="' | head -n1 | sed -e 's/.*"\(.*\)".*/\1/')
        local pid=$(udevadm info -a -n "$target" 2>/dev/null | grep 'ATTRS{idProduct}=="' | head -n1 | sed -e 's/.*"\(.*\)".*/\1/')
        local serial=$(udevadm info -a -n "$target" 2>/dev/null | grep 'ATTRS{serial}=="' | head -n1 | sed -e 's/.*"\(.*\)".*/\1/')
        
        printf "  %-22s -> ${GREEN}%-10s${NC} [ID %s:%s] %s\n" "$symlink" "$(basename "$target")" "$vid" "$pid" "($description)"
        if [ ! -z "$serial" ]; then
            echo "                         Serial: $serial"
        fi
    else
        printf "  %-22s -> ${RED}NOT FOUND${NC}  (%s)\n" "$symlink" "$description"
    fi
}

echo "[Serial Devices]"
check_dev "/dev/roboclaw" "Roboclaw Motor Controller"
check_dev "/dev/dynamixel_flipper" "Dynamixel Flipper Bus"
check_dev "/dev/dynamixel_arm" "Dynamixel Arm Bus"

echo ""
echo "[Cameras]"
check_dev "/dev/camera_front_left" "Front Left Camera"
check_dev "/dev/camera_front_right" "Front Right Camera"
check_dev "/dev/camera_arm" "Arm Tip Camera"
check_dev "/dev/camera_realsense" "Intel RealSense"
check_dev "/dev/camera_ricoh" "Ricoh 360 Camera"
check_dev "/dev/camera_logitech" "Logitech Back Camera"

echo "==========================================="
