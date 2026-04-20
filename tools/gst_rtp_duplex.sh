#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   Robot side:    bash tools/gst_rtp_duplex.sh OPERATOR_IP --robot
#   Operator side: bash tools/gst_rtp_duplex.sh ROBOT_IP --operator

REMOTE_IP=${1:-""}
MODE=${2:-"--robot"}

if [ -z "$REMOTE_IP" ]; then
    echo "Usage: bash tools/gst_rtp_duplex.sh <REMOTE_IP> [--robot|--operator]"
    exit 1
fi

if [ "$MODE" == "--robot" ]; then
    SEND_PORT=5002
    RECV_PORT=5003
    echo "--- MODE: ROBOT ---"
else
    SEND_PORT=5003
    RECV_PORT=5002
    echo "--- MODE: OPERATOR ---"
fi

echo "Sending to   : ${REMOTE_IP}:${SEND_PORT}"
echo "Listening on : ${RECV_PORT}"
echo "Press Ctrl+C to stop both."

# 1. Start Receiver in background
bash "$(dirname "$0")/gst_rtp_receiver.sh" "${RECV_PORT}" &
RECV_PID=$!

# 2. Start Sender in background
bash "$(dirname "$0")/gst_rtp_sender.sh" "${REMOTE_IP}" "${SEND_PORT}" &
SENDER_PID=$!

# Wait for Ctrl+C
trap "kill $RECV_PID $SENDER_PID 2>/dev/null || true; exit" SIGINT SIGTERM
wait
