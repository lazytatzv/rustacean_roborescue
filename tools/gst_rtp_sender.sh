#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_sender.sh DEST_IP DEST_PORT
DEST_IP=${1:-127.0.0.1}
DEST_PORT=${2:-5002}

# Simple low-latency Opus RTP sender with Noise Suppression
# For bidirectional:
#   Robot: bash tools/gst_rtp_sender.sh OPERATOR_IP 5002
#   Operator: bash tools/gst_rtp_sender.sh ROBOT_IP 5003

gst-launch-1.0 -v \
  pulsesrc ! audioconvert ! audioresample ! \
  webrtcdsp noise-suppression-level=high echo-cancel=false gain-control=true target-level-dbfs=20 ! \
  opusenc complexity=10 bitrate=24000 audio-type=voice ! \
  rtpopuspay ! \
  udpsink host=${DEST_IP} port=${DEST_PORT} sync=false async=false
