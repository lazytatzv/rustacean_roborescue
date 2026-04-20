#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_sender.sh DEST_IP DEST_PORT
DEST_IP=${1:-127.0.0.1}
DEST_PORT=${2:-5002}

# Check if webrtcdsp exists
if gst-inspect-1.0 webrtcdsp >/dev/null 2>&1; then
    echo "Using WebRTC DSP for noise suppression..."
    DSP_PIPELINE="webrtcdsp noise-suppression-level=high echo-cancel=false gain-control=true target-level-dbfs=20"
else
    echo "WebRTC DSP not found. Using simple volume control..."
    DSP_PIPELINE="volume volume=0.1"
fi

# Simple low-latency Opus RTP sender
gst-launch-1.0 -v \
  pulsesrc ! audioconvert ! audioresample ! \
  ${DSP_PIPELINE} ! \
  opusenc complexity=10 bitrate=24000 audio-type=voice ! \
  rtpopuspay ! \
  udpsink host=${DEST_IP} port=${DEST_PORT} sync=false async=false
