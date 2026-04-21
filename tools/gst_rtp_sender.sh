#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_sender.sh DEST_IP DEST_PORT
DEST_IP=${1:-127.0.0.1}
DEST_PORT=${2:-5002}

# Detect best available noise cancellation
if gst-inspect-1.0 webrtcdsp >/dev/null 2>&1; then
    echo "Using WebRTC DSP..."
    FILTER="webrtcdsp noise-suppression-level=very-high echo-cancel=false gain-control=true target-level-dbfs=15"
elif gst-inspect-1.0 speexdenoise >/dev/null 2>&1; then
    echo "Using Speex Denoise..."
    FILTER="speexdenoise noise-suppression=-30 ! volume volume=2.0"
else
    echo "No DSP found. Using aggressive volume reduction..."
    FILTER="volume volume=0.03"
fi

# Simple low-latency Opus RTP sender with enhanced filtering
gst-launch-1.0 -v \
  pulsesrc ! audioconvert ! audioresample ! \
  ${FILTER} ! \
  audio/x-raw,rate=16000 ! \
  opusenc complexity=10 bitrate=20000 audio-type=voice ! \
  rtpopuspay ! \
  udpsink host=${DEST_IP} port=${DEST_PORT} sync=false async=false
