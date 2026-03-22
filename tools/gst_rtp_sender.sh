#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_sender.sh DEST_IP DEST_PORT
DEST_IP=${1:-127.0.0.1}
DEST_PORT=${2:-5002}

# Simple low-latency Opus RTP sender using Pulse/ALSA source
gst-launch-1.0 -v \
  pulsesrc ! audioconvert ! audioresample ! opusenc complexity=10 bitrate=64000 ! rtpopuspay ! \
  udpsink host=${DEST_IP} port=${DEST_PORT} sync=false async=false
