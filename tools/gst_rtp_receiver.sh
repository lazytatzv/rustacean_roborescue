#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_receiver.sh LISTEN_PORT
LISTEN_PORT=${1:-5002}

# Receiver pipeline for Opus RTP
# For bidirectional:
#   Robot: bash tools/gst_rtp_receiver.sh 5003
#   Operator: bash tools/gst_rtp_receiver.sh 5002

gst-launch-1.0 -v \
  udpsrc port=${LISTEN_PORT} caps="application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96" ! \
  rtpjitterbuffer latency=100 do-lost=true ! \
  rtpopusdepay ! opusdec ! audioconvert ! audioresample ! autoaudiosink
