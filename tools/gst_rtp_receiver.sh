#!/usr/bin/env bash
set -euo pipefail
# Usage: gst_rtp_receiver.sh LISTEN_PORT
LISTEN_PORT=${1:-5002}

# Receiver pipeline for Opus RTP
gst-launch-1.0 -v \
  udpsrc port=${LISTEN_PORT} caps="application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96" ! \
  rtpopusdepay ! opusdec ! audioconvert ! audioresample ! autoaudiosink
