RTP Fallback (quick, LAN-only)
================================

When WebRTC/STUN/TURN is not available or you need the simplest guaranteed
connectivity on the same LAN, use the GStreamer RTP sender/receiver scripts.

How it works
- Robot runs a sender that encodes audio to Opus and streams RTP to a
  specific IP/port.
- Operator runs a receiver that listens on the port and plays audio.

Limitations
- Works only when operator IP is reachable from robot (no NAT traversal).
- No encryption, no authentication (for fastest setup).

Quickstart

On operator (listening):

```bash
# listen on port 5002 and play
bash tools/gst_rtp_receiver.sh 5002
```

On robot (sending to operator IP):

```bash
bash tools/gst_rtp_sender.sh 192.168.1.50 5002
```

Systemd
------
Edit `deploy/gst_rtp_sender.service` and set `DEST_IP` to operator IP, then:

```bash
sudo cp deploy/gst_rtp_sender.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now gst_rtp_sender.service
```

Similarly for `deploy/gst_rtp_receiver.service` on operator host.
