# Opus-encoded audio packet for bidirectional voice communication
std_msgs/Header header
uint32 sample_rate   # Hz (e.g. 48000)
uint8  channels      # 1=mono, 2=stereo
uint8[] data         # Opus-encoded packet bytes
