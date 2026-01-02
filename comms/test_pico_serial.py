import time
from pico_serial import PicoSerial, SerialConfig

cfg = SerialConfig(baudrate=115200, port=None)
link = PicoSerial(cfg)

link.connect()
print(f"Connected on {link.port}")

# Send a few commands
for v in [0.0, 0.05, 0.10, 0.05, 0.0]:
    print("Sending:", v)
    link.send_velocity(v)
    time.sleep(0.2)

# Optional: stop command
link.send_stop()
print("Stop sent.")


