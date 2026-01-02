import sys
import time
from machine import Pin, PWM
import math

# ----------------------------
# REQUIRED CONSTANTS (EDIT)
# ----------------------------
R_WHEEL_M = 0.0325          # wheel radius [m]
W_MAX_RAD_S = 20.0          # max allowed wheel speed [rad/s] (SAFETY LIMIT)
PWM_FREQ_HZ = 20000         # Cytron supports up to 20 kHz standard :contentReference[oaicite:1]{index=1}

# Open-loop duty shaping (because no velocity feedback yet)
DEADBAND = 0.12             # minimum duty to overcome stiction (0..1)
DUTY_MAX = 0.80             # keep margin in early tests (0..1)

# ----------------------------
# WIRING / PIN MAP (EDIT)
# These are Pico GPIO numbers connected to:
#  - Cytron PWM1, DIR1 (Motor 1)
#  - Cytron PWM2, DIR2 (Motor 2)
# ----------------------------
PWM1_PIN = 20
DIR1_PIN = 21
PWM2_PIN = 18
DIR2_PIN = 19

# ----------------------------
# Hardware init
# ----------------------------
dir1 = Pin(DIR1_PIN, Pin.OUT)
dir2 = Pin(DIR2_PIN, Pin.OUT)

pwm1 = PWM(Pin(PWM1_PIN))
pwm2 = PWM(Pin(PWM2_PIN))
pwm1.freq(PWM_FREQ_HZ)
pwm2.freq(PWM_FREQ_HZ)

def clamp(x, lo, hi):
    if x < lo: return lo
    if x > hi: return hi
    return x

def duty_u16(duty_0_to_1: float) -> int:
    duty_0_to_1 = clamp(duty_0_to_1, 0.0, 1.0)
    return int(duty_0_to_1 * 65535)

def stop_all():
    # On SHIELD-MDD10: PWM low = Brake :contentReference[oaicite:2]{index=2}
    pwm1.duty_u16(0)
    pwm2.duty_u16(0)

def set_channel(pwm: PWM, dir_pin: Pin, cmd: float):
    """
    cmd in [-1, +1]
    sign -> DIR
    magnitude -> PWM duty
    """
    cmd = clamp(cmd, -1.0, 1.0)

    if abs(cmd) < 1e-4:
        pwm.duty_u16(0)      # brake
        return

    # Direction: choose convention. Flip these if direction is reversed.
    if cmd >= 0:
        dir_pin.value(1)
        mag = cmd
    else:
        dir_pin.value(0)
        mag = -cmd

    # Deadband + clamp
    mag = clamp(mag, 0.0, DUTY_MAX)
    mag = max(mag, DEADBAND)

    pwm.duty_u16(duty_u16(mag))

def parse_line(line: str):
    line = line.strip()
    if not line:
        return None
    if line == "S":
        return ("STOP", 0.0)
    if not line.startswith("V,"):
        return None
    try:
        v = float(line.split(",")[1])
        return ("VEL", v)
    except Exception:
        return None

def v_to_w(v_cmd_mps: float) -> float:
    return v_cmd_mps / R_WHEEL_M

def w_to_cmd(w_rad_s: float) -> float:
    # Open-loop normalization: cmd = w / W_MAX
    return clamp(w_rad_s / W_MAX_RAD_S, -1.0, 1.0)

print("[pico] Cytron MDD10 PWM+DIR ready. Expecting: V,<v_cmd> or S")
stop_all()

buf = ""

try:
    while True:
        ch = sys.stdin.read(1)  # blocking read; fine for this stage
        if not ch:
            continue

        if ch == "\n":
            msg = buf
            buf = ""

            parsed = parse_line(msg)
            if parsed is None:
                continue

            kind, v_cmd = parsed

            if kind == "STOP":
                stop_all()
                print("[pico] STOP -> PWM=0 (brake)")
                continue

            # Straight motion: same wheel command both sides (heading loop later)
            w = v_to_w(v_cmd)     # rad/s
            cmd = w_to_cmd(w)     # [-1,1]

            set_channel(pwm1, dir1, cmd)
            set_channel(pwm2, dir2, cmd)

            print(f"[pico] v={v_cmd:+.3f} m/s | w={w:+.3f} rad/s | cmd={cmd:+.3f}")

        else:
            buf += ch
            if len(buf) > 100:
                buf = ""
                stop_all()
                print("[pico] Line overflow -> STOP")

except KeyboardInterrupt:
    stop_all()
    print("[pico] Ctrl+C -> STOP")






# # Pico MicroPython: handshake test
# # Reads:  V,<v_cmd>\n  from Pi over USB serial (stdio)
# # Computes: w = v/r
# # Prints parsed values back.
# 
# import sys
# import time
# 
# # ---- SET THIS ----
# R_WHEEL_M = 0.0325   # wheel radius [m] (CHANGE THIS to your actual wheel radius)
# 
# def parse_v_line(line: str):
#     # Accept: "V,0.1234"
#     line = line.strip()
#     if not line:
#         return None
#     if line == "S":
#         return ("STOP", 0.0)
#     if not line.startswith("V,"):
#         return None
#     try:
#         v = float(line.split(",")[1])
#         return ("VEL", v)
#     except Exception:
#         return None
# 
# print("[pico] Handshake test ready. Expecting: V,<v_cmd>")
# 
# buf = ""
# last_print = time.ticks_ms()
# 
# while True:
#     # Non-blocking-ish read from USB serial
#     ch = sys.stdin.read(1)  # blocks until a char arrives
#     if ch:
#         if ch == "\n":
#             msg = buf
#             buf = ""
# 
#             parsed = parse_v_line(msg)
#             if parsed is None:
#                 print(f"[pico] Ignored: {msg}")
#                 continue
# 
#             kind, v_cmd = parsed
#             if kind == "STOP":
#                 print("[pico] STOP received -> v_cmd=0.0, w=0.0")
#                 continue
# 
#             w = v_cmd / R_WHEEL_M  # rad/s
#             print(f"[pico] v_cmd={v_cmd:+.4f} m/s -> w={w:+.4f} rad/s")
# 
#         else:
#             buf += ch
#             # Prevent runaway if host sends garbage without newline
#             if len(buf) > 100:
#                 print("[pico] Line too long -> cleared")
#                 buf = ""
