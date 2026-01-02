
# ~/robot/pi_controller/main.py
#
# Integrates:
#   sensor/vl53l0x.py   -> distance (m)
#   control/pid.py      -> v_cmd (m/s)
#   comms/pico_serial.py-> sends V,<v_cmd>
#
# Runs a fixed-rate outer loop on the Pi.
# Safety:
# - If ToF invalid for too long -> sends stop
# - Output saturated in PID
# - Prints debug at a throttled rate

from __future__ import annotations

import time
from dataclasses import dataclass

from control.pid import PID, PIDConfig
from comms.pico_serial import PicoSerial, SerialConfig

# Import your existing test function/module if you want, but better:
# You should wrap VL53L0X into a class with a read() method.
# For now we'll define a tiny reader here using your working code.

import board
import busio
import adafruit_vl53l0x
import math


@dataclass
class MainConfig:
    # Control target
    d_ref_m: float = 0.10          # target distance [m]

    # Loop timing
    loop_hz: float = 20.0          # outer loop frequency
    debug_hz: float = 5.0          # print debug frequency

    # ToF filtering (first-order LPF on distance)
    d_lpf_fc_hz: float = 3.0       # cutoff [Hz]

    # ToF validity handling
    out_of_range_mm: int = 8000
    drop_hold_s: float = 0.20      # max time to hold last valid distance
    stop_after_s: float = 0.20     # same as drop_hold_s, explicit

    # Velocity safety limit [m/s]
    v_limit: float = 0.20


class VL53L0XReader:
    def __init__(self, out_of_range_mm: int = 8000):
        self.out_of_range_mm = out_of_range_mm
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)

    def read_m(self) -> tuple[bool, float]:
        """Returns (valid, distance_m)."""
        mm = int(self.sensor.range)
        if mm <= 0 or mm > self.out_of_range_mm:
            return (False, math.nan)
        return (True, mm / 1000.0)


def lpf_alpha(fc_hz: float, dt: float) -> float:
    # alpha = exp(-2*pi*fc*dt) for first-order discrete LPF
    if fc_hz <= 0.0:
        return 0.0
    return math.exp(-2.0 * math.pi * fc_hz * dt)


def main() -> None:
    cfg = MainConfig()

    # --- Sensor ---
    tof = VL53L0XReader(out_of_range_mm=cfg.out_of_range_mm)

    # --- PID ---
    pid_cfg = PIDConfig(
        kp=1.2,
        ki=0.0,
        kd=0.15,
        u_min=-cfg.v_limit,
        u_max=cfg.v_limit,
        d_cutoff_hz=5.0,   # derivative filtering inside PID
        # optional integrator clamp if/when ki>0:
        i_min=None,
        i_max=None,
    )
    pid = PID(pid_cfg)

    # --- Comms ---
    link = PicoSerial(SerialConfig(baudrate=115200, port=None))
    link.connect()
    print(f"[main] Connected to Pico on {link.port}")

    # --- Timing ---
    Ts = 1.0 / cfg.loop_hz
    next_t = time.monotonic() + Ts
    last_t = time.monotonic()

    # --- Distance filter + validity ---
    d_f = None                 # filtered distance
    last_valid_d = None
    last_valid_t = None

    # --- Debug throttling ---
    debug_Ts = 1.0 / cfg.debug_hz
    next_debug_t = time.monotonic()

    print("[main] Running. Ctrl+C to stop.")
    try:
        while True:
            now = time.monotonic()
            dt = now - last_t
            last_t = now

            # 1) Read ToF
            valid, d = tof.read_m()

            if valid:
                last_valid_d = d
                last_valid_t = now

                # 2) Low-pass filter distance (measurement filtering)
                if d_f is None:
                    d_f = d
                else:
                    a = lpf_alpha(cfg.d_lpf_fc_hz, dt)
                    d_f = a * d_f + (1.0 - a) * d
            else:
                # 3) Handle dropout: hold last valid for a short time, else stop
                if last_valid_d is not None and last_valid_t is not None and (now - last_valid_t) <= cfg.drop_hold_s:
                    # Hold filtered value (no update) or hold last_valid_d — choose one
                    # We will hold d_f as-is.
                    pass
                else:
                    # Hard safety stop
                    link.send_stop()
                    if now >= next_debug_t:
                        print("[main] ToF invalid too long -> STOP")
                        next_debug_t = now + debug_Ts
                    # enforce timing and continue
                    sleep_s = next_t - time.monotonic()
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    next_t += Ts
                    continue

            if d_f is None:
                # Still no valid distance ever received; stay stopped
                link.send_stop()
                continue

            # 4) PID update: setpoint is d_ref, measurement is d_f
            v_cmd, e, i_state, d_state = pid.update(cfg.d_ref_m, d_f, dt)

            # 5) Send velocity command to Pico
            try:
                link.send_velocity(v_cmd)
            except Exception as ex:
                # If serial drops, stop trying to drive blindly
                # (You can also attempt reconnect here, but keep it simple.)
                print(f"[main] Serial error -> STOP. {ex}")
                try:
                    link.send_stop()
                except Exception:
                    pass

            # 6) Debug print (throttled)
            if now >= next_debug_t:
                print(
                    f"d_raw={'nan' if not valid else f'{d:.3f}'} m | "
                    f"d_f={d_f:.3f} m | "
                    f"e={e:+.3f} m | "
                    f"I={i_state:+.3f} | "
                    f"D={d_state:+.3f} | "
                    f"v_cmd={v_cmd:+.3f} m/s | "
                    f"dt={dt*1000:.1f} ms"
                )
                next_debug_t = now + debug_Ts

            # 7) Enforce loop rate
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            next_t += Ts

    except KeyboardInterrupt:
        print("\n[main] Ctrl+C -> STOP and exit.")
        try:
            link.send_stop()
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    main()

