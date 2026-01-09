# ~/robot/pi_controller/main.py
#
# Outer-loop distance controller on Pi:
#   VL53L0X -> filtered distance d_f
#   PID(d_ref - d_f) -> v_cmd [m/s]
#   Serial -> Pico: "V,<v_cmd>\n" or "S\n"
#
# Actuation change (DC motor+driver -> continuous rotation servo) DOES NOT
# change this file's control responsibilities. The Pi still outputs v_cmd.
# The Pico will map v_cmd -> servo pulse width (speed command).
#
# Safety:
# - ToF dropout -> send STOP
# - PID output saturation (v_limit)
# - Throttled debug prints

from __future__ import annotations

import time
import math
from dataclasses import dataclass

from control.pid import PID, PIDConfig
from comms.pico_serial import PicoSerial, SerialConfig

import board
import busio
import adafruit_vl53l0x


@dataclass
class MainConfig:
    # Setpoint (distance from obstacle/wall) [m]
    d_ref_m: float = 0.30

    # Outer loop rate
    loop_hz: float = 20.0

    # Debug print rate
    debug_hz: float = 5.0

    # Distance low-pass filter cutoff [Hz]
    d_lpf_fc_hz: float = 3.0

    # ToF validity
    out_of_range_mm: int = 8000
    drop_hold_s: float = 0.20

    # Output limit to Pico (chassis velocity) [m/s]
    v_limit: float = 0.15

    # Serial settings
    baudrate: int = 115200


class VL53L0XReader:
    def __init__(self, out_of_range_mm: int = 8000):
        self.out_of_range_mm = out_of_range_mm
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)

    def read_m(self) -> tuple[bool, float]:
        mm = int(self.sensor.range)
        if mm <= 0 or mm > self.out_of_range_mm:
            return (False, math.nan)
        return (True, mm / 1000.0)


def lpf_alpha(fc_hz: float, dt: float) -> float:
    if fc_hz <=  0:
        return 1.0
    return math.exp(-2.0 * math.pi * fc_hz * dt)


def main() -> None:
    cfg = MainConfig()

    # Sensor
    tof = VL53L0XReader(out_of_range_mm=cfg.out_of_range_mm)

    # PID (output is v_cmd in m/s)
    pid_cfg = PIDConfig(
        kp=1.2,
        ki=0.0,
        kd=0.15,
        u_min=-cfg.v_limit,
        u_max=cfg.v_limit,
        d_cutoff_hz=5.0,
    )
    pid = PID(pid_cfg)

    # Serial to Pico
    link = PicoSerial(SerialConfig(baudrate=cfg.baudrate, port=None))
    link.connect()
    print(f"[main] Connected to Pico on {link.port}")

    # Timing
    Ts = 1.0 / cfg.loop_hz
    next_t = time.monotonic() + Ts
    last_t = time.monotonic()

    debug_Ts = 1.0 / cfg.debug_hz
    next_debug_t = time.monotonic()

    # Filter + validity state
    d_f = None
    last_valid_t = None

    print("[main] Running. Ctrl+C to stop.")
    try:
        while True:
            now = time.monotonic()
            dt = now - last_t
            last_t = now

            # 1) Read ToF
            valid, d = tof.read_m()

            if valid:
                last_valid_t = now

                # 2) Filter distance
                if d_f is None:
                    d_f = d
                else:
                    a = lpf_alpha(cfg.d_lpf_fc_hz, dt)
                    d_f = a * d_f + (1.0 - a) * d
            else:
                # 3) Dropout handling
                if (last_valid_t is None) or ((now - last_valid_t) > cfg.drop_hold_s):
                    # hard stop to Pico
                    link.send_stop()
                    if now >= next_debug_t:
                        print("[main] ToF invalid too long -> STOP")
                        next_debug_t = now + debug_Ts

                    # enforce loop timing and continue
                    sleep_s = next_t - time.monotonic()
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    next_t += Ts
                    continue
                # else: hold d_f (no update)

            if d_f is None:
                link.send_stop()
                continue

            # 4) PID -> v_cmd
            v_cmd, e, i_state, d_state = pid.update(cfg.d_ref_m, d_f, dt)

            # 5) Send v_cmd to Pico (Pico maps to servo PWM)
            try:
                link.send_velocity(v_cmd)
            except Exception as ex:
                print(f"[main] Serial error -> STOP. {ex}")
                try:
                    link.send_stop()
                except Exception:
                    pass

            # 6) Debug print
            if now >= next_debug_t:
                d_raw_str = "nan" if not valid else f"{d:.3f}"
                print(
                    f"d_raw={d_raw_str} m | d_f={d_f:.3f} m | "
                    f"e={e:+.3f} m | v_cmd={v_cmd:+.3f} m/s | "
                    f"I={i_state:+.3f} | D={d_state:+.3f} | dt={dt*1000:.1f} ms"
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



