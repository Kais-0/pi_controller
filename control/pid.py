
# ~/robot/pi_controller/control/pid.py
# Discrete-time PID for outer-loop distance control (Pi side)
#
# Features included (non-negotiable for real robots):
# - measured dt (caller passes dt each update)
# - output saturation
# - integrator anti-windup (conditional integration / clamping)
# - derivative-on-error with optional low-pass filtering on derivative
#
# Notes:
# - Units must be consistent:
#   e [m], dt [s], output u [m/s]  => Kp [1/s], Ki [1/s^2], Kd [dimensionless]
#   (This is why you MUST commit to physical units for command.)
#
# - If you later switch to derivative-on-measurement, do it consciously.
#   For now: D term uses error derivative but filtered.

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import math


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float

    # Output limits (e.g., velocity command limits) [m/s]
    u_min: float = -0.25
    u_max: float = 0.25

    # Derivative low-pass filter cutoff frequency [Hz]
    # Set to 0.0 to disable derivative filtering.
    d_cutoff_hz: float = 5.0

    # Integrator clamp (optional): hard bounds on integral state.
    # If None, no explicit integral clamp is applied beyond anti-windup logic.
    i_min: Optional[float] = None
    i_max: Optional[float] = None

    # Safety: dt bounds (reject nonsense)
    dt_min: float = 1e-4   # 0.1 ms
    dt_max: float = 0.5    # 500 ms


class PID:
    """
    Discrete PID:
      e = r - y
      I <- I + e*dt    (with anti-windup)
      D <- filtered((e - e_prev)/dt)

      u_raw = kp*e + ki*I + kd*D
      u = clamp(u_raw, u_min, u_max)

    Anti-windup implemented via conditional integration (integrator clamping):
      - If output not saturated: integrate normally
      - If saturated: integrate only if integration would move u_raw back toward unsaturation
        i.e., sign(e) opposes sign(u_raw) when saturated.

    Derivative filter:
      First-order low-pass applied to derivative estimate:
        D_f[k] = alpha*D_f[k-1] + (1-alpha)*D[k]
      with alpha = exp(-2*pi*f_c*dt)
    """

    def __init__(self, cfg: PIDConfig):
        if cfg.u_min >= cfg.u_max:
            raise ValueError("u_min must be < u_max")
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._i = 0.0
        self._e_prev: Optional[float] = None
        self._d_f = 0.0
        self._u_last = 0.0

    @property
    def integral(self) -> float:
        return self._i

    @property
    def last_output(self) -> float:
        return self._u_last

    def update(self, setpoint: float, measurement: float, dt: float) -> Tuple[float, float, float, float]:
        """
        Returns: (u, e, i_state, d_state)
        """
        # dt sanity
        if not (self.cfg.dt_min <= dt <= self.cfg.dt_max):
            # Hard stop behavior for insane dt: return last output but don't update state
            # You may choose to return 0.0 for safety; last output is useful for debugging.
            return (self._u_last, setpoint - measurement, self._i, self._d_f)

        e = setpoint - measurement

        # Derivative term (on error)
        if self._e_prev is None:
            d = 0.0
        else:
            d = (e - self._e_prev) / dt

        # Filter derivative if requested
        if self.cfg.d_cutoff_hz and self.cfg.d_cutoff_hz > 0.0:
            alpha = math.exp(-2.0 * math.pi * self.cfg.d_cutoff_hz * dt)
            self._d_f = alpha * self._d_f + (1.0 - alpha) * d
            d_used = self._d_f
        else:
            self._d_f = d
            d_used = d

        # Compute raw output WITHOUT updating integrator yet
        u_no_i = self.cfg.kp * e + self.cfg.kd * d_used
        u_raw = u_no_i + self.cfg.ki * self._i

        # Saturate
        u_sat = clamp(u_raw, self.cfg.u_min, self.cfg.u_max)

        # Anti-windup: conditional integration
        # If not saturated, integrate.
        # If saturated, integrate only if it will reduce saturation (drive u_raw back toward bounds).
        saturated = (u_raw != u_sat)

        integrate = False
        if not saturated:
            integrate = True
        else:
            # If u_raw is above max, we only integrate if e is negative (would pull u_raw down).
            # If u_raw is below min, we only integrate if e is positive (would pull u_raw up).
            if u_raw > self.cfg.u_max and e < 0.0:
                integrate = True
            elif u_raw < self.cfg.u_min and e > 0.0:
                integrate = True

        if integrate and self.cfg.ki != 0.0:
            self._i += e * dt

            # Optional integral clamp
            if self.cfg.i_min is not None and self._i < self.cfg.i_min:
                self._i = self.cfg.i_min
            if self.cfg.i_max is not None and self._i > self.cfg.i_max:
                self._i = self.cfg.i_max

            # Recompute with updated integrator and resaturate (keeps internal consistency)
            u_raw = u_no_i + self.cfg.ki * self._i
            u_sat = clamp(u_raw, self.cfg.u_min, self.cfg.u_max)

        # Update state
        self._e_prev = e
        self._u_last = u_sat

        return (u_sat, e, self._i, d_used)

    def debug_str(self, setpoint: float, measurement: float, dt: float) -> str:
        u, e, i, d = self.update(setpoint, measurement, dt)
        return (
            f"e={e:+.4f} m | "
            f"I={i:+.4f} | "
            f"D={d:+.4f} | "
            f"u={u:+.4f} m/s | "
            f"dt={dt*1000:.1f} ms"
            )



