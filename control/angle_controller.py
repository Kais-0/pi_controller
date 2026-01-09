
import time
import sys
import os

# Add parent directory (project root) to sys.path for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from test_camera import run_camera_loop
# ---------- CONFIG ----------

ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
ANGLE_START = 180.0

SEARCH_STEP = 15.0     # degrees when no target
TRACK_STEP = 5.0       # max degrees per update when tracking
DEADBAND = 0.02        # ignore small error

# ----------------------------


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def direction_from_error(e_x):
    """
    Returns:
        -1 → turn left
         0 → no movement
        +1 → turn right
    """
    if abs(e_x) < DEADBAND:
        return 0
    return 1 if e_x > 0 else -1


def main():
    angle = ANGLE_START

    print("Angle controller started.")
    print("Output = absolute servo angle (0–360°)")
    print("Press Ctrl+C to exit.\n")

    def on_measurement(meas):
        nonlocal angle

        if meas.target_found:
            direction = direction_from_error(meas.e_x)

            if direction != 0:
                # Step size scales with error magnitude
                step = min(TRACK_STEP, abs(meas.e_x) * TRACK_STEP * 10)
                angle += direction * step

                angle = clamp(angle, ANGLE_MIN, ANGLE_MAX)

            print(
                f"[TRACK] "
                f"e_x={meas.e_x:+.4f} | "
                f"dir={'R' if direction > 0 else 'L' if direction < 0 else '-'} | "
                f"angle={angle:7.2f}"
            )

        else:
            # No target → search
            angle += SEARCH_STEP
            if angle > ANGLE_MAX:
                angle = ANGLE_MIN

            print(
                f"[SEARCH] "
                f"angle={angle:7.2f}"
            )

    try:
        run_camera_loop(
            loop_dt=0.05,
            on_measurement=on_measurement,
        )
    except KeyboardInterrupt:
        print("\nAngle controller stopped.")


if __name__ == "__main__":
    main()