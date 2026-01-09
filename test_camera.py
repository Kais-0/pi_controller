#!/usr/bin/env python3

import time
from sensor.camera import YellowTrackerCamera


def run_camera_loop(
    loop_dt=0.05,
    on_measurement=None,
):
    """
    Runs the camera loop and optionally calls a callback
    with each measurement.

    Parameters:
        loop_dt (float): seconds between iterations
        on_measurement (callable): function(meas) -> None
    """
    cam = YellowTrackerCamera(
        width=640,
        height=480,
        fps=30,
        debug_view=False,
    )

    try:
        while True:
            meas = cam.read()

            # If a callback is provided, use it
            if on_measurement is not None:
                on_measurement(meas)
            else:
                # Default behavior (same as before)
                if meas.target_found:
                    print(
                        f"[FOUND] "
                        f"x={meas.x:7.1f} px | "
                        f"e_x={meas.e_x:+.4f} | "
                        f"dt={meas.dt*1000:6.1f} ms"
                    )
                else:
                    print(
                        f"[NONE ] "
                        f"e_x=0.0000 | "
                        f"dt={meas.dt*1000:6.1f} ms"
                    )

            time.sleep(loop_dt)

    finally:
        cam.close()


def main():
    print("Camera test started.")
    print("Move a YELLOW object left/right in front of the camera.")
    print("Press Ctrl+C to exit.\n")

    try:
        run_camera_loop()
    except KeyboardInterrupt:
        print("\nExiting camera test.")


if __name__ == "__main__":
    main()
