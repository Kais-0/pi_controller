# sensor/camera.py
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError as e:
    raise ImportError(
        "Picamera2 not installed. Install with: sudo apt install -y python3-picamera2"
    ) from e


@dataclass
class CameraMeasurement:
    t: float
    dt: float
    width: int
    height: int
    target_found: bool
    x: Optional[float]
    e_x: float  # normalized [-1, 1]


class YellowTrackerCamera:
    """
    Driver for Pi Camera Module 3:
    - Captures frames
    - Detects yellow blob
    - Outputs centroid x and normalized horizontal error e_x
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        # HSV thresholds MUST be tuned in your lighting
        hsv_low: Tuple[int, int, int] = (20, 100, 100),
        hsv_high: Tuple[int, int, int] = (35, 255, 255),
        min_area_px: int = 250,
        debug_view: bool = False,
    ):
        self.width = width
        self.height = height
        self.hsv_low = np.array(hsv_low, dtype=np.uint8)
        self.hsv_high = np.array(hsv_high, dtype=np.uint8)
        self.min_area_px = int(min_area_px)
        self.debug_view = bool(debug_view)

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps},
        )
        self.picam2.configure(config)
        self.picam2.start()

        self._t_prev = time.perf_counter()

    def read(self) -> CameraMeasurement:
        # Measure dt for REAL (do not assume controller dt)
        t_now = time.perf_counter()
        dt = t_now - self._t_prev
        self._t_prev = t_now

        frame = self.picam2.capture_array()  # RGB
        h, w = frame.shape[:2]

        # Convert RGB -> BGR for OpenCV, then HSV
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        # Clean noise (tune as needed)
        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_found = False
        x_centroid = None
        e_x = 0.0

        if contours:
            # Largest blob by area
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area >= self.min_area_px:
                M = cv2.moments(c)
                if M["m00"] > 1e-6:
                    x_centroid = float(M["m10"] / M["m00"])
                    target_found = True

                    x0 = w / 2.0
                    e_x = (x_centroid - x0) / x0  # normalized [-1,1]

                    # Optional debug overlay
                    if self.debug_view:
                        cy = float(M["m01"] / M["m00"])
                        cv2.circle(bgr, (int(x_centroid), int(cy)), 6, (0, 0, 255), -1)
                        cv2.line(bgr, (int(x0), 0), (int(x0), h - 1), (255, 255, 255), 1)
                        cv2.putText(
                            bgr,
                            f"e_x={e_x:+.3f} area={int(area)} dt={dt*1000:.1f}ms",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                        cv2.imshow("camera", bgr)
                        cv2.imshow("mask", mask)
                        cv2.waitKey(1)

        return CameraMeasurement(
            t=t_now,
            dt=dt,
            width=w,
            height=h,
            target_found=target_found,
            x=x_centroid,
            e_x=float(np.clip(e_x, -1.0, 1.0)),
        )

    def close(self):
        self.picam2.stop()
        if self.debug_view:
            cv2.destroyAllWindows()
