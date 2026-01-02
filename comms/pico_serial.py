# ~/robot/pi_controller/comms/pico_serial.py

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, List

import serial
from serial.tools import list_ports


@dataclass
class SerialConfig:
    baudrate: int = 115200
    timeout_s: float = 0.05          # read timeout
    write_timeout_s: float = 0.05
    reconnect_s: float = 1.0         # wait before reconnect attempts
    port: Optional[str] = None       # if None, auto-detect
    vid_pid_whitelist: Optional[List[tuple[int, int]]] = None  # e.g. [(0x2E8A,0x000A)]


class PicoSerial:
    """
    Simple line-based serial transport for Pi->Pico commands.

    - Open serial (explicit port or auto-detect).
    - send_velocity(v_cmd): writes "V,<float>\\n"
    - read_line(): non-blocking-ish line read with timeout
    - reconnect if link drops
    """

    def __init__(self, cfg: SerialConfig):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None
        self.port: Optional[str] = None

    def _auto_detect_port(self) -> str:
        ports = list(list_ports.comports())
        if not ports:
            raise RuntimeError("No serial ports found. Is the Pico connected over USB?")

        # Prefer whitelisted VID/PID if provided
        if self.cfg.vid_pid_whitelist:
            for p in ports:
                if p.vid is None or p.pid is None:
                    continue
                if (p.vid, p.pid) in self.cfg.vid_pid_whitelist:
                    return p.device

        # Otherwise prefer typical Pico USB CDC names
        # Usually /dev/ttyACM0, sometimes /dev/ttyUSB0 depending on firmware/bridge
        preferred = [p.device for p in ports if "ttyACM" in p.device] + \
                    [p.device for p in ports if "ttyUSB" in p.device]

        if preferred:
            return preferred[0]

        # Fallback: just take the first
        return ports[0].device

    def connect(self) -> None:
        if self.ser and self.ser.is_open:
            return

        port = self.cfg.port or self._auto_detect_port()
        self.port = port

        self.ser = serial.Serial(
            port=port,
            baudrate=self.cfg.baudrate,
            timeout=self.cfg.timeout_s,
            write_timeout=self.cfg.write_timeout_s,
        )

        # Give Pico time to reset/enumerate if your firmware resets on open.
        # If your Pico does NOT reset on open, you can reduce this.
        time.sleep(0.2)

        # Clear any junk
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def ensure_connected(self) -> None:
        if self.ser is None or not self.ser.is_open:
            self.connect()

    def send_velocity(self, v_cmd_mps: float) -> None:
        """
        Send chassis velocity command in m/s as: V,<float>\\n
        """
        self.ensure_connected()
        assert self.ser is not None

        line = f"V,{v_cmd_mps:.4f}\n"
        try:
            self.ser.write(line.encode("ascii"))
        except (serial.SerialException, OSError):
            # Link dropped: close and let caller retry next cycle
            self.close()
            raise

    def send_stop(self) -> None:
        self.ensure_connected()
        assert self.ser is not None
        try:
            self.ser.write(b"S\n")
        except (serial.SerialException, OSError):
            self.close()
            raise

    def read_line(self) -> Optional[str]:
        """
        Read one newline-terminated line if available.
        Returns None if nothing received before timeout.
        """
        self.ensure_connected()
        assert self.ser is not None

        try:
            raw = self.ser.readline()
        except (serial.SerialException, OSError):
            self.close()
            raise

        if not raw:
            return None

        try:
            return raw.decode("utf-8", errors="replace").strip()
        except Exception:
            return None


