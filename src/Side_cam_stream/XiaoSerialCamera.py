import time
from typing import Generator, Optional

import serial
from serial.tools import list_ports
import cv2
import numpy as np


class XiaoSerialCamera:
    """
    XIAO ESP32S3 camera over USB-serial.

    Protocol (from Arduino side):
      - Each frame:
        1. A text line: "FRAME <len>\\n"
        2. Then <len> bytes of JPEG image data.

    This class:
      - Auto-detects COM port (with optional preferred port).
      - Automatically reconnects if no valid frame is received for some time.
      - Provides:
          - read_frame() -> np.ndarray | None
          - frames() -> generator of frames
    """

    def __init__(
        self,
        preferred_port: Optional[str] = "COM3",
        baud: int = 2000000,
        no_frame_timeout: float = 3.0,
        serial_timeout: float = 1.0,
    ) -> None:
        """
        :param preferred_port: COM port to try first, e.g. "COM3" or "/dev/ttyACM0".
        :param baud: Must match SERIAL_BAUD in your Arduino firmware.
        :param no_frame_timeout: If no valid frame is received for this many seconds,
                                 the class will try to reconnect the serial port.
        :param serial_timeout: Timeout in seconds for serial read operations.
        """
        self.preferred_port = preferred_port
        self.baud = baud
        self.no_frame_timeout = no_frame_timeout
        self.serial_timeout = serial_timeout

        self.ser: Optional[serial.Serial] = None
        self._last_good_frame_time = time.time()

        # Open serial on creation
        self._open_serial()

    # ---------- Public API ----------

    def read_frame(self) -> Optional[np.ndarray]:
        """
        Read a single frame from the camera.

        Returns:
            - Decoded BGR image (np.ndarray) if successful
            - None if no valid frame was received

        This will also handle auto-reconnect if no valid frame is received
        for longer than `no_frame_timeout`.
        """
        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            if self.ser is None:
                return None

        frame = self._read_frame_once(self.ser)
        now = time.time()

        if frame is not None:
            self._last_good_frame_time = now
            return frame

        # No valid frame this time; check timeout
        if now - self._last_good_frame_time > self.no_frame_timeout:
            print("[XiaoSerialCamera] No frame for a while, reconnecting...")
            self._reconnect()
        return None

    def frames(self) -> Generator[np.ndarray, None, None]:
        """
        Infinite generator of frames.

        Usage:
            cam = XiaoSerialCamera()
            for frame in cam.frames():
                # process frame
        """
        try:
            while True:
                frame = self.read_frame()
                if frame is not None:
                    yield frame
                # Small sleep to avoid busy-wait if nothing is received
                time.sleep(0.001)
        finally:
            self.close()

    def close(self) -> None:
        """Close the serial port and release resources."""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ---------- Internal helpers ----------

    def _auto_find_port(self) -> Optional[str]:
        """Automatically find a likely XIAO ESP32S3 serial port."""
        ports = list(list_ports.comports())
        if not ports:
            print("[XiaoSerialCamera] No serial ports found.")
            return None

        # 1) Try preferred port
        if self.preferred_port:
            for p in ports:
                if p.device == self.preferred_port:
                    print(f"[XiaoSerialCamera] Using preferred port: {self.preferred_port}")
                    return self.preferred_port
            print(f"[XiaoSerialCamera] Preferred port {self.preferred_port} not found, auto-detecting...")

        # 2) Look for 'XIAO' in description
        for p in ports:
            desc = (p.description or "").upper()
            if "XIAO" in desc:
                print(f"[XiaoSerialCamera] Detected XIAO port: {p.device} ({p.description})")
                return p.device

        # 3) Look for generic USB serial
        for p in ports:
            desc = (p.description or "").upper()
            if "USB-SERIAL" in desc or "USB SERIAL" in desc:
                print(f"[XiaoSerialCamera] Using USB serial port: {p.device} ({p.description})")
                return p.device

        # 4) Fallback: use the first one
        p = ports[0]
        print(f"[XiaoSerialCamera] Using first available port: {p.device} ({p.description})")
        return p.device

    def _open_serial(self) -> None:
        """Open serial port with auto-detection."""
        port = self._auto_find_port()
        if port is None:
            self.ser = None
            return

        print(f"[XiaoSerialCamera] Opening {port} at {self.baud} baud...")
        ser = serial.Serial(port, self.baud, timeout=self.serial_timeout)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self.ser = ser
        self._last_good_frame_time = time.time()

    def _reconnect(self) -> None:
        """Close and reopen the serial port."""
        self.close()
        time.sleep(1.0)
        self._open_serial()

    @staticmethod
    def _read_frame_once(ser: serial.Serial) -> Optional[np.ndarray]:
        """
        Low-level function to read one frame from a given serial port.

        Returns:
            np.ndarray frame or None on failure.
        """
        try:
            header = ser.readline().decode(errors="ignore").strip()
            if not header.startswith("FRAME "):
                return None

            # Parse length
            try:
                length = int(header.split(" ")[1])
            except (IndexError, ValueError):
                return None

            # Read JPEG payload
            data = b""
            while len(data) < length:
                chunk = ser.read(length - len(data))
                if not chunk:
                    break
                data += chunk

            if len(data) != length:
                return None

            # Decode JPEG with OpenCV
            img_array = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return img

        except Exception as e:
            print(f"[XiaoSerialCamera] Error while reading frame: {e}")
            return None


# ---------- Example usage: display stream with OpenCV ----------

if __name__ == "__main__":
    cam = XiaoSerialCamera(
        preferred_port="COM3",  # Change if needed, or set to None to fully auto-detect
        baud=2000000,
        no_frame_timeout=3.0,
        serial_timeout=1.0,
    )

    try:
        for frame in cam.frames():
            cv2.imshow("XIAO ESP32S3 Camera", frame)
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
