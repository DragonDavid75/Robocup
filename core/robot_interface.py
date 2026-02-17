# core/robot_interface.py
from uservice import service

class RobotInterface:

    def __init__(self):
        if not service.connected:
            service.setup('localhost')

    # --- Motion commands ---
    def send_velocity(self, v, w):
        service.send("robobot/cmd/ti", f"rc {v} {w}")

    def stop(self):
        self.send_velocity(0.0, 0.0)

    # --- Servo ---
    def set_servo(self, idx, pos, speed):
        service.send("robobot/cmd/T0", f"servo {idx} {pos} {speed}")

    # --- LEDs ---
    def set_led(self, r, g, b):
        service.send("robobot/cmd/T0", f"leds 16 {r} {g} {b}")

    # --- Shutdown ---
    def terminate(self):
        service.terminate()
