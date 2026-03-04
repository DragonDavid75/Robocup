# core/robot_interface.py
from mqtt_python.spose import pose
from mqtt_python.uservice import service
from mqtt_python.sir import ir
from mqtt_python.srobot import robot
from mqtt_python.sedge import edge
from mqtt_python.sgpio import gpio
from mqtt_python.scam import cam
from mqtt_python.simu import imu
import time as t

class RobotInterface:

    def __init__(self, host='localhost'):
        if not service.connected:
            service.setup(host)

    # --- Motion & Control ---
    def set_velocity(self, linear_v, angular_w):
        """Sets forward velocity (m/s) and turn rate (rad/s)."""
        service.send("robobot/cmd/ti", f"rc {linear_v} {angular_w}")

    def stop(self):
        """Stops the robot movement and resets line control."""
        self.set_line_control(0, False)
        self.set_velocity(0.0, 0.0)

    # --- Line Following (Edge) ---
    def set_line_control(self, velocity, follow_left=True, ref_pos=0.0):
        """
        Enables/Disables built-in line following.
        velocity: speed while following.
        follow_left: True to follow left edge,
                     False to follow right edge.
        ref_pos: desired position of the edge (0=centered).
        """
        edge.lineControl(velocity, follow_left, ref_pos)

    def get_line_data(self):
        """Returns current edge positions and line validity count."""
        return {
            "left": edge.posLeft,
            "right": edge.posRight,
            "valid_cnt": edge.lineValidCnt
        }

    # --- Sensors (IR, IMU, Pose) ---
    def get_ir_distances(self):
        """Returns array of IR sensor distances."""
        return ir.ir

    def get_pose(self):
        """Returns current x, y, heading (h)."""
        return {"x": pose.x, "y": pose.y, "h": pose.h}

    def get_odometry(self):
        """Returns trip distance (tripB) and trip heading change (tripBh)."""
        return {"dist": pose.tripB, "angle": pose.tripBh, "time": pose.tripBtime}

    def reset_trip(self):
        """Resets trip meters/timers."""
        pose.tripBreset()

    def get_imu_data(self):
        """Returns gyro and accelerometer data."""
        return {"gyro": imu.gyro, "acc": imu.acc}

    # --- Actuators (Servos & LEDs) ---
    def set_servo(self, idx, pos, speed):
        """
        Sets servo position.
        Example: idx=1, pos=-800 (up), speed=300 (slow)
        """
        service.send("robobot/cmd/T0", f"servo {idx} {pos} {speed}")

    def set_led(self, led_idx, r, g, b):
        """Sets RGB value for a specific LED (e.g., 16)."""
        service.send("robobot/cmd/T0", f"leds {led_idx} {r} {g} {b}")

    # --- System & Logging ---
    def log_now(self, duration):
        """Tells the Teensy to log data for a specific duration."""
        service.send("robobot/cmd/T0/", f"lognow {duration}")

    def set_interface_logging(self, enable):
        """Enables/disables logging in the teensy_interface."""
        val = 1 if enable else 0
        service.send("robobot/cmd/ti", f"log {val}")

    def check_stop_button(self):
        """Returns True if the physical stop button is pressed."""
        return gpio.test_stop_button()

    def is_running(self):
        """Checks if the service stop flag has been set."""
        return not service.stop

    # --- Camera ---
    def get_image(self):
        """Captures image from the camera module."""
        if cam.useCam:
            return cam.getImage()
        return False, None, None

    # --- Shutdown ---
    def terminate(self):
        """Full system shutdown."""
        self.stop()
        service.terminate()