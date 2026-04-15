import threading
import time
import math
import numpy as np
from mqtt_python.uservice import service

class DriveToTask(threading.Thread):
    def __init__(self, world, target_x=None, target_y=None, path=None, final_h=None):
        """
        :param world: The WorldModel instance
        :param target_x: X coordinate for single point or pose arrival
        :param target_y: Y coordinate for single point or pose arrival
        :param path: List of (x, y) tuples for curved path following
        :param final_h: Final heading in radians (Scenario 3)
        """
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Setup ---
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Controller Settings ---
        self.K_P_LIN = 0.5        # Linear speed gain
        self.K_P_ANG = 1.2        # Angular speed gain
        self.DIST_TOL = 0.05      # Arrival tolerance (5cm)
        self.ANG_TOL = 0.08       # Angle tolerance (~4.5 degrees)
        self.MAX_LIN = 0.5        # Max linear speed (rc 0.5)
        self.MAX_ANG = 0.6        # Max angular speed (rc 0.6)

        # --- Mission Logic ---
        self.final_h = final_h
        if path:
            self.waypoints = path
            self.mode = "PATH"
        else:
            self.waypoints = [(target_x, target_y)]
            self.mode = "POINT"

        self.last_drive = 0.0
        self.last_turn = 0.0

    def normalize_angle(self, angle):
        """Keep angle between -pi and pi."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def calculate_control(self, tx, ty, is_intermediate=False):
        """Calculates rc commands to reach a target coordinate."""
        cx, cy, ch = self.world.get_pose()
        
        dx = tx - cx
        dy = ty - cy
        dist = math.sqrt(dx**2 + dy**2)
        
        # If we are at the point, stop linear movement
        # Intermediate points have a larger tolerance for a "curved" feel
        current_tol = 0.2 if is_intermediate else self.DIST_TOL
        if dist < current_tol:
            return 0.0, 0.0, True

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - ch)

        # Logic: If the angle error is too large, turn in place first
        if abs(angle_error) > math.pi / 3:
            drive = 0.0
        else:
            # Scale linear speed by distance
            drive = np.clip(dist * self.K_P_LIN, -self.MAX_LIN, self.MAX_LIN)
        
        turn = np.clip(angle_error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
        
        return round(float(drive), 2), round(float(turn), 2), False

    def rotate_to_heading(self, target_h):
        """Rotates the robot in place to a specific heading."""
        print(f"Rotating to approach vector: {target_h}")
        while self.running:
            _, _, ch = self.world.get_pose()
            angle_error = self.normalize_angle(target_h - ch)

            if abs(angle_error) < self.ANG_TOL:
                break

            turn = np.clip(angle_error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
            self.send_drive(0.0, turn)
            time.sleep(0.05)

    def send_drive(self, drive, turn):
        """Gated MQTT sender to prevent flooding."""
        if drive != self.last_drive or turn != self.last_turn:
            # Timestamp included for robot command sync
            service.send(self.MQTT_TOPIC_DRIVE, f"rc {drive:.1f} {turn:.1f} {time.time()}")
            self.last_drive, self.last_turn = drive, turn

    def run(self):
        print(f"Starting Navigation Task: Mode={self.mode}")
        
        for i, (tx, ty) in enumerate(self.waypoints):
            is_last = (i == len(self.waypoints) - 1)
            reached = False
            
            print(f"Targeting: ({tx}, {ty})")
            
            while self.running and not reached:
                # Calculate commands. If is_intermediate=True, we use loose tolerance for curves.
                d, t, reached = self.calculate_control(tx, ty, is_intermediate=not is_last)
                
                if not reached:
                    self.send_drive(d, t)
                
                time.sleep(0.05) # ~20Hz control loop

        # Scenario 3: Final Heading (Approach Vector)
        if self.running and self.final_h is not None:
            self.rotate_to_heading(self.final_h)

        # Mission Complete
        self.stop()
        print("Navigation task complete.")

    def stop(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")

# --- Example Usage ---
# from core.world_model import WorldModel
# world = WorldModel()

# 1. Drive to X, Y
# task = DriveToTask(world, target_x=1.5, target_y=0.0)

# 2. Drive curved path
# path = [(0.5, 0.5), (1.0, 0.0), (1.5, 0.5)]
# task = DriveToTask(world, path=path)

# 3. Drive to X, Y with Approach Vector (e.g., face North / pi/2)
# task = DriveToTask(world, target_x=1.0, target_y=1.0, final_h=math.pi/2)

# task.start()
