import threading
import time
import math
import numpy as np
from mqtt_python.uservice import service

class DriveToPoint(threading.Thread):
    def __init__(self, world, target_x=None, target_y=None, path=None, final_h=None):
        """
        :param world: The WorldModel instance providing the pose
        :param target_x: X coordinate for single point or pose arrival
        :param target_y: Y coordinate for single point or pose arrival
        :param path: List of (x, y) tuples for curved path following
        :param final_h: Final heading in radians (Approach Vector)
        """
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Setup ---
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Controller Gains ---
        self.K_P_LIN = 0.5        # Speed based on distance
        self.K_P_ANG = 1.2        # Steering speed based on angle error
        self.DIST_TOL = 0.05      # 5cm arrival tolerance
        self.ANG_TOL = 0.08       # ~4.5 degree angle tolerance
        self.MAX_LIN = 0.5        # rc max 0.5
        self.MAX_ANG = 0.6        # rc max 0.6

        # --- Mission Configuration ---
        self.final_h = final_h
        if path:
            self.waypoints = path
            self.mission_type = "PATH"
        else:
            self.waypoints = [(target_x, target_y)]
            self.mission_type = "POINT"

        self.last_drive = 0.0
        self.last_turn = 0.0

    def normalize_angle(self, angle):
        """Wraps angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def calculate_drive_signals(self, tx, ty, is_intermediate=False):
        """Logic to calculate rc drive and turn values."""
        cx, cy, ch = self.world.get_pose()
        
        dx = tx - cx
        dy = ty - cy
        dist = math.sqrt(dx**2 + dy**2)
        
        # Determine tolerance based on whether we need to stop or just 'pass through'
        current_tol = 0.2 if is_intermediate else self.DIST_TOL
        
        if dist < current_tol:
            return 0.0, 0.0, True

        # Steering Logic
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - ch)

        # If pointing the wrong way, rotate in place first
        if abs(angle_error) > math.pi / 3:
            drive = 0.0
        else:
            drive = np.clip(dist * self.K_P_LIN, -self.MAX_LIN, self.MAX_LIN)
        
        turn = np.clip(angle_error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
        
        return round(float(drive), 2), round(float(turn), 2), False

    def align_to_approach_vector(self, target_h):
        """Final rotation to match the desired orientation."""
        print(f"Aligning to final heading: {target_h:.2f} rad")
        while self.running:
            _, _, ch = self.world.get_pose()
            angle_error = self.normalize_angle(target_h - ch)

            if abs(angle_error) < self.ANG_TOL:
                break

            turn = np.clip(angle_error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
            self.execute_rc_command(0.0, turn)
            time.sleep(0.05)

    def execute_rc_command(self, drive, turn):
        """Sends MQTT command only if the values have changed."""
        if drive != self.last_drive or turn != self.last_turn:
            service.send(self.MQTT_TOPIC_DRIVE, f"rc {drive:.1f} {turn:.1f} {time.time()}")
            self.last_drive, self.last_turn = drive, turn

    def run(self):
        print(f"Executing DriveToPoint: {self.mission_type}")
        
        # Iterate through coordinates
        for i, (tx, ty) in enumerate(self.waypoints):
            is_last = (i == len(self.waypoints) - 1)
            reached = False
            
            while self.running and not reached:
                # Use 'is_intermediate' to allow curved motion between path points
                d, t, reached = self.calculate_drive_signals(tx, ty, is_intermediate=not is_last)
                
                if not reached:
                    self.execute_rc_command(d, t)
                
                time.sleep(0.05) # 20Hz Loop

        # If a specific heading was requested (Approach Vector)
        if self.running and self.final_h is not None:
            self.align_to_approach_vector(self.final_h)

        self.stop_robot()
        print("DriveToPoint mission finished.")

    def stop_robot(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")

# --- How to use in your main script ---
# from drive_to_point import DriveToPoint

# Example 1: Single Point
# mission = DriveToPoint(world, target_x=1.0, target_y=0.5)

# Example 2: Curved Path
# mission = DriveToPoint(world, path=[(0.5, 0.5), (1.0, 0.0), (1.5, -0.5)])

# Example 3: Point with Approach Vector (Orientation)
# mission = DriveToPoint(world, target_x=0.0, target_y=0.0, final_h=1.57) # Face North

# mission.start()
