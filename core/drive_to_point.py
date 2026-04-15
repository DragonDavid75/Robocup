import threading
import time
import math
import numpy as np
from mqtt_python.uservice import service

class DriveToPoint(threading.Thread):
    def __init__(self, world, target_x=0.0, target_y=0.0, path=None, final_h=None):
        """
        All coordinates are relative to the robot's current position when called.
        :param world: WorldModel instance providing get_pose() -> (x, y, h)
        :param target_x: Relative forward distance in meters
        :param target_y: Relative sideways distance in meters (Left is positive)
        :param path: Optional list of (x, y) tuples for curved movement
        :param final_h: Relative heading in radians (e.g., math.pi/2 is 90 deg left)
        """
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Topics ---
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Controller Parameters ---
        self.K_P_LIN = 0.5        # Speed gain
        self.K_P_ANG = 1.2        # Steering gain
        self.DIST_TOL = 0.05      # Arrival tolerance (5cm)
        self.ANG_TOL = 0.08       # Angle tolerance (~4.5 degrees)
        self.MAX_LIN = 0.5        # Max drive speed
        self.MAX_ANG = 0.6        # Max turn speed

        # --- Mission Data ---
        self.final_h = final_h
        self.waypoints = path if path else [(target_x, target_y)]
        
        # Snapshot of the global frame when the mission begins
        self.origin_pose = (0.0, 0.0, 0.0) 
        self.last_drive = 0.0
        self.last_turn = 0.0

    def get_relative_pose(self):
        """
        Calculates the current pose relative to the start orientation.
        Transforms Global coordinates into the Local frame.
        """
        # Current Global Position
        gx, gy, gh = self.world.get_pose()
        # Starting Global Position (the Origin)
        ox, oy, oh = self.origin_pose

        # 1. Global Difference
        dx = gx - ox
        dy = gy - oy

        # 2. Rotation Matrix Transformation
        # Rotates the global coordinates back by the starting angle 'oh'
        # so that 'forward' is always the robot's initial X-axis.
        rel_x = dx * math.cos(-oh) - dy * math.sin(-oh)
        rel_y = dx * math.sin(-oh) + dy * math.cos(-oh)
        
        # SWAPPED assignment (if X and Y are interchanged)
        # rel_x = dx * math.sin(-oh) + dy * math.cos(-oh)
        # rel_y = dx * math.cos(-oh) - dy * math.sin(-oh)

        # To flip an axis, multiply the entire result by -1
        # rel_x = -1 * (dx * math.cos(-oh) - dy * math.sin(-oh)) # Flipped Forward/Backward
        # rel_y = -1 * (dx * math.sin(-oh) + dy * math.cos(-oh)) # Flipped Left/Right
        
        # 3. Relative Heading
        rel_h = (gh - oh + math.pi) % (2 * math.pi) - math.pi
        # To this (invert the subtraction order):
        # rel_h = (oh - gh + math.pi) % (2 * math.pi) - math.pi

        return rel_x, rel_y, rel_h

    def calculate_signals(self, tx, ty, is_intermediate=False):
        """Standard P-Controller for relative navigation."""
        rx, ry, rh = self.get_relative_pose()
        
        dx = tx - rx
        dy = ty - ry
        dist = math.sqrt(dx**2 + dy**2)
        
        # Use 20cm tolerance for points in a path to allow 'curved' flow
        tol = 0.2 if is_intermediate else self.DIST_TOL
        if dist < tol:
            return 0.0, 0.0, True

        target_angle = math.atan2(dy, dx)
        angle_error = (target_angle - rh + math.pi) % (2 * math.pi) - math.pi

        # Rotate in place if the point is more than 60 degrees away
        if abs(angle_error) > math.pi / 3:
            drive = 0.0
        else:
            drive = np.clip(dist * self.K_P_LIN, -self.MAX_LIN, self.MAX_LIN)
        
        turn = np.clip(angle_error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
        
        return round(float(drive), 2), round(float(turn), 2), False

    def align_final_heading(self, target_rh):
        """Final rotation to reach the desired approach vector."""
        while self.running:
            _, _, rh = self.get_relative_pose()
            error = (target_rh - rh + math.pi) % (2 * math.pi) - math.pi

            if abs(error) < self.ANG_TOL:
                break

            turn = np.clip(error * self.K_P_ANG, -self.MAX_ANG, self.MAX_ANG)
            self.send_rc(0.0, turn)
            time.sleep(0.05)

    def send_rc(self, drive, turn):
        """Publishes to MQTT only on change."""
        if drive != self.last_drive or turn != self.last_turn:
            service.send(self.MQTT_TOPIC_DRIVE, f"rc {drive:.1f} {turn:.1f} {time.time()}")
            self.last_drive, self.last_turn = drive, turn

    def run(self):
        # Capture the current global state as (0,0,0) for this mission
        self.origin_pose = self.world.get_pose()
        
        for i, (tx, ty) in enumerate(self.waypoints):
            is_last = (i == len(self.waypoints) - 1)
            reached = False
            
            while self.running and not reached:
                d, t, reached = self.calculate_signals(tx, ty, is_intermediate=not is_last)
                if not reached:
                    self.send_rc(d, t)
                time.sleep(0.05)

        # After points are reached, check if we need a final orientation
        if self.running and self.final_h is not None:
            self.align_final_heading(self.final_h)

        self.stop_robot()

    def stop_robot(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")

# usage example..
# import math
# from drive_to_point import DriveToPoint
# from core.world_model import WorldModel

# world = WorldModel()

# # --- SCENARIO 1: Simple Forward Drive ---
# # Drive 1 meter straight ahead and stop
# mission1 = DriveToPoint(world, target_x=1.0, target_y=0.0)
# mission1.start()
# mission1.join() # Wait for it to finish

# # --- SCENARIO 2: S-Curve / Path Following ---
# # Drive through a series of relative points
# # Because they are intermediate points, the robot will curve through them
# path = [
#     (0.5, 0.5),   # 50cm forward, 50cm left
#     (1.0, 0.0),   # 1m forward total, back to center line
#     (1.5, -0.5)   # 1.5m forward, 50cm right
# ]
# mission2 = DriveToPoint(world, path=path)
# mission2.start()
# mission2.join()

# # --- SCENARIO 3: Precise Arrival with Orientation ---
# # Drive 50cm forward and 50cm left, 
# # then rotate to face exactly backwards (180 degrees / pi)
# target_h = math.pi 
# mission3 = DriveToPoint(world, target_x=0.5, target_y=0.5, final_h=target_h)
# mission3.start()
# mission3.join()
