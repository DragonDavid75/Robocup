# tasks/nil/drive_to_hole_point_task.py

import math
import cv2
import numpy as np
from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.scam import cam

class DriveToHolePointTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        
        # Calibration & Homography (from drive_to_hole.py)
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        # Hole Detection Parameters
        self.hole_params = {
            "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
            "brown_upper": np.array([30, 255, 255], dtype=np.uint8),
            "brown_min_area_ratio": 0.0005,
            "brown_max_area_ratio": 0.1,
            "brown_min_circularity": 0.2,
            "brown_morph_kernel": (7, 7),
        }

        self.state = 0 # 0: Searching, 1: Turning, 2: Driving, 3: Done
        self.target_x = 0.0
        self.target_y = 0.0
        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        # Note: scale cm to meters if motion_controller expects meters
        world_x = (transformed[0] / transformed[2]) / 100.0 
        world_y = (transformed[1] / transformed[2]) / 100.0
        return float(world_x), float(world_y)

    def detect_hole(self):
        ok, frame, _ = cam.getImage()
        if not ok: return None
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hole_params["brown_lower"], self.hole_params["brown_upper"])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_candidate = None
        best_score = -1
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100: continue
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if area > best_score:
                best_score = area
                best_candidate = (int(x), int(y))
        return best_candidate

    def update(self):
        # State 0: Look for the hole and calculate coordinates
        if self.state == 0:
            coords = self.detect_hole()
            if coords:
                u, v = coords
                self.target_x, self.target_y = self.get_world_coords(u, v)
                
                # Logic from drive_to_point_task.py
                self.drive_distance_m = math.hypot(self.target_x, self.target_y)
                self.turn_angle_deg = -math.degrees(math.atan2(self.target_y, self.target_x))
                
                print(f"[TASK] Hole detected at x={self.target_x:.2f}m, y={self.target_y:.2f}m")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 1
            return TaskStatus.RUNNING

        # State 1: Wait for turn, then start driving
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print(f"[TASK] Driving {self.drive_distance_m:.2f}m to hole")
                self.motion_controller.drive_distance(self.drive_distance_m, 0.15)
                self.state = 2
            return TaskStatus.RUNNING

        # State 2: Wait for drive to finish
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] Reached hole position")
                self.state = 3
                return TaskStatus.DONE
            return TaskStatus.RUNNING

        return TaskStatus.DONE