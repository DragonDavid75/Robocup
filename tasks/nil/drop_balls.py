import math
import cv2
import numpy as np
from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.scam import cam

class DriveToHoleTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        
        # --- Homography Setup (from golf_balls.py) ---
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        # --- Hole Detector Parameters (from hole.py) ---
        self.params = {
            "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
            "brown_upper": np.array([30, 255, 255], dtype=np.uint8),
            "brown_min_area_ratio": 0.0005,
            "brown_max_area_ratio": 0.1,
            "brown_min_circularity": 0.2,
            "brown_morph_kernel": (7, 7),
        }

        self.state = 0
        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]
        return float(world_x), float(world_y)

    def detect_hole(self, frame):
        """Logic derived from nil/hole.py"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        brown_mask = cv2.inRange(hsv, self.params["brown_lower"], self.params["brown_upper"])

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.params["brown_morph_kernel"])
        brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_candidate = None
        best_score = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Basic filtering logic
            if area < 500: continue 
            
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            best_candidate = (int(x), int(y))
            break # Return the first valid candidate

        return best_candidate

    def detect_and_compute_target(self):
        ok, frame, _ = cam.getImage()
        if not ok: return None

        # Apply crop (similar to hole.py logic)
        height, width = frame.shape[:2]
        crop_h = int(height * 1 / 2)
        crop = frame[height//2:, :] # Look at bottom half

        center = self.detect_hole(crop)
        if not center: return None

        u, v = center
        # Adjust v to be relative to original frame
        v_world_space = v + (height//2)
        world_x, world_y = self.get_world_coords(u, v_world_space)

        target_x = world_y / 100
        target_y = world_x / 100

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        return distance, angle

    def update(self):
        # State Machine: 0=Detect, 1=Turn, 2=Drive
        if self.state == 0:
            result = self.detect_and_compute_target()
            if result:
                self.drive_distance_m, self.turn_angle_deg = result
                self.state = 1
            return TaskStatus.RUNNING

        elif self.state == 1:
            if abs(self.turn_angle_deg) > 3.0:
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
            self.state = 2
            return TaskStatus.RUNNING

        elif self.state == 2:
            if self.drive_distance_m > 0.1: # Threshold to stop
                self.motion_controller.follow_for_distance(self.drive_distance_m - 0.1, 0.1)
                self.state=3
                return TaskStatus.RUNNING
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, 200, 300)
                self.servo_controller.servo_control(2, 0, 300)
            return TaskStatus.DONE

        return TaskStatus.RUNNING

