import threading
import time
import cv2
import numpy as np
import math
from mqtt_python.scam import cam
from mqtt_python.uservice import service

class DriveToHoleTask(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Setup ---
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Calibration & Homography (Copied from drive_to_ball.py) ---
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        self.mtx = np.array([[642.21815902, 0., 406.71091241], [0., 639.62546619, 292.02420168], [0., 0., 1.]])
        self.dist = np.array([[0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]])

        # Pre-calculate undistortion maps
        w, h = 800, 600 
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.newcameramtx, (w, h), cv2.CV_32FC1)

        # --- Hole Detection Parameters (Copied from hole.py) ---
        self.hole_params = {
            "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
            "brown_upper": np.array([30, 255, 255], dtype=np.uint8),
            "brown_min_area_ratio": 0.0005,
            "brown_max_area_ratio": 0.1,
            "brown_min_circularity": 0.2,
            "brown_morph_kernel": (7, 7),
        }

        # --- Control Parameters ---
        self.TARGET_X = 0.0
        self.TARGET_Y = 25.0  # Adjusted closer than ball (34.0) to be over the hole
        self.K_P_TURN = 0.02
        self.K_P_DRIVE = 0.02
        
        self.last_drive = 0.0
        self.last_turn = 0.0
        self.lost_frame_count = 0
        self.MAX_LOST_FRAMES = 10 

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]
        return float(world_x), float(world_y)

    def calculate_control_signals(self, current_x, current_y):
        error_x = self.TARGET_X - current_x 
        error_y = current_y - self.TARGET_Y

        turn = np.clip(error_x * self.K_P_TURN, -0.6, 0.6)
        drive = np.clip(error_y * self.K_P_DRIVE, -0.6, 0.6)

        if abs(error_x) < 1.0: turn = 0.0
        if abs(error_y) < 1.5: drive = 0.0

        return round(float(turn), 2), round(float(drive), 2)

    def detect_hole(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hole_params["brown_lower"], self.hole_params["brown_upper"])
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.hole_params["brown_morph_kernel"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = frame.shape[:2]
        frame_area = width * height
        best_candidate = None
        best_score = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 0: continue
            area_ratio = area / frame_area
            if area_ratio < self.hole_params["brown_min_area_ratio"] or area_ratio > self.hole_params["brown_max_area_ratio"]:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0: continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < self.hole_params["brown_min_circularity"]: continue
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            score = circularity * area_ratio
            if score > best_score:
                best_score = score
                best_candidate = (int(x), int(y))
        return best_candidate

    def run(self):
        while self.running:
            if cam.useCam:
                ok, frame, _ = cam.getImage()
                if ok:
                    # 1. Undistort
                    frame_cal = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    rx, ry, rw, rh = self.roi
                    frame_cal = frame_cal[ry:ry + rh, rx:rx + rw]
                    
                    # 2. Detection (No crop here to maintain coordinate alignment with H matrix)
                    coords = self.detect_hole(frame_cal)

                    target_drive, target_turn = 0.0, 0.0

                    if coords:
                        self.lost_frame_count = 0
                        u, v = coords
                        world_x, world_y = self.get_world_coords(u, v)
                        target_turn, target_drive = self.calculate_control_signals(world_x, world_y)

                        if target_drive == 0.0 and target_turn == 0.0:
                            print("[TASK] Hole reached!")
                            # Optional: service.send("robobot/cmd/T0", "servo 2 0 500") # Open gripper
                    else:
                        self.lost_frame_count += 1
                        if self.lost_frame_count > self.MAX_LOST_FRAMES:
                            target_drive, target_turn = 0.0, 0.0
                        else:
                            target_drive, target_turn = self.last_drive, self.last_turn

                    # 3. Send commands
                    if target_drive != self.last_drive or target_turn != self.last_turn:
                        service.send(self.MQTT_TOPIC_DRIVE, f"rc {target_drive:.1f} {target_turn:.1f}")
                        self.last_drive, self.last_turn = target_drive, target_turn

            time.sleep(0.01)

    def stop(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")