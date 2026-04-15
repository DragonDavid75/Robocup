import threading
import time
import cv2
import numpy as np
from mqtt_python.scam import cam
from mqtt_python.uservice import service

class DriveToBallTask(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Setup ---
        self.MQTT_TOPIC_GRIPPER = "robobot/cmd/T0"
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Calibration & Homography ---
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        # Using np.array for matrices
        self.mtx = np.array([[642.21815902, 0., 406.71091241], [0., 639.62546619, 292.02420168], [0., 0., 1.]])
        self.dist = np.array([[0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]])

        # --- PRE-CALCULATE UNDISTORTION MAPS ---
        # Assuming standard resolution 800x600; adjust if your camera output differs
        w, h = 800, 600 
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.newcameramtx, (w, h), cv2.CV_32FC1)

        # --- Blob Detector Settings ---
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 500
        params.filterByCircularity = True
        params.minCircularity = 0.45
        params.filterByInertia = True
        params.minInertiaRatio = 0.3
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.colors = {
            'red': {'lower1': (0, 10, 80), 'upper1': (10, 255, 255), 'lower2': (170, 10, 80), 'upper2': (180, 255, 255)},
            'blue': {'lower1': (95, 0, 127), 'upper1': (103, 255, 255)},
            'white': {'lower1': (0, 0, 200), 'upper1': (180, 35, 255)}
        }

        self.target_color = 'red'
        self.TARGET_X = 0.0
        self.TARGET_Y = 34.0
        self.K_P_TURN = 0.02
        self.K_P_DRIVE = 0.02
        
        self.last_drive = 0.0
        self.last_turn = 0.0

        # --- Lost Frame Logic ---
        self.lost_frame_count = 0
        self.MAX_LOST_FRAMES = 10 # Number of frames to wait before stopping

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

    def detect_ball(self, frame, color_name):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        c = self.colors.get(color_name)
        
        mask = cv2.inRange(hsv, c['lower1'], c['upper1'])
        if 'lower2' in c:
            mask2 = cv2.inRange(hsv, c['lower2'], c['upper2'])
            mask = cv2.bitwise_or(mask, mask2)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        keypoints = self.detector.detect(~mask)
        if keypoints:
            best_kp = max(keypoints, key=lambda x: x.size)
            return best_kp.pt 
        return None

    def run(self):
        # Initial Gripper State: Open
        service.send(self.MQTT_TOPIC_GRIPPER, f"servo 1 200 100 {time.time()}")
        service.send(self.MQTT_TOPIC_GRIPPER, f"servo 2 0 500 {time.time()}")
        
        while self.running:
            if cam.useCam:
                ok, frame, _ = cam.getImage()
                if ok:
                    # 1. Optimized Undistort using Remap
                    frame_cal = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    rx, ry, rw, rh = self.roi
                    frame_cal = frame_cal[ry:ry + rh, rx:rx + rw]
                    
                    # 2. Crop to Floor
                    crop_y_start = int(frame_cal.shape[0]/3)
                    roi_frame = frame_cal[crop_y_start:, :]

                    # 3. Detection
                    coords = self.detect_ball(roi_frame, self.target_color)

                    target_drive, target_turn = 0.0, 0.0

                    if coords:
                        self.lost_frame_count = 0 # Reset counter
                        u, v = coords
                        v_world_space = v + crop_y_start 
                        
                        world_x, world_y = self.get_world_coords(u, v_world_space)
                        target_turn, target_drive = self.calculate_control_signals(world_x, world_y)

                        if target_drive == 0.0:
                            # Close gripper and notify, but don't break the loop if you want continuous operation
                            service.send(self.MQTT_TOPIC_GRIPPER, f"servo 2 -1000 500 {time.time()}")
                            service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")
                            print("Target reached. Closing gripper.")
                    else:
                        self.lost_frame_count += 1
                        
                        # Only stop/decelerate if we've lost the ball for several frames
                        if self.lost_frame_count > self.MAX_LOST_FRAMES:
                            target_turn = round(max(0, self.last_turn - 0.1) if self.last_turn > 0 else min(0, self.last_turn + 0.1), 2)
                            target_drive = round(max(0, self.last_drive - 0.1) if self.last_drive > 0 else min(0, self.last_drive + 0.1), 2)
                        else:
                            # Keep doing what we were doing (maintain last command)
                            target_drive, target_turn = self.last_drive, self.last_turn

                    # 4. MQTT Gating: Only send if values changed
                    if target_drive != self.last_drive or target_turn != self.last_turn:
                        drive_cmd = f"rc {target_drive:.1f} {target_turn:.1f}"
                        service.send(self.MQTT_TOPIC_DRIVE, drive_cmd)
                        self.last_drive, self.last_turn = target_drive, target_turn

                    # 5. Update shared world state
                    with self.world.lock:
                        self.world.image = frame_cal
                        self.world.ball_coords = coords

            # Sleep slightly to allow other threads to breathe
            time.sleep(0.01)

    def stop(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")
