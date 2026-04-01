import threading
import time
import cv2
import numpy as np
from mqtt_python.scam import cam
from mqtt_python.uservice import service
from mqtt_python.sedge import edge

class DriveToBallTask(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        
        # --- MQTT Setup ---
        self.MQTT_TOPIC_GRIPPER = "robobot/cmd/T0"
        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

        # --- Calibration & Homography ---
        # Pixel points and World points from your calibration
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        # Camera Intrinsic Matrices
        self.mtx = np.matrix([[642.21815902, 0., 406.71091241], [0., 639.62546619, 292.02420168], [0., 0., 1.]])
        self.dist = np.matrix([[0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]])

        # --- Blob Detector Settings ---
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 500
        params.filterByCircularity = True
        params.minCircularity = 0.45
        params.filterByInertia = True
        params.minInertiaRatio = 0.3
        self.detector = cv2.SimpleBlobDetector_create(params)

        # --- Color Definitions (H, S, V) ---
        self.colors = {
            'red': {'lower1': (0, 10, 127), 'upper1': (10, 255, 255), 'lower2': (170, 10, 127), 'upper2': (180, 255, 255)},
            'blue': {'lower1': (95, 0, 127), 'upper1': (103, 255, 255)},
            'white': {'lower1': (0, 0, 200), 'upper1': (180, 35, 255)}
        }

        # Control Variables
        self.target_color = 'red' # Default target
        self.TARGET_X = 0.0
        self.TARGET_Y = 34.0
        self.K_P_TURN = 0.02
        self.K_P_DRIVE = 0.02

    def get_world_coords(self, u, v):
        """Converts pixel coordinates (u, v) to world coordinates (x, y)."""
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]
        return float(world_x), float(world_y)

    def calculate_control_signals(self, current_x, current_y):
        """control logic for movement."""
        error_x = self.TARGET_X - current_x 
        error_y = current_y - self.TARGET_Y

        turn = np.clip(error_x * self.K_P_TURN, -0.6, 0.6)
        drive = np.clip(error_y * self.K_P_DRIVE, -0.6, 0.6)

        if abs(error_x) < 1.0: turn = 0.0
        if abs(error_y) < 1.5: drive = 0.0

        return round(float(turn), 2), round(float(drive), 2)

    def detect_ball(self, frame, color_name):
        """
        Detects ball of a specific color.
        Returns: (u, v) coordinates if found, else None.
        """
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (11, 11), 0), cv2.COLOR_BGR2HSV)
        c = self.colors.get(color_name)
        
        # Handle Red wrap-around vs single range colors
        mask = cv2.inRange(hsv, c['lower1'], c['upper1'])
        if 'lower2' in c:
            mask2 = cv2.inRange(hsv, c['lower2'], c['upper2'])
            mask = cv2.bitwise_or(mask, mask2)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Detect blobs (inverted mask)
        keypoints = self.detector.detect(~mask)
        
        if keypoints:
            # Return the largest blob or the first one found
            best_kp = max(keypoints, key=lambda x: x.size)
            return best_kp.pt # Returns (u, v)
        return None

    def run(self):
        service.send(self.MQTT_TOPIC_GRIPPER, f"servo 1 200 500 {time.time()}")
        turn, drive = 0.0, 0.0
        while self.running:
            if cam.useCam:
                ok, frame, _ = cam.getImage()
                print(f"DriveToBallTask: Frame received - OK={ok}, Shape={frame.shape if ok else 'N/A'}")
                if ok:
                    # 1. Undistort and Pre-process
                    h, w = frame.shape[:2]
                    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
                    frame_cal = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
                    x, y, w, h = roi
                    frame_cal = frame_cal[y:y + h, x:x + w]
                    
                    # Crop to focus on floor (lower 2/3)
                    crop_y = int(frame_cal.shape[0]/3)
                    roi_frame = frame_cal[crop_y:, :]

                    # 2. Search for the target ball
                    coords = self.detect_ball(roi_frame, self.target_color)

                    # turn, drive = 0.0, 0.0
                    
                    if coords:
                        u, v = coords
                        # Adjust v because we cropped the top 1/3
                        v_adjusted = v + crop_y 
                        
                        # Convert to world and calculate signals
                        world_x, world_y = self.get_world_coords(u, v_adjusted)
                        turn, drive = self.calculate_control_signals(world_x, world_y)

                        # Gripper Logic based on Drive (Distance)
                        if drive == 0.0:
                            # Close gripper
                            service.send(self.MQTT_TOPIC_GRIPPER, f"servo 2 -1000 500 {time.time()}")
                            # service.publish(self.MQTT_TOPIC_GRIPPER, "servo 2 -1000 500")
                            # 2. Send explicit stop command to motors
                            service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}")
                            
                            # 3. Print status and exit the thread loop
                            print(f"Target {self.target_color} reached. Task complete. Exiting.")
                            self.running = False 
                            break # Break the while loop immediately
                        else:
                            # Open/Ready gripper
                            service.send(self.MQTT_TOPIC_GRIPPER, f"servo 2 0 500 {time.time()}")

                    # 3. Apply drive command
                    drive_cmd = f"rc {drive:.1f} {turn:.1f}"
                    service.send(self.MQTT_TOPIC_DRIVE, drive_cmd)
                    print(f"Drive Command: {drive_cmd}")

                    # 4. Update shared world state
                    with self.world.lock:
                        self.world.image = frame_cal
                        self.world.ball_coords = coords

            time.sleep(0.03)

    def stop(self):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, f"rc 0.0 0.0 {time.time()}") # Stop robot on exit
