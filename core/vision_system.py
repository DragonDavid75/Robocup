import threading
import time
import cv2 as cv
import numpy as np
from mqtt_python.scam import cam
from mqtt_python.sedge import edge

class VisionSystem(threading.Thread):

    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        
        # --- 1. SETTINGS ---
        self.gripper_x = 420
        self.gripper_y = 340
        self.lights_on = True 

        # --- 2. PERSISTENT STORAGE ---
        # Stored as an attribute so it doesn't reset every loop
        self.history_balls = {"blue": [], "red": [], "white": [], "green": []}

        # --- 3. BLOB DETECTOR SETUP ---
        params = cv.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea, params.maxArea = 200, 10000
        params.filterByCircularity = True
        params.minCircularity = 0.45
        params.minInertiaRatio = 0.3
        self.detector = cv.SimpleBlobDetector_create(params)

        # --- 4. HSV LIMITS ---
        self.update_hsv_limits()

    def update_hsv_limits(self):
        """Sets HSV based on light toggle."""
        if self.lights_on:
            self.blueLower, self.whiteLower = (95, 50, 200), (0, 0, 200)
        else:
            self.blueLower, self.whiteLower = (95, 0, 127), (0, 0, 127)
        
        self.blueUpper = (103, 255, 255)
        self.greenLower, self.greenUpper = (29, 86, 6), (64, 255, 255)
        self.redLower1, self.redUpper1 = (0, 0, 127), (10, 255, 255)
        self.redLower2, self.redUpper2 = (170, 0, 127), (180, 255, 255)
        self.whiteUpper = (180, 35, 255)

    def run(self):
        while self.running:
            if cam.useCam:
                ok, img, imgTime = cam.getImage()
                if ok:
                    # Original line detection
                    edge.paint(img)

                    # --- BALL DETECTION LOGIC ---
                    # 1. Process Frame (Crop bottom 2/3 and Blur)
                    crop_y = int(img.shape[0]/3)
                    work_img = img[crop_y:]
                    blurred = cv.GaussianBlur(work_img, (11, 11), 0)
                    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

                    # 2. Generate Masks
                    masks = {
                        "blue": cv.inRange(hsv, self.blueLower, self.blueUpper),
                        "red": cv.bitwise_or(cv.inRange(hsv, self.redLower1, self.redUpper1), 
                                             cv.inRange(hsv, self.redLower2, self.redUpper2)),
                        "white": cv.inRange(hsv, self.whiteLower, self.whiteUpper),
                        "green": cv.inRange(hsv, self.greenLower, self.greenUpper)
                    }

                    # 3. Detect and Calculate Manhattan Distance
                    for color, mask in masks.items():
                        mask = cv.dilate(cv.erode(mask, None, iterations=2), None, iterations=2)
                        keypoints = self.detector.detect(~mask)
                        
                        if keypoints:
                            self.history_balls[color] = []
                            for kp in keypoints:
                                x, y = int(kp.pt[0]), int(kp.pt[1])
                                # Manhattan: |x1 - x2| + |y1 - y2|
                                dist = abs(x - self.gripper_x) + abs(y - self.gripper_y)
                                self.history_balls[color].append((x, y, dist))

                    # --- UPDATE WORLD ---
                    with self.world.lock:
                        self.world.image = img
                        self.world.line_detected = edge.lineValidCnt > 2
                        self.world.line_left = edge.posLeft
                        self.world.line_right = edge.posRight
                        # Push the new ball data to the world object
                        self.world.vision_balls = self.history_balls

            time.sleep(0.03)  # ~30 Hz

    def stop(self):
        self.running = False