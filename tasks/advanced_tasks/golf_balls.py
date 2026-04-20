# Third of the tasks
#Worked a few times together with timer first

import math
from tasks.base_task import BaseTask, TaskStatus
import time
import cv2
import numpy as np
from mqtt_python.scam import cam
from mqtt_python.uservice import service


from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time

class DriveToGolf1Task(BaseTask):
    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
        target_color='golf'
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller

        self.target_x = 0.0
        self.target_y = 0.0

        self.state = 0

        # Speeds
        self.turn_speed = 0.3
        self.drive_speed = 0.15

        # Computed at start
        self.turn_angle_deg = 0.0
        self.drive_distance_m = 0.0
        self.return_angle_deg = 0.0

        #_________________________________________

        self.detect_ball_cam = False
        
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
        w, h = 806, 602
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.newcameramtx, (w, h), cv2.CV_32FC1)

        # --- Blob Detector Settings ---
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10
        params.maxThreshold = 200
        params.filterByArea = True
        params.minArea = 500  # was 1000, 500 also ok, 200...
        params.maxArea = 10000
        params.filterByCircularity = True
        params.minCircularity = 0.45
        params.filterByConvexity = False
        params.minConvexity = 0.87
        params.filterByInertia = True
        params.minInertiaRatio = 0.3
        self.detector = cv2.SimpleBlobDetector_create(params)
        # --- Blob Detector Settings ---
        # params = cv2.SimpleBlobDetector_Params()
        # params.filterByArea = True
        # params.minArea = 500
        # params.filterByCircularity = True
        # params.minCircularity = 0.45
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.3
        # self.detector = cv2.SimpleBlobDetector_create(params)

        # self.colors = {
        #     'red': {'lower1': (0, 10, 80), 'upper1': (10, 255, 255), 'lower2': (170, 10, 80), 'upper2': (180, 255, 255)},
        #     'blue': {'lower1': (95, 0, 127), 'upper1': (103, 255, 255)},
        #     'white': {'lower1': (0, 0, 200), 'upper1': (180, 35, 255)}
        # }
        self.colors = {
            'red': {'lower1': (0, 10, 127), 'upper1': (10, 255, 255), 'lower2': (170, 10, 127),'upper2': (180, 255, 255)},
            'golf': {'lower1': (5, 50, 80), 'upper1': (25, 255, 255)},
            'blue': {'lower1': (95, 50, 200), 'upper1': (103, 255, 255)},
            'white': {'lower1': (0, 0, 200), 'upper1': (180, 35, 255)}
        }

        self.target_color = target_color
        self.GRIPPER_DISTANCE = 0.24 #meter
        self.K_P_TURN = 0.02
        self.K_P_DRIVE = 0.02
        
        self.last_drive = 0.0
        self.last_turn = 0.0
        self.turn_offset = math.radians(6)

        # --- Lost Frame Logic ---
        self.lost_frame_count = 0
        self.MAX_LOST_FRAMES = 10 # Number of frames to wait before stopping

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]
        return float(world_x), float(world_y)
    
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

    def detect_and_compute_target(self):
        ok, frame, _ = cam.getImage()
        if not ok:
            return None

        frame_cal = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        rx, ry, rw, rh = self.roi
        frame_cal = frame_cal[ry:ry + rh, rx:rx + rw]

        crop_y_start = int(frame_cal.shape[0] / 3)
        roi_frame = frame_cal[crop_y_start:, :]

        coords = self.detect_ball(roi_frame, self.target_color)
        if not coords:
            return None

        u, v = coords
        v_world_space = v + crop_y_start
        world_x, world_y = self.get_world_coords(u, v_world_space)

        # convert
        target_x = world_y / 100
        target_y = world_x / 100

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        return distance, angle

    def start(self):
        super().start()
        print("[TASK] GolfBallsTask started")
        self.state = 0
        

        world_x, world_y = None,None
        print("[TASK] Capturing camera frame for ball detection...")

        ok, frame, _ = cam.getImage()
        print(f"[TASK] Starting DriveToBallTask, camera frame capture {'succeeded' if ok else 'failed'}")
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

            if coords:
                self.detect_ball_cam = True
                self.lost_frame_count = 0 # Reset counter
                u, v = coords
                v_world_space = v + crop_y_start 
                        
                world_x, world_y = self.get_world_coords(u, v_world_space)
                print(f"[DETECTION] Detected {self.target_color} ball at pixel ({u:.1f}, {v_world_space:.1f}) -> world ({world_x:.2f}, {world_y:.2f})")
        
        if self.detect_ball_cam:
            self.target_x = world_y/100
            self.target_y = world_x/100

            # Distance to point
            self.drive_distance_m = (math.hypot(self.target_x, self.target_y)-self.GRIPPER_DISTANCE)*0.9

            self.turn_angle_deg = -(math.degrees(
                math.atan2(self.target_y, self.target_x)
            ))
            print(f"[TASK] Initial turn angle: {self.turn_angle_deg:.2f} deg, drive distance: {self.drive_distance_m:.2f} m")
            if self.turn_angle_deg > 4:#left
                self.turn_angle_deg -= (math.degrees(self.turn_offset)-4)
            elif self.turn_angle_deg < -4:#right
                self.turn_angle_deg += math.degrees(self.turn_offset)

            self.return_angle_deg = -self.turn_angle_deg

        print(
            f"[TASK] DriveToBall started: "
            f"target_x={self.target_x:.2f}, "
            f"target_y={self.target_y:.2f}, "
            f"turn={self.turn_angle_deg:.2f} deg, "
            f"distance={self.drive_distance_m:.2f} m"
        )

    def update(self):
        # 0 = open gripper
        # 1 = detect ball
        # 2 = turn toward ball
        # 3 = drive toward ball
        # 4 = re-detect / correct
        # 5 = close gripper
        # 6 = done

        if self.state == 0:
            print("[TASK] Opening gripper")
            self.servo_controller.servo_control(1, 200, 300)     # arm position, adjust if needed
            self.servo_controller.servo_control(2, 0, 300)       # gripper open, adjust if needed
            self.state = 1

        elif self.state == 1:
            print("[TASK] Detecting ball")
            result = self.detect_and_compute_target()

            if result is None:
                print("[TASK] No ball detected")
                self.state = 6
            else:
                distance, angle = result

                # Keep this variable consistent:
                # drive_distance_m = distance from robot to ball center
                self.drive_distance_m = distance
                self.turn_angle_deg = angle

                print(f"[TASK] Ball detected: distance={distance:.2f} m, angle={angle:.2f} deg")
                self.state = 2

        elif self.state == 2:
            if abs(self.turn_angle_deg) > 3.0:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK] Turning {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 21
            else:
                self.state = 3

        elif self.state == 21:
            if not self.motion_controller.is_busy():
                self.state = 3

        elif self.state == 3:
            # Distance robot should still drive before grabbing
            remaining_to_drive = max(0.0, self.drive_distance_m - self.GRIPPER_DISTANCE)

            print(f"[TASK] Remaining to drive: {remaining_to_drive:.2f} m")

            # Close enough to grab
            if remaining_to_drive <= 0.03:
                print("[TASK] Close enough to ball")
                self.state = 5
            else:
                # Small step forward for robustness
                step_distance = min(0.05, remaining_to_drive)

                print(f"[TASK] Driving forward by {step_distance:.2f} m")
                self.motion_controller.follow_for_distance(
                    step_distance,
                    0.05,
                    action="LEFT"
                )
                self.state = 4

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                print("[TASK] Re-detecting ball")
                result = self.detect_and_compute_target()

                if result is None:
                    print("[TASK] Lost ball, attempting grab if close")
                    self.state = 5
                else:
                    distance, angle = result
                    self.drive_distance_m = distance
                    self.turn_angle_deg = angle

                    print(f"[TASK] Updated target: distance={distance:.2f} m, angle={angle:.2f} deg")

                    # If heading is off, turn again before next step
                    if abs(angle) > 3.0:
                        self.state = 2
                    else:
                        self.state = 3

        elif self.state == 5:
            print("[TASK] Closing gripper")
            self.servo_controller.servo_control(2, -1000, 800)   # closed, adjust if needed
            self.state = 6

        elif self.state == 6:
            print("[TASK] Ball pickup task completed")
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] DriveToPoint stopped")
