# tasks/vivek/drive_to_point_task.py

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
        self.drive_error = 0

        # Computed at start
        self.turn_angle_deg = 0.0
        self.drive_distance_m = 0.0
        self.return_angle_deg = 0.0

        #_________________________________________

        self.detect_ball_cam = False

        # --- Calibration & Homography ---
        self.pixel_points = np.array([(110, 551), (716, 552), (249, 372), (565, 372), (408, 432), (406, 372), (414, 551), (201, 434), (615, 432)], dtype=np.float32)
        self.world_points = np.array([(-15, 30), (15, 30), (-15, 60), (15, 60), (0, 30), (0, 45), (0, 60), (-15, 45), (15, 45)], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.world_points, cv2.RANSAC, 5.0)

        # Using np.array for matrices
        self.mtx = np.array([[642.21815902, 0., 406.71091241], [0., 639.62546619, 292.02420168], [0., 0., 1.]])
        self.dist = np.array([[0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]])

        # --- PRE-CALCULATE UNDISTORTION MAPS ---
        # Resolution is 806x602 for the uncalibrated camera; adjust if your camera output differs
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

        self.colors = {
            'red': {'lower1': (0, 10, 127), 'upper1': (10, 255, 255), 'lower2': (170, 10, 127),'upper2': (180, 255, 255)},
            'golf': {'lower1': (5, 50, 80), 'upper1': (25, 255, 255)},
            'blue': {'lower1': (95, 50, 200), 'upper1': (103, 255, 255)},
            'white': {'lower1': (0, 0, 200), 'upper1': (180, 35, 255)}
        }

        self.target_color = target_color
        self.GRIPPER_DISTANCE = 0.25 #meter (change this value if robot consistently stops too soon or late, i.e. calibration is off)

        self.stage_2 = 0.25 # travel until 25cm are remaining
        
        self.last_drive = 0.0
        self.last_turn = 0.0
        self.turn_offset = math.radians(6)

        # --- Lost Frame Logic ---
        self.lost_frame_count = 0
        self.MAX_LOST_FRAMES = 10 # Number of frames to wait before stopping

    def get_world_coords(self, u, v):
        # function to convert u, v cordinates to world x, and world y cordinates
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)
        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]
        return float(world_x), float(world_y)

    def detect_ball(self, frame, color_name):
        # function to detect ball on the frame, color name to detect the specific colored ball
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
        # function to capture frame, call detect ball, and calculate the world coordinates to move to
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
        # move servo arm down
        self.servo_controller.servo_control(1, 200, 500)
        time.sleep(0.5)# wait until it is down

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
            #convert from cm to m
            self.target_x = world_y/100
            self.target_y = world_x/100

            # Distance to point
            self.drive_distance_m = (math.hypot(self.target_x, self.target_y)-self.GRIPPER_DISTANCE)
            print(f"[TASK] Initial drive distance (after stage 2 buffer): {self.drive_distance_m:.2f} m")
            self.drive_distance_m = math.hypot(self.target_x, self.target_y)-self.GRIPPER_DISTANCE - self.stage_2

            print(f"[TASK] Initial drive distance (after stage 2 buffer): {self.drive_distance_m:.2f} m")


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

        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            self.state = 1

        #servo arm down and open gripper
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached - open gripper and lower the arm if not already in that state")
                self.servo_controller.servo_control(1, 210, 300)
                self.servo_controller.servo_control(2, 0, 300)
                self.state = 2

        #check if ball was detected in the start() 
        elif self.state == 2:
            if not self.detect_ball_cam:
                self.state = 12
            else:
                self.state = 3

        #turn to face the ball
        elif self.state == 3:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK] Stage 0 - Turning {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 4
            else:
                self.state = 5

        # wait until turn is completed
        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.state = 5

        # drive to the ball, using follow_for_distance to drive on the line, so that the line can guide it
        elif self.state == 5:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Stage 1 - Driving by {self.drive_distance_m:.2f} m")
                self.motion_controller.follow_for_distance(self.drive_distance_m, 0.2, action="LEFT")
            self.state = 6

        #wait until the drive is completed
        elif self.state == 6:
            if not self.motion_controller.is_busy():
                print("[TASK] Stage 1 complete, starting stage 2 correction")
                # detect the ball again once we are 25cm away from it
                result = self.detect_and_compute_target()
                if result is None:
                    print("[TASK] Lost ball after stage 1")
                    self.state = 12
                else:
                    distance, angle = result
                    self.drive_error = distance - (self.GRIPPER_DISTANCE + self.stage_2)
                    self.drive_distance_m = max(0.0, distance - self.GRIPPER_DISTANCE)
                    print(f"[TASK] Stage 2 - Driving by {self.drive_distance_m:.2f} m")
                    # self.drive_distance_m = self.drive_distance_m + (self.drive_error*1.5)
                    print("[TASK] Move error = ", self.drive_error)
                    print(f"[TASK] Stage 2 - Driving by {self.drive_distance_m:.2f} m")
                    self.turn_angle_deg = angle
                    self.state = 7

        #turn to face the ball
        elif self.state == 7:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK] Stage 2 - Correcting turn {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 8
            else:
                self.state = 9

        # wait until turn is completed
        elif self.state == 8:
            if not self.motion_controller.is_busy():
                self.state = 9

        #drive to the ball
        elif self.state == 9:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Stage 2 - Final drive by {self.drive_distance_m:.2f} m")
                # self.motion_controller.drive_distance(self.drive_distance_m, 0.2)
                self.motion_controller.follow_for_distance(self.drive_distance_m, 0.1, action="LEFT")
                self.state = 10
            else:
                self.state = 11

        # wait until drive is complete, and then close the gripper
        elif self.state == 10:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, -1000, 300)
                time.sleep(1)
                self.state = 11

        elif self.state == 11:
            print("[TASK] DriveToPoint completed")
            # lift the servo arm slowly, 
            self.servo_controller.servo_control(1, -500, 20)
            self.state = 12

        elif self.state == 12:
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] DriveToPoint stopped")
