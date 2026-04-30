import math
from tasks.base_task import BaseTask, TaskStatus
import cv2
import numpy as np
from mqtt_python.scam import cam
import time


class DriveToArUcoTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller, target_ids=(12, 14)):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.target_ids = target_ids

        self.target_x = 0.0
        self.target_y = 0.0

        self.state = 0
        self.detect_aruco_cam = False

        self.turn_angle_deg = 0.0
        self.drive_distance_m = 0.0

        self.drive_speed = 0.15
        self.turn_offset = math.radians(3)
        self.GRIPPER_DISTANCE = self.world.gripper_distance

        # --- Calibration & Homography ---
        self.pixel_points = np.array(
            [
                (110, 551), (716, 552),
                (249, 372), (565, 372),
                (408, 432), (406, 372),
                (414, 551), (201, 434),
                (615, 432)
            ],
            dtype=np.float32
        )

        self.world_points = np.array(
            [
                (-15, 30), (15, 30),
                (-15, 60), (15, 60),
                (0, 30), (0, 45),
                (0, 60), (-15, 45),
                (15, 45)
            ],
            dtype=np.float32
        )

        self.H, _ = cv2.findHomography(
            self.pixel_points,
            self.world_points,
            cv2.RANSAC,
            5.0
        )

        self.mtx = np.array([
            [642.21815902, 0., 406.71091241],
            [0., 639.62546619, 292.02420168],
            [0., 0., 1.]
        ])

        self.dist = np.array([
            [0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]
        ])

        w, h = 806, 602

        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx,
            self.dist,
            (w, h),
            1,
            (w, h)
        )

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.mtx,
            self.dist,
            None,
            self.newcameramtx,
            (w, h),
            cv2.CV_32FC1
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)

        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]

        return float(world_x), float(world_y)

    def detect_aruco_centers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        params = cv2.aruco.DetectorParameters()

        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 7

        params.minMarkerPerimeterRate = 0.02
        params.maxMarkerPerimeterRate = 4.0

        params.polygonalApproxAccuracyRate = 0.05
        params.minCornerDistanceRate = 0.02
        params.minDistanceToBorder = 2

        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1

        detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            params
        )

        corners, ids, rejected = detector.detectMarkers(gray)

        centers = {}

        if ids is None:
            print(f"[ARUCO] No markers detected. Rejected candidates: {len(rejected)}")
            return centers

        print(f"[ARUCO] Detected IDs: {ids.flatten().tolist()}")

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners[0]

            center_u = float(np.mean(pts[:, 0]))
            center_v = float(np.mean(pts[:, 1]))

            left_height = np.linalg.norm(pts[0] - pts[3])
            right_height = np.linalg.norm(pts[1] - pts[2])
            marker_height = float((left_height + right_height) / 2.0)

            bottom_u = float((pts[2][0] + pts[3][0]) / 2.0)
            bottom_v = float((pts[2][1] + pts[3][1]) / 2.0)

            # ArUco marker is upright.
            # Use a point below the bottom border.
            # Offset is 80% of the marker height.
            floor_u = bottom_u
            floor_v = bottom_v + (0.8 * marker_height)

            centers[int(marker_id)] = (floor_u, floor_v)

            print(
                f"[ARUCO] ID {int(marker_id)} "
                f"center=({center_u:.1f}, {center_v:.1f}), "
                f"bottom=({bottom_u:.1f}, {bottom_v:.1f}), "
                f"floor=({floor_u:.1f}, {floor_v:.1f}), "
                f"height={marker_height:.1f}px"
            )

        return centers

    def start(self):
        super().start()

        self.state = 0
        self.detect_aruco_cam = False

        print("[TASK] DriveToArUcoTask started")
        print("[TASK] Capturing full camera frame for ArUco detection...")

        ok, frame, _ = cam.getImage()
        print(f"[TASK] Camera frame capture {'succeeded' if ok else 'failed'}")

        if ok:
            frame_cal = cv2.remap(
                frame,
                self.mapx,
                self.mapy,
                cv2.INTER_LINEAR
            )

            # Do NOT crop for ArUco detection.
            centers = self.detect_aruco_centers(frame_cal)

            visible_ids = [
                marker_id for marker_id in self.target_ids
                if marker_id in centers
            ]

            target_u = None
            target_v = None

            if len(visible_ids) >= 2:
                id1, id2 = visible_ids[0], visible_ids[1]

                u1, v1 = centers[id1]
                u2, v2 = centers[id2]

                target_u = (u1 + u2) / 2.0
                target_v = (v1 + v2) / 2.0

                print(f"[TASK] Using midpoint between ArUco {id1} and {id2}")

            elif len(visible_ids) == 1:
                id1 = visible_ids[0]

                target_u, target_v = centers[id1]

                print(f"[TASK] Only ArUco {id1} detected, driving to its floor point")

            else:
                print(
                    f"[TASK] Could not see any target ArUco IDs {self.target_ids}. "
                    f"Detected: {list(centers.keys())}"
                )

            if target_u is not None:
                world_x, world_y = self.get_world_coords(target_u, target_v)

                print(
                    f"[TASK] Target pixel ({target_u:.1f}, {target_v:.1f}) "
                    f"-> world ({world_x:.2f}, {world_y:.2f})"
                )

                self.detect_aruco_cam = True

                # Same coordinate convention as your ball task
                self.target_x = world_y / 100
                self.target_y = world_x / 100

                self.drive_distance_m = (
                    math.hypot(self.target_x, self.target_y)
                    - self.GRIPPER_DISTANCE
                )
                self.drive_distance_m = max(0.0, self.drive_distance_m)

                self.turn_angle_deg = -math.degrees(
                    math.atan2(self.target_y, self.target_x)
                )

                if self.turn_angle_deg > 4:
                    self.turn_angle_deg -= math.degrees(self.turn_offset)
                elif self.turn_angle_deg < -4:
                    self.turn_angle_deg += math.degrees(self.turn_offset)

        print(
            f"[TASK] DriveToArUco target_x={self.target_x:.2f}, "
            f"target_y={self.target_y:.2f}, "
            f"turn={self.turn_angle_deg:.2f} deg, "
            f"distance={self.drive_distance_m:.2f} m"
        )

    def update(self):
        # Check if ArUco was detected
        if self.state == 0:
            if not self.detect_aruco_cam:
                self.state = 5
            else:
                self.state = 1

        # Turn once toward target
        elif self.state == 1:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(
                    f"[TASK] Turning {direction} by "
                    f"{abs(self.turn_angle_deg):.2f} deg"
                )

                self.motion_controller.turn_in_place(
                    math.radians(self.turn_angle_deg)
                )

                self.state = 2
            else:
                self.state = 3

        # Wait for turn
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.state = 3

        # Drive once to target
        elif self.state == 3:
            if self.drive_distance_m > 1e-6:
                print(
                    f"[TASK] Driving to ArUco target in one shot by "
                    f"{self.drive_distance_m:.2f} m"
                )

                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed
                )

                self.state = 4
            else:
                self.state = 5

        # Wait for drive to finish
        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, 0, 600)
                time.sleep(0.3)
                self.state = 5

        # Done
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    -self.drive_speed
                )
                self.state = 6
                print("[TASK] DriveToArUcoTask completed")
        elif self.state == 6:
            if not self.motion_controller.is_busy():
                self.stop()
                print("[TASK] DriveToArUcoTask completed")
                return TaskStatus.DONE  
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] DriveToArUcoTask stopped")