import math
import time

import cv2
import numpy as np

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.scam import cam


class PickAndDropBallTask(BaseTask):
    """
    Combined task:
    1. Detect red and/or blue balls.
    2. Pick the closest visible ball first.
    3. Return to the original pose after pickup.
    4. Drive to the correct ArUco drop zone:
       - blue -> midpoint/floor point of ArUco IDs 14 and 15
       - red  -> midpoint/floor point of ArUco IDs 12 and 13
    5. Open gripper to drop the ball.
    6. Reverse out from the drop zone.

    Detection/calculation logic is kept from the original ball and ArUco tasks.
    """

    DROP_ARUCO_IDS = {
        "blue": (14, 15),
        "red": (12, 13),
    }

    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
        target_colors=("red", "blue"),
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.target_color = list(target_colors)

        self.state = 0

        self.target_x = 0.0
        self.target_y = 0.0

        self.detect_ball_cam = False
        self.detect_aruco_cam = False

        self.closest_ball = None
        self.drop_target_ids = None

        self.turn_angle_deg = 0.0
        self.drive_distance_m = 0.0
        self.return_angle_deg = 0.0

        self.total_distance = 0.0
        self.total_turn = 0.0

        self.drive_error = 0.0
        self.drive_speed_ball_stage_1 = 0.2
        self.drive_speed_ball_stage_2 = 0.15
        self.drive_speed_aruco = 0.15

        self.GRIPPER_DISTANCE = self.world.gripper_distance
        self.stage_2 = self.world.stage_2
        self.turn_offset = math.radians(3)

        self.max_ball_distance_m = 2.5

        # --- Calibration & Homography ---
        self.pixel_points = np.array(
            [
                (110, 551), (716, 552),
                (249, 372), (565, 372),
                (408, 432), (406, 372),
                (414, 551), (201, 434),
                (615, 432),
            ],
            dtype=np.float32,
        )

        self.world_points = np.array(
            [
                (-15, 30), (15, 30),
                (-15, 60), (15, 60),
                (0, 30), (0, 45),
                (0, 60), (-15, 45),
                (15, 45),
            ],
            dtype=np.float32,
        )

        self.H, _ = cv2.findHomography(
            self.pixel_points,
            self.world_points,
            cv2.RANSAC,
            5.0,
        )

        self.mtx = np.array(
            [
                [642.21815902, 0.0, 406.71091241],
                [0.0, 639.62546619, 292.02420168],
                [0.0, 0.0, 1.0],
            ]
        )

        self.dist = np.array(
            [[0.02674745, -0.10674703, -0.00277349, 0.0034295, 0.16937755]]
        )

        w, h = 806, 602
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
            self.mtx,
            self.dist,
            (w, h),
            1,
            (w, h),
        )

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.mtx,
            self.dist,
            None,
            self.newcameramtx,
            (w, h),
            cv2.CV_32FC1,
        )

        # --- Ball blob detector settings ---
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10
        params.maxThreshold = 200
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 10000
        params.filterByCircularity = True
        params.minCircularity = 0.45
        params.filterByConvexity = False
        params.minConvexity = 0.87
        params.filterByInertia = True
        params.minInertiaRatio = 0.3
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.colors = {
            "red": {
                "lower1": (0, 50, 60),
                "upper1": (10, 255, 255),
                "lower2": (170, 50, 60),
                "upper2": (180, 255, 255),
            },
            "golf": {
                "lower1": (5, 50, 80),
                "upper1": (25, 255, 255),
            },
            "blue": {
                "lower1": (105, 50, 60),
                "upper1": (135, 255, 255),
            },
            "white": {
                "lower1": (0, 0, 200),
                "upper1": (180, 35, 255),
            },
        }

        # --- ArUco detector setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # ---------------------------------------------------------------------
    # Shared camera/geometry helpers
    # ---------------------------------------------------------------------

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)

        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]

        return float(world_x), float(world_y)

    def get_calibrated_frame(self):
        ok, frame, _ = cam.getImage()
        if not ok:
            return None

        frame_cal = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        rx, ry, rw, rh = self.roi
        frame_cal = frame_cal[ry:ry + rh, rx:rx + rw]
        return frame_cal

    def apply_turn_offset(self, angle_deg):
        if angle_deg > 4:
            return angle_deg - math.degrees(self.turn_offset)
        if angle_deg < -4:
            return angle_deg + math.degrees(self.turn_offset)
        return angle_deg

    # ---------------------------------------------------------------------
    # Ball detection and pickup helpers
    # ---------------------------------------------------------------------

    def detect_ball(self, frame, color_name):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        c = self.colors.get(color_name)

        if c is None:
            print(f"[TASK][BALL] Unknown color: {color_name}")
            return None

        mask = cv2.inRange(hsv, c["lower1"], c["upper1"])
        if "lower2" in c:
            mask2 = cv2.inRange(hsv, c["lower2"], c["upper2"])
            mask = cv2.bitwise_or(mask, mask2)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        keypoints = self.detector.detect(~mask)
        if keypoints:
            best_kp = max(keypoints, key=lambda x: x.size)
            return best_kp.pt
        return None

    def find_closest_ball(self):
        frame_cal = self.get_calibrated_frame()
        if frame_cal is None:
            print("[BALL] Camera frame capture failed")
            return None

        crop_y_start = int(frame_cal.shape[0] / 3)
        roi_frame = frame_cal[crop_y_start:, :]

        closest = None

        for color_name in self.target_color:
            coords = self.detect_ball(roi_frame, color_name)
            if not coords:
                continue

            u, v = coords
            v_world_space = v + crop_y_start
            world_x, world_y = self.get_world_coords(u, v_world_space)

            distance = math.hypot(world_x / 100, world_y / 100)

            # Ignore balls farther than configured max distance
            if distance > self.max_ball_distance_m:
                print(
                    f"[BALL] Ignoring {color_name} ball at {distance:.2f} m "
                    f"(max allowed: {self.max_ball_distance_m:.2f} m)"
                )
                continue

            candidate = {
                "color": color_name,
                "u": u,
                "v": v_world_space,
                "world_x": world_x,
                "world_y": world_y,
                "distance": distance,
            }

            if closest is None or candidate["distance"] < closest["distance"]:
                closest = candidate

        return closest

    def detect_and_compute_ball_target(self):
        frame_cal = self.get_calibrated_frame()
        if frame_cal is None:
            return None

        crop_y_start = int(frame_cal.shape[0] / 3)
        roi_frame = frame_cal[crop_y_start:, :]

        coords = self.detect_ball(roi_frame, self.closest_ball)
        if not coords:
            return None

        u, v = coords
        v_world_space = v + crop_y_start
        world_x, world_y = self.get_world_coords(u, v_world_space)

        target_x = world_y / 100
        target_y = world_x / 100

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        return distance, angle

    def compute_initial_ball_motion(self, ball):
        self.closest_ball = ball["color"]
        self.world.first_ball = self.closest_ball
        self.drop_target_ids = self.DROP_ARUCO_IDS.get(self.closest_ball)

        self.target_x = ball["world_y"] / 100
        self.target_y = ball["world_x"] / 100

        distance_to_ball = math.hypot(self.target_x, self.target_y)

        self.total_distance = max(0.0, distance_to_ball - self.GRIPPER_DISTANCE)
        self.drive_distance_m = max(
            0.0,
            distance_to_ball - self.GRIPPER_DISTANCE - self.stage_2,
        )

        self.turn_angle_deg = -math.degrees(math.atan2(self.target_y, self.target_x))
        self.turn_angle_deg = self.apply_turn_offset(self.turn_angle_deg)

        self.total_turn = self.turn_angle_deg
        self.return_angle_deg = -self.turn_angle_deg

        print(
            f"[BALL] Closest {self.closest_ball} ball at "
            f"pixel ({ball['u']:.1f}, {ball['v']:.1f}) -> "
            f"world ({ball['world_x']:.2f}, {ball['world_y']:.2f})"
        )
        print(
            f"[BALL] turn={self.turn_angle_deg:.2f} deg, "
            f"stage1_drive={self.drive_distance_m:.2f} m, "
            f"total_return_distance={self.total_distance:.2f} m"
        )
        print(f"[TASK][DROP] {self.closest_ball} ball will use ArUco IDs {self.drop_target_ids}")

    # ---------------------------------------------------------------------
    # ArUco detection and drop helpers
    # ---------------------------------------------------------------------

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

        detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)

        centers = {}

        if ids is None:
            print(f"[TASK][ARUCO] No markers detected. Rejected candidates: {len(rejected)}")
            return centers

        print(f"[TASK][ARUCO] Detected IDs: {ids.flatten().tolist()}")

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners[0]

            center_u = float(np.mean(pts[:, 0]))
            center_v = float(np.mean(pts[:, 1]))

            left_height = np.linalg.norm(pts[0] - pts[3])
            right_height = np.linalg.norm(pts[1] - pts[2])
            marker_height = float((left_height + right_height) / 2.0)

            bottom_u = float((pts[2][0] + pts[3][0]) / 2.0)
            bottom_v = float((pts[2][1] + pts[3][1]) / 2.0)

            floor_u = bottom_u
            floor_v = bottom_v + (0.65 * marker_height)

            centers[int(marker_id)] = (floor_u, floor_v)

            print(
                f"[ARUCO] ID {int(marker_id)} "
                f"center=({center_u:.1f}, {center_v:.1f}), "
                f"floor=({floor_u:.1f}, {floor_v:.1f}), "
                f"height={marker_height:.1f}px"
            )

        return centers

    def compute_aruco_drop_motion(self):
        self.detect_aruco_cam = False

        if self.closest_ball not in self.DROP_ARUCO_IDS:
            print(f"[TASK][DROP] No ArUco mapping for ball color: {self.closest_ball}")
            return False

        self.drop_target_ids = self.DROP_ARUCO_IDS[self.closest_ball]

        frame_cal = self.get_calibrated_frame()
        if frame_cal is None:
            print("[TASK][ARUCO] Camera frame capture failed")
            return False

        # Do not crop for ArUco detection.
        centers = self.detect_aruco_centers(frame_cal)

        visible_ids = [marker_id for marker_id in self.drop_target_ids if marker_id in centers]

        target_u = None
        target_v = None

        if len(visible_ids) >= 2:
            id1, id2 = visible_ids[0], visible_ids[1]
            u1, v1 = centers[id1]
            u2, v2 = centers[id2]
            target_u = (u1 + u2) / 2.0
            target_v = (v1 + v2) / 2.0
            print(f"[TASK][DROP] Using midpoint between ArUco {id1} and {id2}")

        elif len(visible_ids) == 1:
            id1 = visible_ids[0]
            target_u, target_v = centers[id1]
            print(f"[TASK][DROP] Only ArUco {id1} detected, using its floor point")

        else:
            print(
                f"[TASK][DROP] Could not see target ArUco IDs {self.drop_target_ids}. "
                f"Detected: {list(centers.keys())}"
            )
            return False

        world_x, world_y = self.get_world_coords(target_u, target_v)

        print(
            f"[DROP] Target pixel ({target_u:.1f}, {target_v:.1f}) "
            f"-> world ({world_x:.2f}, {world_y:.2f})"
        )

        self.target_x = world_y / 100
        self.target_y = world_x / 100

        self.drive_distance_m = max(
            0.0,
            math.hypot(self.target_x, self.target_y) - self.GRIPPER_DISTANCE,
        )

        self.turn_angle_deg = -math.degrees(math.atan2(self.target_y, self.target_x))
        self.turn_angle_deg = self.apply_turn_offset(self.turn_angle_deg)

        self.detect_aruco_cam = True

        print(
            f"[DROP] ArUco target for {self.closest_ball}: "
            f"turn={self.turn_angle_deg:.2f} deg, "
            f"distance={self.drive_distance_m:.2f} m"
        )

        return True

    # ---------------------------------------------------------------------
    # Task lifecycle
    # ---------------------------------------------------------------------

    def start(self):
        super().start()

        print("[TASK][TASK] PickAndDropBallTask started")

        self.state = 0
        self.detect_ball_cam = False
        self.detect_aruco_cam = False
        self.closest_ball = None
        self.drop_target_ids = None

        ball = self.find_closest_ball()

        if ball is None:
            print("[TASK][BALL] No target balls detected")
            return

        self.detect_ball_cam = True
        self.compute_initial_ball_motion(ball)

    def update(self):
        # -----------------------------------------------------------------
        # Pickup sequence
        # -----------------------------------------------------------------

        # Open gripper and lower arm.
        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            self.servo_controller.servo_control(2, 0, 300)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, 210, 300)
                self.servo_controller.servo_control(2, 0, 300)
                self.state = 2

        elif self.state == 2:
            if not self.detect_ball_cam:
                print("[TASK][TASK] No ball found. Ending task.")
                self.state = 99
            else:
                self.state = 3

        # Turn toward closest ball.
        elif self.state == 3:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK][BALL] Turning {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 4
            else:
                self.state = 5

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.state = 5

        # Drive most of the way to the ball.
        elif self.state == 5:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK][BALL] Stage 1 driving by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed_ball_stage_1,
                )
            self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                print("[TASK][BALL] Stage 1 complete, doing close-range correction")
                result = self.detect_and_compute_ball_target()

                if result is None:
                    print("[TASK][BALL] Lost ball after stage 1. Ending pickup sequence.")
                    self.state = 99
                else:
                    distance, angle = result

                    self.drive_error = distance - (self.GRIPPER_DISTANCE + self.stage_2)
                    self.drive_distance_m = max(0.0, distance - self.GRIPPER_DISTANCE)
                    self.drive_distance_m = self.drive_distance_m + self.drive_error
                    self.drive_distance_m = max(0.0, self.drive_distance_m)

                    self.turn_angle_deg = angle

                    print(f"[TASK][BALL] Stage 2 correction turn={self.turn_angle_deg:.2f} deg")
                    print(f"[TASK][BALL] Stage 2 drive={self.drive_distance_m:.2f} m")
                    print(f"[TASK][BALL] Move error={self.drive_error:.2f} m")

                    self.state = 7

        # Correct final angle to ball.
        elif self.state == 7:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK][BALL] Stage 2 turning {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 8
            else:
                self.state = 9

        elif self.state == 8:
            if not self.motion_controller.is_busy():
                self.state = 9

        # Final drive to ball.
        elif self.state == 9:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK][BALL] Stage 2 driving by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed_ball_stage_2,
                )
                self.state = 10
            else:
                self.state = 11

        # Grip ball and reverse back to original distance.
        elif self.state == 10:
            if not self.motion_controller.is_busy():
                print(f"[TASK][BALL] Gripping {self.closest_ball} ball")
                self.servo_controller.servo_control(2, -1000, 500)
                time.sleep(0.5)
                self.motion_controller.drive_distance(self.total_distance, -0.3)
                self.state = 11

        # Undo initial turn to approximately restore original pose.
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                print(f"[TASK][BALL] Returning heading by {-self.total_turn:.2f} deg")
                self.motion_controller.turn_in_place(math.radians(-self.total_turn))
                self.state = 12

        # -----------------------------------------------------------------
        # Pre-ArUco positioning sequence
        # Only this part changes depending on ball color.
        # After this, both red and blue go to the same shared ArUco
        # detect -> drive -> drop -> drive back sequence.
        # -----------------------------------------------------------------

        elif self.state == 12:
            if not self.motion_controller.is_busy():
                if self.closest_ball == "blue":
                    print("[TASK][PRE-ARUCO] Blue ball: turning toward ArUco view pose")
                    # TODO: replace 0 with your calibrated blue pre-ArUco turn angle.
                    self.motion_controller.turn_in_place(math.radians(-50))
                    self.state = 20

                elif self.closest_ball == "red":
                    print("[TASK][PRE-ARUCO] Red ball: turning slightly before line search")
                    # TODO: tune this small red pre-line turn angle.
                    self.motion_controller.turn_in_place(math.radians(-10))
                    self.state = 30

                else:
                    print(f"[TASK][PRE-ARUCO] Unknown ball color: {self.closest_ball}")
                    self.state = 99

        # ---------------- BLUE PRE-ARUCO PATH ----------------
        elif self.state == 20:
            if not self.motion_controller.is_busy():
                print("[TASK][PRE-ARUCO] Blue positioning complete")
                self.state = 40

        # ---------------- RED PRE-ARUCO PATH ----------------
        elif self.state == 30:
            if not self.motion_controller.is_busy():
                print("[TASK][PRE-ARUCO] Red: driving forward to find line")
                # TODO: replace this placeholder with actual line-follow / line-detect logic.
                self.motion_controller.drive_to_line(0.2)
                # self.motion_controller.drive_distance(0.3, 0.1)
                self.state = 31

        elif self.state == 31:
            if not self.motion_controller.is_busy():
                print("[TASK][PRE-ARUCO] Red: line reached, turning 180 before ArUco detection")
                self.motion_controller.turn_in_place(math.radians(-150))
                self.state = 32

        elif self.state == 32:
            if not self.motion_controller.is_busy():
                print("[TASK][PRE-ARUCO] Red: line reached, turning 180 before ArUco detection")
                self.motion_controller.drive_distance(0.4,0.2)
                self.state = 33

        elif self.state == 33:
            if not self.motion_controller.is_busy():
                print("[TASK][PRE-ARUCO] Red positioning complete")
                self.state = 40

        # -----------------------------------------------------------------
        # Shared ArUco sequence for BOTH red and blue
        # detect ArUco -> turn -> drive -> drop -> drive back
        # -----------------------------------------------------------------

        elif self.state == 40:
            if not self.motion_controller.is_busy():
                print(f"[TASK][DROP] Looking for ArUco drop zone for {self.closest_ball} ball")
                found = self.compute_aruco_drop_motion()

                if not found:
                    print("[TASK][DROP] Could not compute ArUco drop target. Ending task.")
                    self.state = 99
                else:
                    self.state = 13

        # Turn toward ArUco drop target.
        elif self.state == 13:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(f"[TASK][DROP] Turning {direction} by {abs(self.turn_angle_deg):.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 14
            else:
                self.state = 15

        elif self.state == 14:
            if not self.motion_controller.is_busy():
                self.state = 15

        # Drive to drop target.
        elif self.state == 15:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK][DROP] Driving to ArUco target by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed_aruco,
                )
                self.state = 16
            else:
                self.state = 17

        # Drop ball.
        elif self.state == 16:
            if not self.motion_controller.is_busy():
                print(f"[TASK][DROP] Releasing {self.closest_ball} ball")
                self.servo_controller.servo_control(2, 0, 600)
                time.sleep(0.3)
                self.state = 17

        # Back away from drop target.
        elif self.state == 17:
            if not self.motion_controller.is_busy():
                if self.drive_distance_m > 1e-6:
                    print(f"[TASK][DROP] Backing away by {self.drive_distance_m:.2f} m")
                    self.motion_controller.drive_distance(
                        self.drive_distance_m,
                        -self.drive_speed_aruco,
                    )
                    self.state = 18
                else:
                    self.state = 19 if self.closest_ball == "blue" else 99

        elif self.state == 18:
            if not self.motion_controller.is_busy():
                if self.closest_ball == "blue":
                    print("[TASK][DROP] Blue ball: turning 50 degrees after backing out")
                    self.motion_controller.turn_in_place(math.radians(60))
                    self.state = 19
                else:
                    self.state = 99

        elif self.state == 19:
            if not self.motion_controller.is_busy():
                self.state = 99

        # Done.
        elif self.state == 99:
            if not self.motion_controller.is_busy():
                print("[TASK] PickAndDropBallTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] PickAndDropBallTask stopped")