import math
import time
import os
import cv2
import numpy as np
from tasks.base_task import BaseTask, TaskStatus


class DriveToArUcoTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller, target_ids=(12, 14)):
        super().__init__(world, motion_controller, servo_controller)

        self.target_ids = list(target_ids)

        self.stream_url = "http://10.197.219.91:7123/stream.mjpg"
        self.cap = cv2.VideoCapture(self.stream_url)

        if not self.cap.isOpened():
            print(f"[TASK] WARNING: Could not open stream: {self.stream_url}")

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.parameters.adaptiveThreshConstant = 10
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.perspectiveRemovePixelPerCell = 8
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, self.parameters)

        self.pixel_points = np.array(
            [
                (110, 551),
                (716, 552),
                (249, 372),
                (565, 372),
                (408, 432),
                (406, 372),
                (414, 551),
                (201, 434),
                (615, 432),
            ],
            dtype=np.float32,
        )

        self.world_points = np.array(
            [
                (-15, 30),
                (15, 30),
                (-15, 60),
                (15, 60),
                (0, 30),
                (0, 45),
                (0, 60),
                (-15, 45),
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

        self.state = 0
        self.target_detected = False

        self.GRIPPER_DISTANCE = 0.30
        self.stage_2 = 0.20

        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0

        self.debug_dir = "/home/local/debug_images"
        self.debug_counter = 0
        self.debug_save_every = 1
        self.save_debug_images = True
        os.makedirs(self.debug_dir, exist_ok=True)

    def _save_debug(self, name, image):
        if image is None or not self.save_debug_images:
            return

        latest_path = os.path.join(self.debug_dir, f"latest_{name}.jpg")
        cv2.imwrite(latest_path, image)

        if self.debug_counter % self.debug_save_every == 0:
            ts = int(time.time() * 1000)
            numbered_path = os.path.join(self.debug_dir, f"{name}_{ts}.jpg")
            cv2.imwrite(numbered_path, image)

    def set_target_ids(self, new_ids):
        self.target_ids = list(new_ids)
        print(f"[TASK] Target IDs updated to: {self.target_ids}")

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)

        if transformed[2] == 0:
            return None

        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]

        return float(world_x), float(world_y)

    def get_frame_from_stream(self):
        if self.cap is None or not self.cap.isOpened():
            print("[TASK] Reopening camera stream...")
            self.cap = cv2.VideoCapture(self.stream_url)

        ok, frame = self.cap.read()

        if not ok or frame is None:
            print("[TASK] Stream read failed")
            return None

        return frame

    def extract_aruco_positions(self, corners, ids, frame_height):
        positions = {}
        lowest_marker = None

        if ids is None or len(ids) == 0:
            return positions, lowest_marker

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            points = marker_corners.reshape(-1, 2)

            center_x = int(np.mean(points[:, 0]))
            center_y = int(np.mean(points[:, 1]))

            bottom_y = int(np.max(points[:, 1]))
            bottom_points = points[points[:, 1] >= bottom_y - 1]
            bottom_x = int(np.mean(bottom_points[:, 0])) if bottom_points.size else center_x

            width = np.linalg.norm(points[0] - points[1])
            height = np.linalg.norm(points[1] - points[2])
            marker_size = int((width + height) / 2)

            y_offset = int(marker_size * 0.85)
            adjusted_y = min(bottom_y + y_offset, frame_height - 1)

            positions[int(marker_id)] = {
                "x": center_x,
                "y": adjusted_y,
                "center_x": center_x,
                "center_y": center_y,
                "bottom_x": bottom_x,
                "bottom_y": bottom_y,
                "adjusted_y": adjusted_y,
                "size": marker_size,
            }

        if positions:
            lowest_id = max(positions.keys(), key=lambda key: positions[key]["bottom_y"])
            lowest_marker = {"id": lowest_id, **positions[lowest_id]}

        return positions, lowest_marker

    def detect_aruco_target(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)
        positions, lowest = self.extract_aruco_positions(corners, ids, frame.shape[0])

        mask = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            23,
            self.parameters.adaptiveThreshConstant,
        )

        debug = frame.copy()

        detected_ids = [] if ids is None else ids.flatten().tolist()
        print(f"[TASK] Detected ArUco IDs: {detected_ids}")
        print(f"[TASK] Looking for target IDs: {self.target_ids}")

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

            for marker_id, info in positions.items():
                cv2.circle(debug, (info["x"], info["y"]), 4, (0, 255, 0), -1)
                cv2.putText(
                    debug,
                    f"ID {marker_id}: ({info['x']},{info['y']}) size={info['size']}",
                    (info["x"] + 5, info["y"] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            if lowest is not None:
                cv2.putText(
                    debug,
                    f"Lowest: id={lowest['id']} x={lowest['x']} y={lowest['y']}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.circle(debug, (lowest["x"], lowest["y"]), 8, (0, 255, 255), 2)

        elif rejected is not None and len(rejected) > 0:
            cv2.aruco.drawDetectedMarkers(debug, rejected, borderColor=(0, 0, 255))
            print(f"[TASK] Rejected candidates: {len(rejected)}")

        else:
            cv2.putText(
                debug,
                "No ArUco markers or candidates detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            print("[TASK] No ArUco markers or candidates detected")

        visible_targets = []

        for target_id in self.target_ids:
            if target_id in positions:
                visible_targets.append((target_id, positions[target_id]))

        if len(visible_targets) == 0:
            cv2.putText(
                debug,
                f"No target IDs found: {self.target_ids}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            self._save_debug("aruco_raw_frame", frame)
            self._save_debug("aruco_debug", debug)
            self._save_debug("aruco_tuning_mask", mask)

            print(f"[TASK] None of target IDs {self.target_ids} found")
            return None

        elif len(visible_targets) == 1:
            marker_id, info = visible_targets[0]

            target_x = info["x"]
            target_y = info["y"]

            cv2.circle(debug, (target_x, target_y), 10, (0, 255, 255), 2)
            cv2.putText(
                debug,
                f"Using single ID {marker_id}: ({target_x},{target_y})",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            print(f"[TASK] Only ID {marker_id} detected. Using its point: ({target_x}, {target_y})")

        else:
            id1, info1 = visible_targets[0]
            id2, info2 = visible_targets[1]

            x1, y1 = info1["x"], info1["y"]
            x2, y2 = info2["x"], info2["y"]

            target_x = int((x1 + x2) / 2)
            target_y = int((y1 + y2) / 2)

            cv2.circle(debug, (x1, y1), 8, (255, 0, 0), 2)
            cv2.circle(debug, (x2, y2), 8, (255, 0, 0), 2)
            cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(debug, (target_x, target_y), 10, (0, 255, 255), -1)

            cv2.putText(
                debug,
                f"Midpoint IDs {id1}-{id2}: ({target_x},{target_y})",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            print(f"[TASK] Both IDs detected: {id1}, {id2}")
            print(f"[TASK] Midpoint: ({target_x}, {target_y})")

        self._save_debug("aruco_raw_frame", frame)
        self._save_debug("aruco_debug", debug)
        self._save_debug("aruco_tuning_mask", mask)

        return target_x, target_y

    def detect_and_compute_target(self):
        self.debug_counter += 1

        frame = self.get_frame_from_stream()
        if frame is None:
            return None

        center = self.detect_aruco_target(frame)

        if center is None:
            return None

        u_full, v_full = center

        world_coords = self.get_world_coords(u_full, v_full)
        if world_coords is None:
            print("[TASK] Homography transform failed")
            return None

        world_x, world_y = world_coords

        target_x = world_y / 100.0
        target_y = world_x / 100.0

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        print(f"[TASK] Target point pixel: ({u_full}, {v_full})")
        print(f"[TASK] World coords: x={world_x:.2f}, y={world_y:.2f}")
        print(f"[TASK] Target distance: {distance:.3f} m")
        print(f"[TASK] Target angle: {angle:.2f} deg")

        return distance, angle

    def start(self):
        super().start()

        self.state = 0
        self.target_detected = False
        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0

        self.servo_controller.servo_control(1, 200, 500)
        time.sleep(0.5)

        print("[TASK] Starting DriveToArUcoTask with target IDs:", self.target_ids)

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, 210, 300)
                self.servo_controller.servo_control(2, -1000, 300)
                self.state = 2

        elif self.state == 2:
            result = self.detect_and_compute_target()

            if result is None:
                print("[TASK] Searching for ArUco target IDs...")
                return TaskStatus.RUNNING

            self.target_detected = True
            distance, turn = result

            self.drive_distance_m = max(
                0.0,
                distance - self.GRIPPER_DISTANCE - self.stage_2,
            )

            self.turn_angle_deg = turn

            print(
                f"[TASK] Initial target acquired. "
                f"Drive: {self.drive_distance_m:.2f} m, "
                f"Turn: {self.turn_angle_deg:.2f} deg"
            )

            self.state = 3

        elif self.state == 3:
            if abs(self.turn_angle_deg) > 1e-6:
                print(f"[TASK] Turning by {self.turn_angle_deg:.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 4
            else:
                self.state = 5

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.state = 5

        elif self.state == 5:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Driving by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(self.drive_distance_m, 0.2)
            self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                result = self.detect_and_compute_target()

                if result is None:
                    print("[TASK] Lost target after first drive. Searching again...")
                    self.state = 2
                    return TaskStatus.RUNNING

                distance, angle = result

                self.drive_distance_m = max(
                    0.0,
                    distance - self.GRIPPER_DISTANCE,
                )
                self.turn_angle_deg = angle

                print(
                    f"[TASK] Correction target acquired. "
                    f"Drive: {self.drive_distance_m:.2f} m, "
                    f"Turn: {self.turn_angle_deg:.2f} deg"
                )

                self.state = 7

        elif self.state == 7:
            if abs(self.turn_angle_deg) > 1e-6:
                print(f"[TASK] Correction turn by {self.turn_angle_deg:.2f} deg")
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 8
            else:
                self.state = 9

        elif self.state == 8:
            if not self.motion_controller.is_busy():
                self.state = 9

        elif self.state == 9:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Final drive by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(self.drive_distance_m, 0.2)
                self.state = 10
            else:
                self.state = 11

        elif self.state == 10:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, 0, 300)
                time.sleep(1)
                self.state = 11

        elif self.state == 11:
            self.servo_controller.servo_control(1, -500, 20)
            self.state = 12

        elif self.state == 12:
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()

        if self.cap is not None:
            self.cap.release()

        print("[TASK] DriveToArUcoTask stopped")
