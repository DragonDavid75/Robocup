import math
import time
import cv2
import numpy as np
from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.scam import cam


class DriveToHoleTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

        # --- Homography Setup ---
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
            self.pixel_points, self.world_points, cv2.RANSAC, 5.0
        )

        # --- Hole Detector Parameters ---
        self.params = {
            "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
            "brown_upper": np.array([30, 255, 255], dtype=np.uint8),
            "brown_min_area": 300,   # lowered a bit for debugging
            "brown_morph_kernel": (7, 7),
        }

        # --- State Machine ---
        # 0 = detect
        # 1 = turn
        # 2 = wait for turn
        # 3 = drive
        # 4 = wait for drive
        # 5 = servo action / done
        self.state = 0

        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0
        self.servos_done = False

        # Set to False if you are on a headless robot and cv2.imshow crashes
        self.show_debug_windows = False
        self.save_debug_images = True

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)

        if transformed[2] == 0:
            return None

        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]

        return float(world_x), float(world_y)

    def _show_or_save_debug(self, name, image):
        if self.show_debug_windows:
            cv2.imshow(name, image)
        if self.save_debug_images:
            ts = int(time.time() * 1000)
            cv2.imwrite(f"/tmp/{name}_{ts}.jpg", image)

    def detect_hole(self, frame):
        """
        Detect hole in the provided frame (expected to be the bottom-half crop).
        Returns:
            best_candidate: (u, v) center in the cropped frame coordinates, or None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        brown_mask = cv2.inRange(
            hsv,
            self.params["brown_lower"],
            self.params["brown_upper"],
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, self.params["brown_morph_kernel"]
        )
        brown_mask = cv2.morphologyEx(
            brown_mask, cv2.MORPH_CLOSE, kernel, iterations=2
        )

        contours, _ = cv2.findContours(
            brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        debug = frame.copy()
        best_candidate = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.params["brown_min_area"]:
                continue

            (x, y), radius = cv2.minEnclosingCircle(cnt)

            # draw every valid contour for debugging
            cv2.drawContours(debug, [cnt], -1, (255, 0, 0), 2)
            cv2.circle(debug, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            if area > best_area:
                best_area = area
                best_candidate = (int(x), int(y))

        # draw best candidate more clearly
        if best_candidate is not None:
            cv2.circle(debug, best_candidate, 8, (0, 255, 0), -1)
            cv2.putText(
                debug,
                f"best: {best_candidate}",
                (best_candidate[0] + 10, best_candidate[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        self._show_or_save_debug("hole_crop_debug", debug)
        self._show_or_save_debug("brown_mask", brown_mask)

        return best_candidate

    def detect_and_compute_target(self):
        ok, frame, _ = cam.getImage()
        if not ok or frame is None:
            print("Camera read failed")
            return None

        height, width = frame.shape[:2]

        # Look only at the bottom half for detection
        y_offset = height // 2
        crop = frame[y_offset:, :]

        # Show what the camera sees
        self._show_or_save_debug("full_frame", frame)
        self._show_or_save_debug("bottom_half_crop", crop)

        center = self.detect_hole(crop)
        if center is None:
            print("No hole detected in bottom half")
            if self.show_debug_windows:
                cv2.waitKey(1)
            return None

        # Coordinates relative to crop
        u_crop, v_crop = center

        # Convert to FULL image coordinates
        u_full = u_crop
        v_full = v_crop + y_offset

        print(f"Hole center in crop coords: ({u_crop}, {v_crop})")
        print(f"Hole center in full coords: ({u_full}, {v_full})")

        # Draw on full frame too
        full_vis = frame.copy()
        crop_vis = crop.copy()

        cv2.circle(crop_vis, (u_crop, v_crop), 8, (0, 255, 0), -1)
        cv2.putText(
            crop_vis,
            f"crop: ({u_crop},{v_crop})",
            (u_crop + 10, v_crop - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        cv2.circle(full_vis, (u_full, v_full), 8, (0, 255, 0), -1)
        cv2.putText(
            full_vis,
            f"full: ({u_full},{v_full})",
            (u_full + 10, v_full - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        self._show_or_save_debug("crop_with_center", crop_vis)
        self._show_or_save_debug("full_with_center", full_vis)

        world_coords = self.get_world_coords(u_full, v_full)
        if world_coords is None:
            print("Homography transform failed")
            if self.show_debug_windows:
                cv2.waitKey(1)
            return None

        world_x, world_y = world_coords
        print(f"World coords: x={world_x:.2f}, y={world_y:.2f}")

        # Convert cm to meters and align axes to robot frame
        target_x = world_y / 100.0
        target_y = world_x / 100.0

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        print(f"Target distance: {distance:.3f} m")
        print(f"Target angle: {angle:.2f} deg")

        if self.show_debug_windows:
            cv2.waitKey(1)

        return distance, angle

    def update(self):
        if self.state == 0:
            result = self.detect_and_compute_target()
            if result is not None:
                self.drive_distance_m, self.turn_angle_deg = result
                self.state = 1
            return TaskStatus.RUNNING

        elif self.state == 1:
            if abs(self.turn_angle_deg) > 3.0:
                self.motion_controller.turn_in_place(
                    math.radians(self.turn_angle_deg)
                )
                self.state = 2
            else:
                self.state = 3
            return TaskStatus.RUNNING

        elif self.state == 2:
            if self.motion_controller.is_busy():
                return TaskStatus.RUNNING
            self.state = 3
            return TaskStatus.RUNNING

        elif self.state == 3:
            if self.drive_distance_m > 0.1:
                self.motion_controller.follow_for_distance(
                    self.drive_distance_m - 0.1, 0.1
                )
                self.state = 4
            else:
                self.state = 5
            return TaskStatus.RUNNING

        elif self.state == 4:
            if self.motion_controller.is_busy():
                return TaskStatus.RUNNING
            self.state = 5
            return TaskStatus.RUNNING

        elif self.state == 5:
            if not self.servos_done:
                self.servo_controller.servo_control(1, 200, 300)
                self.servo_controller.servo_control(2, 0, 300)
                self.servos_done = True
            return TaskStatus.DONE

        return TaskStatus.RUNNING
