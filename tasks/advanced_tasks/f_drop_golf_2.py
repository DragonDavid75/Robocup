"""
tasks/advanced_tasks/f_drop_golf_2.py

6th task: Drop second ball

Start: Facing the hole. Arm up and closed with the ball.
Functionality: Drive to hole, and drop the ball. Turn around.
End: Next to the hole(l3) facing away from it. Arm down and open.

Status:
"""

import math
import time
import os
import cv2
import numpy as np
from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.scam import cam


class DriveToHole2Task(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

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

        self.params = {
            "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
            "brown_upper": np.array([30, 255, 255], dtype=np.uint8),

            # Red/orange exclusion mask using your values
            "red_lower1": np.array([0, 10, 80], dtype=np.uint8),
            "red_upper1": np.array([10, 255, 255], dtype=np.uint8),
            "red_lower2": np.array([170, 10, 80], dtype=np.uint8),
            "red_upper2": np.array([180, 255, 255], dtype=np.uint8),

            "brown_min_area": 300,
            "brown_morph_kernel": (7, 7),
        }

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
            self.mtx, self.dist, (w, h), 1, (w, h)
        )

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.mtx,
            self.dist,
            None,
            self.newcameramtx,
            (w, h),
            cv2.CV_32FC1,
        )

        self.state = 0
        self.hole_detected = False

        self.GRIPPER_DISTANCE = self.world.gripper_distance
        self.stage_2 = self.world.stage_2

        self.drive_distance_m = 0.0
        self.turn_angle_deg = 0.0
        self.servos_done = False

        self.show_debug_windows = False
        self.save_debug_images = False
        self.debug_dir = "/home/local/debug_images"
        self.debug_counter = 0
        self.debug_save_every = 1

        os.makedirs(self.debug_dir, exist_ok=True)

    def get_world_coords(self, u, v):
        pixel_vector = np.array([u, v, 1.0], dtype=np.float32).reshape(3, 1)
        transformed = np.dot(self.H, pixel_vector)

        if transformed[2] == 0:
            return None

        world_x = transformed[0] / transformed[2]
        world_y = transformed[1] / transformed[2]

        return float(world_x), float(world_y)

    def _show_or_save_debug(self, name, image):
        if image is None:
            return

        if self.show_debug_windows:
            cv2.imshow(name, image)
            cv2.waitKey(1)

        if self.save_debug_images:
            latest_path = os.path.join(self.debug_dir, f"latest_{name}.jpg")
            cv2.imwrite(latest_path, image)

            if self.debug_counter % self.debug_save_every == 0:
                ts = int(time.time() * 1000)
                numbered_path = os.path.join(self.debug_dir, f"{name}_{ts}.jpg")
                cv2.imwrite(numbered_path, image)

    def detect_hole(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        brown_mask = cv2.inRange(
            hsv,
            self.params["brown_lower"],
            self.params["brown_upper"],
        )

        red_mask1 = cv2.inRange(
            hsv,
            self.params["red_lower1"],
            self.params["red_upper1"],
        )

        red_mask2 = cv2.inRange(
            hsv,
            self.params["red_lower2"],
            self.params["red_upper2"],
        )

        red_orange_mask = cv2.bitwise_or(red_mask1, red_mask2)

        brown_mask = cv2.bitwise_and(
            brown_mask,
            cv2.bitwise_not(red_orange_mask),
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            self.params["brown_morph_kernel"],
        )

        brown_mask = cv2.morphologyEx(
            brown_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2,
        )

        contours, _ = cv2.findContours(
            brown_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        debug = frame.copy()
        best_candidate = None
        best_area = 0.0

        height, width = frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < self.params["brown_min_area"]:
                continue

            (x, y), radius = cv2.minEnclosingCircle(cnt)

            # 🚫 FILTER: ignore bottom 1/3 of the CROP
            if y > int(height * (2/3)):
                continue

            cv2.drawContours(debug, [cnt], -1, (255, 0, 0), 2)
            cv2.circle(debug, (int(x), int(y)), int(radius), (0, 255, 255), 2)

            if area > best_area:
                best_area = area
                best_candidate = (int(x), int(y))

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

        self._show_or_save_debug("filtered_hole_debug", debug)
        self._show_or_save_debug("brown_mask_filtered", brown_mask)

        return best_candidate

    def detect_and_compute_target(self):
        self.debug_counter += 1

        ok, frame, _ = cam.getImage()
        if not ok or frame is None:
            print("Camera read failed")
            return None

        frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)

        height, width = frame.shape[:2]

        # Bottom third of image
        y_offset = int(height * 2 / 3)
        crop = frame[y_offset:, :]

        self._show_or_save_debug("full_frame", frame)
        self._show_or_save_debug("bottom_third_crop", crop)

        center = self.detect_hole(crop)

        full_vis = frame.copy()
        crop_vis = crop.copy()

        if center is None:
            print("No hole detected in bottom third")
            self._show_or_save_debug("full_with_center", full_vis)
            self._show_or_save_debug("crop_with_center", crop_vis)
            return None

        u_crop, v_crop = center

        u_full = u_crop
        v_full = v_crop + y_offset

        print(f"Hole center in crop coords: ({u_crop}, {v_crop})")
        print(f"Hole center in full coords: ({u_full}, {v_full})")

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
            return None

        world_x, world_y = world_coords
        print(f"World coords: x={world_x:.2f}, y={world_y:.2f}")

        target_x = world_y / 100.0
        target_y = world_x / 100.0

        distance = math.hypot(target_x, target_y)
        angle = -math.degrees(math.atan2(target_y, target_x))

        print(f"TASK Target distance: {distance:.3f} m")
        print(f"TASK Target angle: {angle:.2f} deg")

        return distance, angle

    def start(self):
        super().start()
        print("[TASK] Hole Task Started")

        self.state = 0

        self.servo_controller.servo_control(1, 200, 500)
        time.sleep(0.5)

        result = self.detect_and_compute_target()

        if result is not None:
            self.hole_detected = True
            print("[TASK] Hole detected at start")
            distance, turn = result
            self.drive_distance_m = distance - self.GRIPPER_DISTANCE - self.stage_2
            self.drive_distance_m = max(0.0, self.drive_distance_m)
            self.turn_angle_deg = turn
        else:
            self.hole_detected = False

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached - open gripper and lower the arm")
                self.servo_controller.servo_control(1, 210, 300)
                self.servo_controller.servo_control(2, -1000, 300)
                self.state = 2

        elif self.state == 2:
            if not self.hole_detected:
                self.state = 13
            else:
                self.state = 3

        elif self.state == 3:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(
                    f"[TASK] Stage 0 - Turning {direction} by "
                    f"{abs(self.turn_angle_deg):.2f} deg"
                )
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 4
            else:
                self.state = 5

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.state = 5

        elif self.state == 5:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Stage 1 - Driving by {self.drive_distance_m:.2f} m")
                self.motion_controller.drive_distance(self.drive_distance_m, 0.2)
            self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                print("[TASK] Stage 1 complete, starting stage 2 correction")

                result = self.detect_and_compute_target()

                if result is None:
                    print("[TASK] Lost hole after stage 1")
                    self.state = 12
                else:
                    distance, angle = result
                    self.drive_distance_m = max(0.0, distance - self.GRIPPER_DISTANCE)
                    self.turn_angle_deg = angle
                    self.state = 7

        elif self.state == 7:
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(
                    f"[TASK] Stage 2 - Correcting turn {direction} by "
                    f"{abs(self.turn_angle_deg):.2f} deg"
                )
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 8
            else:
                self.state = 9

        elif self.state == 8:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, -330, 300)
                self.state = 9

        elif self.state == 9:
            if self.drive_distance_m > 1e-6:
                print(f"[TASK] Stage 2 - Final drive by {self.drive_distance_m:.2f} m")
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
            if not self.motion_controller.is_busy():
                print("[TASK] turn right")
                self.motion_controller.turn_in_place(math.radians(150))
                self.state = 13

        elif self.state == 13:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveToPoint completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveToPoint stopped")
