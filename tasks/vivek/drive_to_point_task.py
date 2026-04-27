# tasks/vivek/drive_to_point_task.py

import math
from tasks.base_task import BaseTask, TaskStatus
import time


class DriveToPointTask(BaseTask):
    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
        target_x=0.0,
        target_y=0.0,
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller

        # Local frame:
        # start at (0, 0)
        # +x = forward
        # -x = backward
        # +y = right
        # -y = left
        self.target_x = float(target_x)
        self.target_y = float(target_y)

        # States:
        # 0 = turn toward point
        # 1 = wait for first turn
        # 2 = drive to point
        # 3 = wait for drive
        # 4 = turn back to original heading
        # 5 = wait for final turn
        # 6 = done
        self.state = 0

        # Speeds
        self.turn_speed = 0.3
        self.drive_speed = 0.15

        # Computed at start
        self.turn_angle_deg = 0.0
        self.drive_distance_m = 0.0
        self.return_angle_deg = 0.0

    def start(self):
        super().start()
        self.state = 0

        # Distance to point
        self.drive_distance_m = math.hypot(self.target_x, self.target_y)

        # Angle to point from current heading
        #
        # Using your coordinate system:
        # +x = forward
        # +y = right
        #
        # atan2(y, x) gives:
        #  - positive angle for points to the right
        #  - negative angle for points to the left
        #
        # This assumes your controller uses:
        # +angle = right turn
        # -angle = left turn
        self.turn_angle_deg = -math.degrees(
            math.atan2(self.target_y, self.target_x)
        )

        # Turn back to original heading after reaching the point
        self.return_angle_deg = -self.turn_angle_deg

        print(
            f"[TASK] DriveToPoint started: "
            f"target_x={self.target_x:.2f}, "
            f"target_y={self.target_y:.2f}, "
            f"turn={self.turn_angle_deg:.2f} deg, "
            f"distance={self.drive_distance_m:.2f} m"
        )

    def update(self):
        # Step 1: turn to face the point
        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            if abs(self.turn_angle_deg) > 1e-6:
                direction = "right" if self.turn_angle_deg > 0 else "left"
                print(
                    f"[TASK] Turning {direction} by "
                    f"{abs(self.turn_angle_deg):.2f} deg"
                )
                self.motion_controller.turn_in_place(math.radians(self.turn_angle_deg))
                self.state = 1
            else:
                self.state = 2

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, -250, 200)
                self.state = 2

        # Step 2: drive along the hypotenuse to the point
        elif self.state == 2:
            if self.drive_distance_m > 1e-6:
                print(
                    f"[TASK] Driving to point by "
                    f"{self.drive_distance_m:.2f} m"
                )
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed
                )
                self.state = 3
            else:
                self.state = 4

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, -250, 200)
                self.state = 4

        # Step 3: turn back to the original heading
        elif self.state == 4:
            if abs(self.return_angle_deg) > 1e-6:
                direction = "right" if self.return_angle_deg > 0 else "left"
                print(
                    f"[TASK] Turning back {direction} by "
                    f"{abs(self.return_angle_deg):.2f} deg"
                )
                time.sleep(1)
                # self.motion_controller.turn_in_place(math.radians(self.return_angle_deg))
                self.state = 5
            else:
                self.state = 6

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                time.sleep(0.5)
                self.servo_controller.servo_control(2, 0, 500)
                self.motion_controller.turn_in_place(math.radians(162))
                self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(0.2, 0.2)
                self.state = 7

        elif self.state == 7:
            print("[TASK] DriveToPoint completed")
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        # self.motion_controller.stop()
        print("[TASK] DriveToPoint stopped")
