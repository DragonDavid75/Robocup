# tasks/vivek/drive_to_point_task.py

import math
from tasks.base_task import BaseTask, TaskStatus
import time


class TurnAndDriveTask(BaseTask):
    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
        turn=0.0,
        drive=0.0,
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.turn = turn
        self.drive = drive
        self.state = 0

        # Speeds
        self.turn_speed = 0.3
        self.drive_speed = 0.15

    def start(self):
        super().start()
        self.state = 0

        print(
            f"[TASK] DriveToPoint started: "
            f"turn={self.turn:.2f} deg, "
            f"drive={self.drive:.2f} m"
        )

    def update(self):
        # Step 1: turn 
        if self.state == 0:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(math.radians(turn))
                self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(
                    self.drive_distance_m,
                    self.drive_speed
                )     
                self.state = 2

        elif self.state == 2:
            print("[TASK] DriveToPoint completed")
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        # self.motion_controller.stop()
        print("[TASK] DriveToPoint stopped")
