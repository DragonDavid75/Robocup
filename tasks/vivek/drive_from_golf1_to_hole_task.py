# tasks/vivek/drive_to_point_task.py

import math
from tasks.base_task import BaseTask, TaskStatus


class DriveGolfToHoleTask(BaseTask):
    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.state = 0

    def start(self):
        super().start()
        self.state = 0

        print(
            f"[TASK] DriveGolfToHoleTask started: "
        )

    def update(self):
        # Step 1: turn to face the point
        if self.state == 0:
            print("[TASK] Exiting sea saw")
            self.motion_controller.drive_distance(0.7, 0.1)
            # self.motion_controller.follow_until_line_loss(0.1)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2

        # Step 2: drive along the hypotenuse to the point
        elif self.state == 2:
            print("[TASK] Drive till we find a line")
            self.motion_controller.drive_to_line(0.1)
            self.state = 3

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.state = 4

        # Step 3: turn back to the original heading
        elif self.state == 4:
            print("[TASK] turn right")
            self.motion_controller.turn_in_place(math.radians(-90))
            self.state = 5

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.state = 6
        
        elif self.state == 6:
            print("[TASK] follow until intersection")
            self.motion_controller.follow_until_intersection_or_end_line(0.3)
            self.state = 7

        elif self.state == 7:
            if not self.motion_controller.is_busy():
                self.state = 8

        elif self.state == 8:
            print("[TASK] turn right on the intersection")
            self.motion_controller.follow_for_distance(1.1,0.3,action="RIGHT")
            self.state = 9

        elif self.state == 9:
            if not self.motion_controller.is_busy():
                self.state = 10

        elif self.state == 10:
            print("[TASK] DriveToPoint completed")
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] DriveToPoint stopped")
