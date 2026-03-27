# tasks/exit_roundabout.py

from tasks.base_task import BaseTask, TaskStatus
import math


class ExitRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] ExitRoundaboutTask started")
        self.state = 0

    def update(self):

        # STATE 0: Turn 90° RIGHT to face exit direction
        if self.state == 0:
            print("[TASK] Turning 90° right to exit")
            self.motion_controller.turn_in_place(math.radians(90))  # RIGHT turn
            self.state = 1

        # STATE 1: Wait for turn to finish
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving forward 25 cm")
                self.state = 2

        # STATE 2: Drive forward 25 cm
        elif self.state == 2:
            self.motion_controller.drive_distance(0.25, 0.08)  # 25 cm
            self.state = 3

        # STATE 3: Wait for forward motion to finish
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] ExitRoundaboutTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] ExitRoundaboutTask stopped")
