# tasks/bonus_time.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class BonusTimeTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] BonusTimeTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            print("[TASK] BonusTimeTask: State 0")
            # Start following the line
            self.motion_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(5, 0.4, left_side=True, ref_pos=0.0)
            self.state = 1

        elif self.state == 1:
            print("[TASK] BonusTimeTask: State 1")
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(2.2, -0.2)
                self.state = 2

        elif self.state == 2:
            print("[TASK] BonusTimeTask: State 2")
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(2.1, 0.2)
                self.state = 3

        elif self.state == 3:
            print("[TASK] BonusTimeTask: State 3")
            if not self.motion_controller.is_busy():
                self.motion_controller.follow_for_distance(5, 0.4, left_side=True, ref_pos=0.0)
                self.state = 4

        elif self.state == 4:
            print("[TASK] BonusTimeTask: State 4")
            if not self.motion_controller.is_busy():
                print("[TASK] BonusTimeTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BonusTimeTask stopped")
