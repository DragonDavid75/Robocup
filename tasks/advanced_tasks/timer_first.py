# tasks/timer_first.py

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class TimerFirstTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] TimerFirstTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(0.2, 0.2)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(-1.45)
                self.state = 2

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(2.7, 0.4)
                self.state = 3
        #Turn around to go away from timer back to the white line
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(2.90)
                self.state = 4

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(2.7, 0.4)
                self.state = 5

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(-1.57)
                self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                self.motion_controller.follow_for_distance(9.5, 0.6, action="LEFT")
                self.state = 7

        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] TimerFirstTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] TimerFirstTask stopped")
