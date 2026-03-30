# tasks/first_ball.py

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class FirstBallTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] FirstBallTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, 100, 300)
            self.motion_controller.follow_for_distance(0.8, 0.3)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(0.5, 0.1)
                self.state = 2

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, -800, 300)
                self.motion_controller.follow_until_intersection_or_end_line(0.2)
                self.state = 3

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(-1.57)
                self.state = 4

        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.motion_controller.follow_for_distance(2.0, 0.4)
                self.state = 5

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] FirstBallTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] FirstBallTask stopped")