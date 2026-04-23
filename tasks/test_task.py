# tasks/first_ball.py

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class TestTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] TestTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.drive_to_line(0.2)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] TestTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] TestTask stopped")