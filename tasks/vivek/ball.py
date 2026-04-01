"""
tasks/ball.py
"""

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class BallTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, distance=1.0, velocity=0.2):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.start_time = 0
        self.distance = distance
        self.velocity = velocity

    def start(self):
        super().start()
        print("[TASK] BallTask started")
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Start the drive-to-ball task
            self.motion_controller.drive_to_ball()
            self.state = 1


        elif self.state == 1:
            if self.motion_controller.is_busy():
                print("[TASK] BallTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BallTask stopped")
