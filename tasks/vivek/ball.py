"""
tasks/ball.py
"""

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
from core.drive_to_ball import DriveToBallTask
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
            self.drive_to_ball_task = DriveToBallTask(self.world)
            self.drive_to_ball_task.start()
            self.state = 1


        elif self.state == 1:
            if not self.drive_to_ball_task.is_alive():
                print("[TASK] BallTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BallTask stopped")
