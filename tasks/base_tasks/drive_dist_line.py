"""
tasks/drive_dist_line.py

This module contains the DriveDistLineTask. It positions a servo (e.g., to lower a 
sensor array) and instructs the robot to follow a line for a specified distance 
at a set velocity before stopping.
"""

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveDistLineTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, distance=1.0, velocity=0.2):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.distance = distance
        self.velocity = velocity

    def start(self):
        super().start()
        print("[TASK] DriveDistLineTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # Start following the line
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(self.distance, self.velocity)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveDistLineTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveDistLineTask stopped")
