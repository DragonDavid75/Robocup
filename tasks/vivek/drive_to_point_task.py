"""
tasks/drive_to_point_task.py
"""

from tasks.base_task import BaseTask, TaskStatus
from core.drive_to_point import DriveToPoint
import time


class DriveToPointTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, target_x=0.0, target_y=0.0, path=None, final_h=None):
        super().__init__(world, motion_controller, servo_controller)
        self.target_x = target_x
        self.target_y = target_y
        self.path = path
        self.final_h = final_h
        self.drive_to_point = None
        self._started = False # Track if we actually handed the task over

    def start(self):
        super().start()
        print("[TASK] DriveToPointTask started")
        self.drive_to_point = DriveToPoint(self.world, self.target_x, self.target_y)
        self.motion_controller.do_task(self.drive_to_point)
        self._started = True

    def update(self):
        # If the motion controller is STILL busy, the task is STILL running
        if self.motion_controller.is_busy():
            return TaskStatus.RUNNING
        
        # If we reached here, the motion controller is NOT busy anymore
        if self._started:
            print("[TASK] DriveToPointTask completed")
            return TaskStatus.DONE
            
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        if self.drive_to_point:
            self.drive_to_point.running = False
        print("[TASK] DriveToPointTask stopped")
        print("[TASK] DriveToPointTask stopped")
