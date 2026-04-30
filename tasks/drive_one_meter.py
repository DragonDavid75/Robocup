# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveOneMeterTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] DriveOneMeter started")
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Start driving
            self.motion_controller.drive_distance(2.2, -0.2)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2

        elif self.state == 2:
            # Wait until fully stopped
            if not self.motion_controller.is_busy():
                print("[TASK] DriveOneMeter completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveOneMeter stopped")
