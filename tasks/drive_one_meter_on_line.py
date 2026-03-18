# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveOneMeterOnLineTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] DriveOneMeterOnLine started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # Start following the line
            self.motion_controller.follow_for_distance(150.0, 0.2, left_side=False, ref_pos=0.0)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2

        elif self.state == 3:
            # Wait until fully stopped
            if not self.motion_controller.is_busy():
                print("[TASK] DriveOneMeterOnLine completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveOneMeterOnLine stopped")
