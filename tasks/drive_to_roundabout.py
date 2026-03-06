# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveToRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] DriveToRoundabout started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # 1. Start following the line
            self.motion_controller.follow_until_intersection_or_end_line(0.25, left_side=True, ref_pos=0.0)
            self.state = 1

        elif self.state == 1:
            # 2. Wait for line following to finish
            if not self.motion_controller.is_busy():
                # Line ended, now start driving into roundabout
                self.state = 2

        elif self.state == 2:
            # 3. Wait for the distance drive to finish
            if not self.motion_controller.is_busy():
                self.state = 3

        elif self.state == 3:
            # 4. Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                print("[TASK] DriveToRoundabout completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveToRoundabout stopped")
