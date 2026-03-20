# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveDistLineTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, distance=1.0, velocity=0.2, left_side=True, ref_pos=0.0):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.distance = distance
        self.velocity = velocity
        self.left_side = left_side
        self.ref_pos = ref_pos

    def start(self):
        super().start()
        print("[TASK] DriveDistLineTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # Start following the line
            self.motion_controller.follow_for_distance(self.distance, self.velocity, self.left_side, self.ref_pos)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveDistLineTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveDistLineTask stopped")
