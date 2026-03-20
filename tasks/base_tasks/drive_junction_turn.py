# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveJunctionTurnTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, velocity=0.2, left_side=True, ref_pos=0.0, turn_angle=1.57):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.distance = distance
        self.velocity = velocity
        self.left_side = left_side
        self.ref_pos = ref_pos
        self.turn_angle = turn_angle

    def start(self):
        super().start()
        print("[TASK] DriveJunctionTurnTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # Start following the line
            self.motion_controller.follow_until_intersection_or_end_line(self.velocity, self.left_side, self.ref_pos)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(0.1, 0.2)
                self.state = 2

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(self.turn_angle)
                self.state = 3

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveJunctionTurnTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveJunctionTurnTask stopped")
