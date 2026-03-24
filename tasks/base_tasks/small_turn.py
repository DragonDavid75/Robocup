# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class SmallTurnTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, velocity=0.2, action="STRAIGHT"):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.velocity = velocity
        self.action = action

    def start(self):
        super().start()
        print("[TASK] SmallTurnTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            # Start following the line
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(5.0, self.velocity, self.action)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] SmallTurnTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] SmallTurnTask stopped")
