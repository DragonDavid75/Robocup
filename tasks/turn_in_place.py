# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class TurnInPlaceTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] TurnInPlace started")
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Start turning
            self.motion_controller.servo_control(1, -800, 300)  # Example servo control
            self.motion_controller.turn_in_place(1.57 * 4)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2

        elif self.state == 2:
            # Wait until fully stopped
            if not self.motion_controller.is_busy():
                print("[TASK] TurnInPlace completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] TurnInPlace stopped")