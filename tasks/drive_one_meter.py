# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from spose import pose
import time


class DriveOneMeterTask(BaseTask):

    def __init__(self, world, robot):
        super().__init__(world, robot)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] DriveOneMeter started")
        pose.tripBreset()
        self.robot.set_led(0, 100, 0)  # green
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Start driving
            self.world.set_motion(0.2, 0.0)
            self.robot.set_servo(1, -800, 300)
            self.state = 1

        elif self.state == 1:
            # Check distance or timeout
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                self.world.set_motion(0.0, 0.0)
                self.robot.set_servo(1, 0, 0)
                self.state = 2

        elif self.state == 2:
            # Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                print("[TASK] DriveOneMeter completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.robot.set_led(0, 0, 0)
        print("[TASK] DriveOneMeter stopped")
