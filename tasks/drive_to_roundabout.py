# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from spose import pose
import time


class DriveToRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] DriveToRoundabout started")
        pose.tripBreset()
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Start driving
            self.motion_controller.follow_until_intersection(0.5)
            while self.motion_controller.current_task == 'line':
                time.sleep(0.1)  # Wait until the line following task is active
            
            # Turn left at the intersection
            

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
