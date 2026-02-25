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
            # Follow the line until the end of the line (start of the roundabout)
            self.motion_controller.follow_until_end_of_line(0.5, left_side=True, ref_pos=0.0)
            while self.motion_controller.current_task != None:
                time.sleep(0.1)  # Wait until the line following task is active
            self.state = 1

        elif self.state == 1:
            # Drive 1 meter to enter the roundabout
            self.motion_controller.drive_distance(1.0, 0.5)
            while self.motion_controller.current_task != None:
                time.sleep(0.1)  # Wait until the drive distance task is active
            self.state = 2

        elif self.state == 2:
            # Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                print("[TASK] DriveToRoundabout completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.robot.set_led(0, 0, 0)
        print("[TASK] DriveToRoundabout stopped")
