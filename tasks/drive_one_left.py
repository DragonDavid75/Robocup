# tasks/drive_one_left.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveOneLeftTask(BaseTask):

    def __init__(self, world, robot):
        super().__init__(world, robot)
        self.state = 0
        self.start_time = 0
        self.forward_dist = 1.0  # meters
        self.left_dist = 0.5     # meters

    def start(self):
        super().start()
        print("[TASK] DriveOneLeft started")
        pose.tripBreset()
        self.robot.set_led(0, 100, 0)  # green
        self.state = 0
        self.start_time = time.time()

    def update(self):

        if self.state == 0:
            # Drive forward 1 meter
            self.world.set_motion(0.2, 0.0)
            self.robot.set_servo(1, -800, 300)
            self.state = 1

        elif self.state == 1:
            # Check forward distance
            if pose.tripB > self.forward_dist or pose.tripBtimePassed() > 15:
                self.world.set_motion(0.0, 0.0)
                self.robot.set_servo(1, 0, 0)
                pose.tripBreset()  # Reset for next leg
                self.state = 2

        elif self.state == 2:
            # Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                self.state = 3

        elif self.state == 3:
            # Turn 90 degrees left (negative turn rate)
            self.world.set_motion(0.0, 1.5)  # turn left
            self.state = 4

        elif self.state == 4:
            # Wait for turn (approximately 90 degrees)
            time.sleep(1.0)  # ~90 degrees at 1.5 rad/s
            self.world.set_motion(0.0, 0.0)
            pose.tripBreset()  # Reset for forward movement
            self.state = 5

        elif self.state == 5:
            # Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                self.state = 6

        elif self.state == 6:
            # Drive forward 0.5 meters (now heading left)
            self.world.set_motion(0.2, 0.0)
            self.robot.set_servo(1, -800, 300)
            self.state = 7

        elif self.state == 7:
            # Check left distance (now using tripB for forward in new direction)
            if pose.tripB > self.left_dist or pose.tripBtimePassed() > 10:
                self.world.set_motion(0.0, 0.0)
                self.robot.set_servo(1, 0, 0)
                self.state = 8

        elif self.state == 8:
            # Wait until fully stopped
            if abs(pose.velocity()) < 0.001:
                print("[TASK] DriveOneLeft completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.robot.set_led(0, 0, 0)
        print("[TASK] DriveOneLeft stopped")
