# tasks/drive_one_left.py

from tasks.base_task import BaseTask, TaskStatus
from tasks.robot_primitives import drive, rotate, wait_until_stopped, record_pose


class DriveOneLeftTask(BaseTask):

    def __init__(self, world, robot):
        super().__init__(world, robot)
        self.state = 0
        self.start_x = 0.0
        self.start_y = 0.0

    def start(self):
        super().start()
        print("[TASK] DriveOneLeft started")
        self.robot.set_led(0, 100, 0, 0)  # green
        self.start_x, self.start_y, _ = record_pose(self.world)

    def update(self):
        if self.state == 0:
            self.world.set_motion(0.2, 0.0)
            self.robot.set_servo(1, -800, 300)
            self.state = 1

        elif self.state == 1:
            done, self.start_x, self.start_y = drive(self.world, self.robot, 1.0, start_x=self.start_x, start_y=self.start_y)
            if done:
                self.state = 2

        elif self.state == 2:
            if wait_until_stopped():
                self.state = 3

        elif self.state == 3:
            self.world.set_motion(0.0, 1.5)
            self.state = 4

        elif self.state == 4:
            import time
            time.sleep(1.0)  # ~90 degrees at 1.5 rad/s
            self.world.set_motion(0.0, 0.0)
            self.start_x, self.start_y, _ = record_pose(self.world)
            self.state = 5

        elif self.state == 5:
            if wait_until_stopped():
                self.state = 6

        elif self.state == 6:
            self.world.set_motion(0.2, 0.0)
            self.robot.set_servo(1, -800, 300)
            self.state = 7

        elif self.state == 7:
            done, self.start_x, self.start_y = drive(self.world, self.robot, 0.5, start_x=self.start_x, start_y=self.start_y)
            if done:
                self.state = 8

        elif self.state == 8:
            if wait_until_stopped():
                print("[TASK] DriveOneLeft completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.robot.set_led(0, 0, 0, 0)
        print("[TASK] DriveOneLeft stopped")
