# tasks/DriveCircle.py

from tasks.base_task import BaseTask, TaskStatus
import math


class DriveCircleTask(BaseTask):

    def __init__(self, world, motion_controller):
        """
        Constructor of the task.

        world:
            Reference to the global world model.

        motion_controller:
            Object responsible for robot motion.
        """
        super().__init__(world, motion_controller)

        # State machine
        # 0 = start circle
        # 1 = wait until finished
        self.state = 0

        # Desired circle radius (meters)
        self.radius = 0.40

        # Linear velocity (m/s)
        self.linear_speed = 0.05

        # Angular velocity (rad/s)
        # v = r * omega  ->  omega = v / r
       # self.angular_speed = self.linear_speed / self.radius

        # Circumference of the circle
        #self.circumference = 2 * math.pi * self.radius

        # Time needed to complete the circle
        #self.duration = self.circumference / self.linear_speed 

    def start(self):
        """
        Called once when the task begins.
        """
        super().start()

        print("[TASK] DriveCircle started")
        self.state = 0

    def update(self): 
        # Linear velocity (m/s)
        self.linear_speed = 0.05

        # Angular velocity (rad/s)
        # v = r * omega  ->  omega = v / r
   #     self.angular_speed = self.linear_speed / self.radius
        self.angular_speed = 0.0856
        # Circumference of the circle
        self.circumference = 2 * math.pi * self.radius

        # Time needed to complete the circle
        self.duration = self.circumference / self.linear_speed

        """
        Called repeatedly by the mission manager.
        """

        # STATE 0: Start circular motion
        if self.state == 0:

            self.motion_controller.drive_circle(
                self.linear_speed,
                self.angular_speed,
                self.duration
            )

            self.state = 1

        # STATE 1: Wait until motion controller finishes
        elif self.state == 1:

            if not self.motion_controller.is_busy():
                print("[TASK] DriveCircle completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        """
        Called if the mission manager aborts the task.
        """
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] DriveCircle stopped")

