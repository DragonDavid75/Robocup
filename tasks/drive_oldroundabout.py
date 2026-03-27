# tasks/drive_roundabout.py

from tasks.base_task import BaseTask, TaskStatus
import math


class DriveRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] DriveRoundaboutTask started")
        self.state = 0

    def update(self):

        # STATE 0: Enter roundabout with a right arc
        if self.state == 0:
            print("[TASK] Entering roundabout")
            self.motion_controller.drive_arc(
                angle_rad=math.radians(45),   # right turn
                radius=0.30,                    # tighter curve
                linear_speed=0.09              # safe speed
            )
            self.state = 1

        # STATE 1: Wait for entry arc to finish
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving on roundabout")
                self.state = 2

        # STATE 2: Continue around the roundabot
        elif self.state == 2:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(15),    # continue around
                radius=0.30,                    # slightly wider curve
                linear_speed=0.09
            )
            self.state = 3
        

        # STATE 3: Wait for first roundabout arc
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Second roundabout arc")
                self.state = 4

        # STATE 4: Second arc around roundabout
        elif self.state == 4:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(180),
                radius=0.45,
                linear_speed=0.07
            )
            self.state = 5

        # STATE 5: Wait for second roundabout arc
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveRoundaboutTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] DriveRoundaboutTask stopped")



