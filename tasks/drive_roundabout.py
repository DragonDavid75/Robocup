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

        # STATE 0: Enter roundabout with a small right arc
        if self.state == 0:
            print("[TASK] Entering roundabout")
            self.motion_controller.drive_arc(
                angle_rad=math.radians(45),   # slight right turn to enter
                radius=0.30,                  # tight entry curve
                linear_speed=0.09             # safe speed
            )
            self.state = 1

        # STATE 1: Wait for entry arc to finish
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving on roundabout")
                self.state = 2

        # STATE 2: Small arc to align on roundabout
        elif self.state == 2:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(15),   # small adjustment
                radius=0.30,
                linear_speed=0.09
            )
            self.state = 3

        # STATE 3: Wait for alignment arc
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] First 90° turn on roundabout")
                self.state = 4

        # STATE 4: First 90° RIGHT turn (follow roundabout)
        elif self.state == 4:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(90),  # RIGHT turn
                radius=0.375,                 # smoother curve
                linear_speed=0.2
            )
            self.state = 5

        # STATE 5: Wait for first 90° turn
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] Second 90° turn on roundabout")
                self.state = 6

        # STATE 6: Second 90° RIGHT turn (ADDED)
        elif self.state == 6:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(90),  # RIGHT turn again
                radius=0.375,
                linear_speed=0.2
            )
            self.state = 7

        # STATE 7: Wait for second 90° turn
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveRoundaboutTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] DriveRoundaboutTask stopped")

