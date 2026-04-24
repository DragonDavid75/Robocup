#First one of the tasks
#Goes to the rounabout, turns right and follows the roundabout until it can exit, 
# then turns right again to exit the roundabout and continues on the white line for 25 cm before stopping
# tasks/advanced_tasks/start_roundabout.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time
import math


class StartRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] StartRoundaboutTask started")
        self.state = 0

    def update(self) -> int:

        # STATE 0: Follow white line until first intersection
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_until_intersection_or_end_line(0.35)
            self.state = 1

        # STATE 1: Wait for intersection detection
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached, turning left")
                self.state = 2

        # STATE 2: Turn left at intersection
        elif self.state == 2:
            self.motion_controller.turn_in_place(math.radians(30))
            self.state = 3

        # STATE 3: Wait for turn to finish
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Turning complete, following line to end")
                self.state = 4

        # STATE 4: Follow line until it ends
        elif self.state == 4:
            self.motion_controller.follow_until_line_loss(0.2)
            self.state = 5

        # STATE 5: Wait for line end detection
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] Entering roundabout")
                self.state = 6

        # STATE 6: Enter roundabout with a small right arc
        elif self.state == 6:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(45),
                radius=0.20,
                linear_speed=0.09
            )
            self.state = 7

        # STATE 7: Wait for entry arc to finish
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving on roundabout")
                self.state = 8

        # STATE 8: Small arc to align on roundabout
        elif self.state == 8:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(15),
                radius=0.30,
                linear_speed=0.09
            )
            self.state = 9

        # STATE 9: Wait for alignment arc
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] First 90 turn on roundabout")
                self.state = 10

        # STATE 10: First 90 RIGHT turn (follow roundabout)
        elif self.state == 10:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(90),
                radius=0.39, #0.375
                linear_speed=0.2
            )
            self.state = 11

        # STATE 11: Wait for first 90 turn
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                print("[TASK] Second 90 turn on roundabout")
                self.state = 12

        # STATE 12: Second 90 RIGHT turn
        elif self.state == 12:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(100),
                radius=0.380,  #0.375
                linear_speed=0.2
            )
            self.state = 13

        # STATE 13: Wait for second 90 turn
        elif self.state == 13:
            if not self.motion_controller.is_busy():
                print("[TASK] Turning 90 right to exit")
                self.state = 14

        # STATE 14: Turn 90 RIGHT to face exit direction
        elif self.state == 14:
            self.motion_controller.turn_in_place(math.radians(90))
            self.state = 15

        # STATE 15: Wait for turn to finish
        elif self.state == 15:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving forward 25 cm")
                self.state = 16

        # STATE 16: Drive forward 25 cm
        elif self.state == 16:
            self.motion_controller.drive_distance(0.25, 0.08)
            self.state = 17

        # STATE 17: Wait for forward motion to finish
        elif self.state == 17:
            if not self.motion_controller.is_busy():
                print("[TASK] StartRoundaboutTask completed")
                return TaskStatus.DONE
        #TO DO - add 20 cm straight from the next task
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] StartRoundaboutTask stopped")



