"""
tasks/advanced_tasks/f_down_stairs.py

6th task: Down the stairs
Start: Facing the hole after dropping the ball. Arm down and open.
Functionality: Turn around, find the line, go to intersection(l). Take turn towards the stairs (either take rigth turn or turn around, go a bit further and take a left turn). Go down the stairs, and stop after 1m.
End: After 1m, after the stairs. Arm up and open.

Status:
"""

# tasks/vivek/drive_to_point_task.py

import math
from tasks.base_task import BaseTask, TaskStatus
import time
import cv2
import numpy as np
from mqtt_python.scam import cam
from mqtt_python.uservice import service


from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time

class DriveDownStairsSafe(BaseTask):
    def __init__(
        self,
        world,
        motion_controller,
        servo_controller,
    ):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] GolfBallsTask started")
        self.state = 0
        
    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, 200, 300)
            self.state = 1

        #servo arm down and open gripper
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached - open gripper and lower the arm if not already in that state")
                self.servo_controller.servo_control(1, -390, 300)
                self.servo_controller.servo_control(2, -200, 300)
                self.motion_controller.drive_distance(0.20, 0.2)
                self.state = 3
        
        #follow until line
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right on the intersection")
                self.motion_controller.drive_to_line(0.1)
                self.state = 4

        # Step 3: turn left 90
        elif self.state == 4:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right")
                self.motion_controller.turn_in_place(math.radians(-60))
                self.state = 8

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right on the intersection")
                self.motion_controller.follow_until_intersection_or_end_line(0.4)
                self.state = 6

        elif self.state == 6:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right")
                self.motion_controller.turn_in_place(math.radians(-150))
                self.state = 7

        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right")
                self.motion_controller.drive_distance(0.15, 0.2)
                self.state = 8

        elif self.state == 8:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right on the intersection")
                self.servo_controller.servo_control(2, -200, 500)
                self.motion_controller.follow_until_intersection_or_end_line(0.2)
                self.state = 9

        #interseection before stairs reached, turn left by 90
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right")
                self.motion_controller.turn_in_place(math.radians(90))
                self.state = 10

        elif self.state == 10:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right on the intersection")
                # self.motion_controller.follow_for_distance(0.25,0.2,action="STRAIGHT")
                self.motion_controller.follow_for_distance(0.24, 0.25)
                self.servo_controller.servo_control(1, -500, 300)
                self.state = 11
            
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveToPoint completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveToPoint stopped")
