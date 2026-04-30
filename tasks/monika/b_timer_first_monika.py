#Second of the tasks
#Starts right after the rounabout, goes to the timer, 
# turns around and goes back to the white line, 
# then continues on the white line for 9.5 m before stopping
# tasks/timer_first.py

import math

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class TimerFirstTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] TimerFirstTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(0.2, 0.2)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(-1.65)
                self.state = 2

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_distance(2.7, 0.4)
                self.state = 3
        #Turn around to go away from timer back to the white line
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(3.0) #2.9
                self.state = 4
        #Go to the white line
        elif self.state == 4:
            if not self.motion_controller.is_busy():
                self.motion_controller.drive_to_line(0.4)
                self.state = 5
        #In the junction turn left to continue on the white line
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(-1.57)
                self.state = 6
        #Go 9.5 m forward and turn left in the next junction, then continue 60 cm forward
        elif self.state == 6:
            if not self.motion_controller.is_busy():
                self.motion_controller.follow_until_intersection_or_end_line(0.6)
                self.state = 7
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached, turning left")
                self.state = 8
        elif self.state == 8:
            self.motion_controller.turn_in_place(math.radians(90)) #TURN
            self.state = 9
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] Turn completed, driving forward 0.5 m")
                self.state = 10
        elif self.state == 10:
            print("[TASK] Driving forward 0.5 m")
            self.motion_controller.follow_for_distance(0.6, 0.6)
            self.state = 11
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, 200, 500) #loewer arm
                self.servo_controller.servo_control(2, 0, 300)
                time.sleep(0.5)
                print("[TASK] TimerFirstTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] TimerFirstTask stopped")

