"""
tasks/advanced_tasks/f_down_stairs.py

6th task: Down the stairs
Start: Away from the hole after dropping the ball. Arm down and open.
Functionality: Find the line, go to intersection(l). Take turn towards the stairs (either take right turn or turn around, go a bit further and take a left turn). Go down the stairs, reach intersection, turn left, drive 20 cm.
End: After 20cm, after the intersection after stairs. Arm up and open.

Status:
"""
import math
import time

from tasks.base_task import BaseTask, TaskStatus


class DriveDownStairs(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] Simple DriveDownStairs started")
        self.state = 0

    def update(self):
        # Step 0: drive to the first intersection
        if self.state == 0:
            print("[TASK] Driving away from the hole")
            self.motion_controller.drive_to_line(0.1)
            self.state = 1
        elif self.state == 1:
            print("[TASK] turn left on the line")
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(math.radians(90))
                self.state = 2
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] follow until intersection")
                self.motion_controller.follow_until_intersection_or_end_line(0.1)
                self.state = 3
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] turn right before stairs")
                self.motion_controller.turn_in_place(math.radians(-90))
                self.state = 4
        elif self.state == 4:
            if not self.motion_controller.is_busy():
                print("[TASK] drive forward to exit intersection")
                self.motion_controller.drive_distance(0.2, 0.1)
                self.state = 5
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] drive forward")
                self.motion_controller.follow_until_intersection_or_end_line(0.2)
                self.state = 6
        elif self.state == 6:
            if not self.motion_controller.is_busy():
                print("[TASK] turn left at the intersection before the roundabout")
                self.motion_controller.turn_in_place(math.radians(90))
                self.state = 7
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] drive a bit to exit the intersection")
                self.motion_controller.follow_for_distance(0.2, 0.1)
                self.state = 8
        elif self.state == 8:
            if not self.motion_controller.is_busy():
                print("[TASK] lift the arm before the exit rounabout task")
                self.servo_controller.servo_control(1, -500, 300)
                time.sleep(1.5)
                self.state = 9
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveDownStairs completed")
                return TaskStatus.DONE
               
        return TaskStatus.RUNNING

    def stop(self):
            super().stop()
            self.motion_controller.stop()
            print("[TASK] DriveDownStairs stopped")
