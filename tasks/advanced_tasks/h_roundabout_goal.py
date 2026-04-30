"""
tasks/advanced_tasks/h_roundabout_goal.py

NOW Start: On the line before the roundabout, arm up and open.
NOW Functionality:  Follow line until it ends. Do roundabout through gates(2,3) and exit(d). Go the intersection(b), turn left (either turn left or go forward, turn around and turn right), and go to goal(c).
                    Also handels intersection being crossed to late or too early after exiting the roundabout, by checking the elapsed time and if it is too long, it will do a backup plan to find the line again and go to the goal.
End: Win!

                SHOULD Start: After 1m, after the stairs. Arm up and open.
                SHOULD Functionality: Go to intersection(n1), turn left, follow line until it ends. Do roundabout through gates(2,3) and exit(d). Go the intersection(b), turn left (either turn left or go forward, turn around and turn right), and go to goal(c).
                End: Win!

Status:
"""

from tasks.base_task import BaseTask, TaskStatus
import math
import time

class DriveEndTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] DriveEndTask started")
        self.state = 0

    def update(self):
        # STATE 0: Follow white line until first intersection
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.servo_controller.servo_control(2, 0, 300) #open gripper
            self.motion_controller.follow_until_line_loss(0.2)
            self.state = 1
    
        # STATE 1: Wait for intersection detection
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Entering roundabout")
                self.state = 2

        # STATE 2: Enter roundabout with a small right arc
        elif self.state == 2:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(75),
                radius=0.20,
                linear_speed=0.2
            )
            self.state = 3

        # STATE 3: Wait for entry arc to finish
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving on roundabout")
                self.state = 4

        # STATE 4: Small arc to align on roundabout
        elif self.state == 4:
            self.motion_controller.drive_arc(
                angle_rad=math.radians(-45),
                radius=0.20,
                linear_speed=0.09
            )
            self.state = 5

        # STATE 5: Wait for alignment arc
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] First 90° turn on roundabout")
                self.state = 6

        # STATE 6: First 90° turn on roundabout
        elif self.state == 6:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(90),
                radius=0.36,
                linear_speed=0.2
            )
            self.state = 7

        # STATE 7: Wait for first 90° turn
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] Second 90° turn on roundabout")
                self.state = 8

        # STATE 8: Second 90° turn on roundabout
        elif self.state == 8:
            self.motion_controller.drive_arc(
                angle_rad=-math.radians(80),
                radius=0.35,
                linear_speed=0.2
            )
            self.state = 9

        # STATE 9: Wait for second 90° turn
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] Turn on roundabout")
                self.state = 12

        # STATE 12: Turn 90 RIGHT to face exit direction
        elif self.state == 12:
            self.motion_controller.turn_in_place(math.radians(90))
            self.state = 13

        # STATE 13: Wait for turn to finish
        elif self.state == 13:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving forward")
                self.state = 14
        #STATE 14: Drive forward
        elif self.state == 14:
        ##########################################################################################    
            self.start = time.perf_counter() #TIMER STARTED
            self.motion_controller.drive_to_line(0.1) #SEARCING FOR LINE AFTER EXITING ROUNABOUT
            self.state = 15
        # STATE 15: Wait for line detection 
        elif self.state == 15:
            if not self.motion_controller.is_busy():
                self.elapsed_ms = (time.perf_counter() - self.start)*1000 #returns time in ms
                print(f"[TASK] Intersection reached, turning left, elapsed time: {self.elapsed_ms:.2f} ms")
                self.state = 16
        #WHAT IF IT DOES NOT EXIT THE ROUNABOUT IN THE RIGHT PLACE = takes too long
        # STATE 16: Follow line until intersection but check if the elapsed time is not too large
        elif self.state == 16:
            if (self.elapsed_ms > 5000): #5 seconds
                print(f"[TASK] Timeout reached, stop, elapsed time: {self.elapsed_ms:.2f} ms")
                self.state = 24
            else:
                self.motion_controller.follow_until_intersection_or_end_line(0.2)
                print(f"[TASK] Driving towards intersection, elapsed time: {self.elapsed_ms:.2f} ms")
                self.state = 17
        # STATE 17: Wait for forward drive towards intersection
        elif self.state == 17:
            if not self.motion_controller.is_busy():
                print("[TASK] Intersection reached, turning left")
                self.state = 18
        # STATE 18: Turning in intersection 
        elif self.state == 18:
            self.motion_controller.turn_in_place(math.radians(110))
            self.state = 19
        # STATE 19: Wait for final turn
        elif self.state == 19:
            if not self.motion_controller.is_busy():
                print("[TASK]  Turning complete, following line to end")
                self.state = 20
        # STATE 20: Follow line until it ends
        elif self.state == 20:
            #self.motion_controller.follow_until_line_loss(0.2)
            self.motion_controller.follow_for_distance(2.2, 0.4)
            self.state = 21
        elif self.state == 21:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving to end")
            time.sleep(0.5)
            self.state = 22
        # STATE 22: At the end, lower the arm 
        elif self.state == 22:
            self.servo_controller.servo_control(1, 200, 500) #lower arm
            self.state = 23
       #STATE 23: END
        elif self.state == 23:
            if not self.motion_controller.is_busy():
                print("[TASK] Reached the end")
                return TaskStatus.DONE
        #BACK UP STATE: if the robot does not exit the rounabout
        elif self.state == 24:
            print("[TASK] Drives 10 cm forward")
            self.motion_controller.drive_distance(0.1, 0.1) #after line found, drive forward a bit
            self.state = 25
        elif self.state == 25:
            if not self.motion_controller.is_busy():
                print("[TASK] Turns 70 degrees in place")
                self.motion_controller.turn_in_place(math.radians(90)) #turn in place
                time.sleep(0.5)
                self.state = 26
        elif self.state == 26:
            if not self.motion_controller.is_busy():
                print("[TASK] Wait for the turn")
                self.motion_controller.follow_for_distance(1, 0.2, "RIGHT") #go home for backup
                self.state = 23
        
 
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.robot.stop()
        print("[TASK] DriveEndTask stopped")