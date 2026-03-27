# tasks/drive_in.py

from tasks.base_task import BaseTask, TaskStatus
import time
import math 

class DriveInTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] DriveIn started")
        self.state = 0
        self.start_time = time.time()

    def update(self):

        # STEP 0: DRIVE FORWARD 20 cm
        if self.state == 0:
            print("[TASK] Driving forward 20 cm")
            self.motion_controller.drive_distance(0.001, 0.09)
            self.state = 1

        # WAIT FOR FORWARD TO FINISH
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Starting arc into roundabout")
                self.state = 2

        # STEP 1: ENTER ROUNDABOUT (RIGHT TURN)
        elif self.state == 2:
            print("[TASK] Entering roundabout (right turn)")
          
            angle_deg = 120  # 👈 now in degrees
            angle_rad = math.radians(angle_deg)  # 👈 convert

            self.motion_controller.drive_arc(
                angle_rad=angle_rad,   # 👈 NEGATIVE = right turn        # ~70 degrees (adjust if needed)
                linear_speed=0.09,
                angular_speed=0.30   # negative = right turn
            )
            self.state = 3

        # WAIT FOR ARC TO FINISH
        elif self.state == 3:
            if not self.motion_controller.is_busy():
            
                print("[TASK] DriveIn completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveIn stopped")
