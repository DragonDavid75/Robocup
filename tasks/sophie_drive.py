# tasks/drive_one_meter.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose
import time


class DriveSophieTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0
        self.start_time = 0

    def start(self):
        super().start()
        print("[TASK] DriveOneMeter started")
        self.state = 0
        self.motion_controller.robot.set_servo(1, -500, 300)
        self.start_time = time.time()


    def update(self):
        if self.state == 0:
            print("[TASK] Turning 90 degrees left")
            self.motion_controller.turn_in_place(1.57)  # π/2 radians
            self.state = 1

        # WAIT FOR TURN TO FINISH
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2
	
        elif self.state == 2:
            # Start driving
            self.motion_controller.drive_distance(0.5, 0.05)
            self.state = 3

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                self.state = 4
       
       elif self.state == 4:
            print("[TASK] Turning RIGHT 90°")
            self.motion_controller.turn_in_place(-1.57)
            self.state = 5

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.state = 6

        #DRIVE 1 meter
        elif self.state == 6:
            print("[TASK] Driving 1.0 m")
            self.motion_controller.drive_distance(1.0, 0.05)
            self.state = 7

        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveSophie completed")
                return TaskStatus.DONE
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] DriveSophie stopped")
